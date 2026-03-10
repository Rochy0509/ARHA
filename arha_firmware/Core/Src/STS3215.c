/**
 * @file    sts3215_gripper.c
 * @brief   STS3215 single-motor gripper driver.
 *
 * Control strategy
 * ----------------
 * Mode 0 (position servo) is used throughout.  A torque limit
 * is set in SRAM register 0x30 before every motion command so
 * the motor stalls — and holds — when it grips an object rather
 * than forcing through it.
 *
 * Half-duplex wiring note
 * -----------------------
 * TX and RX share a single wire.  The user-supplied
 * STS_UART_Transmit() must switch the bus direction pin before
 * and after sending, and must allow ~100 µs for the last byte
 * to clear before releasing the line to RX.
 *
 * Byte order
 * ----------
 * STS / SMS series: LOW byte first, then HIGH byte.
 * This differs from the SCS series (high byte first).
 */

#include "motor/STS3215.h"
#include <string.h>

/* ============================================================
 * Private helpers
 * ============================================================ */

/**
 * @brief Pack a uint16_t into two bytes, low byte first.
 */
static void _pack16(uint8_t *buf, uint16_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)(val >> 8);
}

/**
 * @brief Unpack two bytes (low byte first) into a uint16_t.
 */
static uint16_t _unpack16(const uint8_t *buf)
{
    return (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
}

/**
 * @brief Unpack two bytes into a signed int16_t.
 */
static int16_t _unpack16s(const uint8_t *buf)
{
    return (int16_t)_unpack16(buf);
}

/**
 * @brief Compute the packet checksum.
 *
 * checksum = ~(id + length + instruction + param_0 + ... + param_n)
 * Only the lowest byte is used.
 */
static uint8_t _checksum(uint8_t id, uint8_t length, uint8_t instruction,
                          const uint8_t *params, uint8_t param_len)
{
    uint8_t sum = id + length + instruction;
    for (uint8_t i = 0; i < param_len; i++) {
        sum += params[i];
    }
    return ~sum;
}

/**
 * @brief Convert a torque percentage (0–100) to the 0–1000 register scale.
 */
static uint16_t _pct_to_torque_reg(uint8_t pct)
{
    if (pct == 0)  pct = GRIPPER_DEFAULT_TORQUE_PCT;
    if (pct > 100) pct = 100;
    return (uint16_t)(pct * 10u);   /* 1% = 10 register units */
}

/* ============================================================
 * Low-Level Protocol
 * ============================================================ */

void STS_SendPacket(uint8_t id, uint8_t instruction,
                    const uint8_t *params, uint8_t param_len)
{
    uint8_t buf[128];
    uint8_t length = param_len + 2;   /* LEN = N_params + 2 */

    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = length;
    buf[4] = instruction;

    if (params != NULL && param_len > 0) {
        memcpy(&buf[5], params, param_len);
    }

    buf[5 + param_len] = _checksum(id, length, instruction,
                                   params, param_len);

    STS_UART_Transmit(buf, (uint8_t)(6 + param_len));
}

STS_Status STS_ReceivePacket(uint8_t id, uint8_t *out_data, uint8_t *out_len)
{
    /* Read fixed header: 0xFF 0xFF <ID> <LEN> */
    uint8_t header[4];
    if (STS_UART_Receive(header, 4) < 4)        return STS_ERR_TIMEOUT;
    if (header[0] != 0xFF || header[1] != 0xFF) return STS_ERR_FRAME;

    uint8_t length      = header[3];            /* LEN = n_params + 2    */
    uint8_t payload_len = length;               /* ERROR + params + csum */

    uint8_t payload[64];
    if (STS_UART_Receive(payload, payload_len) < payload_len)
        return STS_ERR_TIMEOUT;

    uint8_t error_byte    = payload[0];
    uint8_t recv_checksum = payload[payload_len - 1];
    uint8_t n_params      = (uint8_t)(payload_len - 2);  /* strip ERROR + csum */

    /* Validate that the response came from the servo we addressed */
    if (header[2] != id) return STS_ERR_FRAME;

    /* Validate checksum */
    uint8_t calc = _checksum(header[2], length, error_byte,
                             (n_params > 0) ? &payload[1] : NULL, n_params);
    if (calc != recv_checksum) return STS_ERR_FRAME;

    /* Motor-side error */
    if (error_byte != 0x00) return STS_ERR_MOTOR;

    /* Copy parameters to caller buffer */
    if (out_data != NULL && out_len != NULL && n_params > 0) {
        uint8_t copy = (n_params < *out_len) ? n_params : *out_len;
        memcpy(out_data, &payload[1], copy);
        *out_len = copy;
    }

    return STS_OK;
}

STS_Status STS_ReadData(uint8_t id, uint8_t address,
                        uint8_t len, uint8_t *out)
{
    uint8_t params[2] = { address, len };
    STS_SendPacket(id, STS_READ_DATA, params, 2);
    uint8_t actual = len;
    return STS_ReceivePacket(id, out, &actual);
}

STS_Status STS_WriteData(uint8_t id, uint8_t address,
                         const uint8_t *data, uint8_t len)
{
    uint8_t buf[64];
    buf[0] = address;
    memcpy(&buf[1], data, len);
    STS_SendPacket(id, STS_WRITE_DATA, buf, (uint8_t)(len + 1));

    if (id == STS_BROADCAST_ID) return STS_OK;   /* no response expected */
    uint8_t dummy = 0;
    return STS_ReceivePacket(id, NULL, &dummy);
}

STS_Status STS_EPROMUnlock(uint8_t id)
{
    uint8_t val = 0;
    return STS_WriteData(id, STS_ADDR_EPROM_LOCK, &val, 1);
}

STS_Status STS_EPROMLock(uint8_t id)
{
    uint8_t val = 1;
    return STS_WriteData(id, STS_ADDR_EPROM_LOCK, &val, 1);
}

/* ============================================================
 * Private Motion Helper
 * ============================================================ */

static STS_Status STS_RegWriteData(uint8_t id, uint8_t address,
                                   const uint8_t *data, uint8_t len)
{
    uint8_t buf[64];
    buf[0] = address;
    memcpy(&buf[1], data, len);
    STS_SendPacket(id, STS_REG_WRITE, buf, (uint8_t)(len + 1));

    if (id == STS_BROADCAST_ID) return STS_OK;   /* no response expected */
    uint8_t dummy = 0;
    return STS_ReceivePacket(id, NULL, &dummy);
}

static STS_Status STS_Action(uint8_t id)
{
    STS_SendPacket(id, STS_ACTION, NULL, 0);
    if (id == STS_BROADCAST_ID) return STS_OK;
    uint8_t dummy = 0;
    return STS_ReceivePacket(id, NULL, &dummy);
}

/**
 * @brief Write goal position and goal speed together in one transaction.
 *
 * Writes 6 bytes starting at STS_ADDR_GOAL_POSITION:
 *   [0–1] goal position  (low byte first)
 *   [2–3] goal time      (0 = not used in position mode)
 *   [4–5] goal speed     (0 = max speed)
 */
static STS_Status _move(uint8_t id, uint16_t position, uint16_t speed)
{
    uint8_t data[6];
    _pack16(&data[0], position);
    _pack16(&data[2], 0);        /* time field unused in mode 0 */
    _pack16(&data[4], speed);
    
    /* 1. Register the write command. The servo ACKs this BEFORE moving, 
     *    which protects the delicate UART line from startup current EMI. */
    STS_Status st = STS_RegWriteData(id, STS_ADDR_GOAL_POSITION, data, 6);
    if (st != STS_OK) return st;

    /* 2. Broadcast Action to trigger the buffered motion. 
     *    Broadcasts do not require an ACK, so we don't care about noise. */
    return STS_Action(STS_BROADCAST_ID);
}

/* ============================================================
 * Gripper High-Level API
 * ============================================================ */

STS_Status GRIPPER_Init(uint8_t torque_pct)
{
    STS_Status st;

    /* 1. Verify the motor is present */
    st = GRIPPER_Ping();
    if (st != STS_OK) return st;

    /* 2. (Removed) We NEVER write EPROM on every boot. 
     *    Writing EPROM halts the servo's CPU and causes flash wear.
     *    Consecutive EPROM writes without long delays will crash the servo.
     *    We assume the servo is in its factory default position mode. */

    /* 3. Apply the runtime torque limit (SRAM — no unlock needed) */
    st = GRIPPER_SetTorque(torque_pct);
    if (st != STS_OK) return st;

    /* 4. Apply a soft acceleration profile to prevent 12V inrush current 
     *    alarms (Overload Protection) when starting from a dead stop. */
    uint8_t acc = 50; 
    st = STS_WriteData(GRIPPER_SERVO_ID, STS_ADDR_ACCELERATION, &acc, 1);
    if (st != STS_OK) return st;

    /* 5. Enable torque output */
    st = GRIPPER_EnableTorque();
    return st;
}

STS_Status GRIPPER_Close(uint16_t speed)
{
    if (speed == 0) speed = GRIPPER_DEFAULT_SPEED;
    return _move(GRIPPER_SERVO_ID, GRIPPER_POS_CLOSED, speed);
}

STS_Status GRIPPER_Open(uint16_t speed)
{
    if (speed == 0) speed = GRIPPER_DEFAULT_SPEED;
    return _move(GRIPPER_SERVO_ID, GRIPPER_POS_OPEN, speed);
}

STS_Status GRIPPER_MoveTo(uint16_t position, uint16_t speed)
{
    if (speed == 0) speed = GRIPPER_DEFAULT_SPEED;
    return _move(GRIPPER_SERVO_ID, position, speed);
}

STS_Status GRIPPER_SetTorque(uint8_t torque_pct)
{
    uint16_t reg_val = _pct_to_torque_reg(torque_pct);
    uint8_t data[2];
    _pack16(data, reg_val);
    /* Write to SRAM torque limit (0x30) — no EPROM unlock needed */
    return STS_WriteData(GRIPPER_SERVO_ID, STS_ADDR_TORQUE_LIMIT, data, 2);
}

STS_Status GRIPPER_Release(void)
{
    uint8_t val = 0;
    return STS_WriteData(GRIPPER_SERVO_ID, STS_ADDR_TORQUE_ENABLE, &val, 1);
}

STS_Status GRIPPER_EnableTorque(void)
{
    uint8_t val = 1;
    return STS_WriteData(GRIPPER_SERVO_ID, STS_ADDR_TORQUE_ENABLE, &val, 1);
}

uint8_t GRIPPER_IsGripping(uint16_t load_threshold)
{
    uint8_t raw[2] = {0};
    if (STS_ReadData(GRIPPER_SERVO_ID, STS_ADDR_PRESENT_LOAD, 2, raw) != STS_OK)
        return 0;
    uint16_t load = _unpack16(raw);
    return (load >= load_threshold) ? 1 : 0;
}

STS_Status GRIPPER_ReadStatus(GripperStatus *out)
{
    if (out == NULL) return STS_ERR_FRAME;

    /*
     * Read 8 consecutive bytes starting at STS_ADDR_PRESENT_POSITION (0x38):
     *   [0–1] present position
     *   [2–3] present speed
     *   [4–5] present load
     *   [6]   present voltage
     *   [7]   present temperature
     */
    uint8_t raw[8] = {0};
    STS_Status st = STS_ReadData(GRIPPER_SERVO_ID,
                                 STS_ADDR_PRESENT_POSITION, 8, raw);
    if (st != STS_OK) return st;

    out->position = _unpack16(&raw[0]);
    out->speed    = _unpack16s(&raw[2]);
    out->load     = _unpack16(&raw[4]);
    out->voltage  = raw[6] * 0.1f;     /* register unit = 0.1 V */
    out->temp_c   = raw[7];

    return STS_OK;
}

STS_Status GRIPPER_Ping(void)
{
    STS_SendPacket(GRIPPER_SERVO_ID, STS_PING, NULL, 0);
    uint8_t dummy = 0;
    return STS_ReceivePacket(GRIPPER_SERVO_ID, NULL, &dummy);
}

STS_Status GRIPPER_Calibrate(void)
{
    /*
     * Writing 128 (0x80) to the Torque Enable register (0x28) triggers
     * the one-button centre calibration.  The servo saves the current
     * shaft position as the new 2048 (180°) zero point to EPROM.
     */
    uint8_t val = 128;
    return STS_WriteData(GRIPPER_SERVO_ID, STS_ADDR_TORQUE_ENABLE, &val, 1);
}

STS_Status GRIPPER_ScanBus(uint8_t *out_id)
{
    for (uint8_t id = 0; id <= 20; id++) {
        STS_SendPacket(id, STS_PING, NULL, 0);
        uint8_t dummy = 0;
        if (STS_ReceivePacket(id, NULL, &dummy) == STS_OK) {
            if (out_id != NULL) *out_id = id;
            return STS_OK;
        }
    }
    return STS_ERR_TIMEOUT;
}

STS_Status GRIPPER_SetID(uint8_t current_id, uint8_t new_id)
{
    STS_Status st;

    st = STS_EPROMUnlock(current_id);
    if (st != STS_OK) return st;

    st = STS_WriteData(current_id, STS_ADDR_ID, &new_id, 1);
    if (st != STS_OK) {
        STS_EPROMLock(new_id);   /* try to re-lock even on error */
        return st;
    }

    /* The servo now responds on new_id immediately */
    return STS_EPROMLock(new_id);
} 