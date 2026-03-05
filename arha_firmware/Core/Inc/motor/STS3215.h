#ifndef STS3215_GRIPPER_H_
#define STS3215_GRIPPER_H_

#include <stdint.h>

/* ============================================================
 * STS3215 Gripper Driver
 *
 * Single-motor gripper using a Feetech STS3215 12V servo.
 * Protocol : SMS1.0 TTL half-duplex UART, 8-N-1
 * Byte order: low byte first (STS/SMS series)
 * Default baud rate: 1,000,000 bps
 *
 * Control strategy:
 *   - Mode 0 (position servo) with a torque limit applied.
 *   - The gripper closes toward GRIPPER_POS_CLOSED until it
 *     stalls against an object.  The torque limit prevents
 *     crushing.  Once stalled the servo holds position and
 *     keeps squeezing at the set torque.
 * ============================================================ */

/* ============================================================
 * Hardware Configuration — adjust to match your setup
 * ============================================================ */

/** ID assigned to the gripper motor on the bus. */
#define GRIPPER_SERVO_ID          1

/**
 * @brief Fully open position (encoder counts, 0–4095).
 * 2048 = 180° neutral.  Adjust to match your mechanical range.
 */
#define GRIPPER_POS_OPEN          1024   /* ~90°  */

/**
 * @brief Fully closed / maximum squeeze position.
 * The motor will stall before reaching this if an object is held.
 * Set slightly past the hard-close point so it always stalls on
 * the object rather than on the mechanical end-stop.
 */
#define GRIPPER_POS_CLOSED        3072   /* ~270° */

/**
 * @brief Default closing speed (encoder steps per second).
 * Lower = slower and gentler.  0 = maximum speed.
 * Recommended range for a gripper: 100–500.
 */
#define GRIPPER_DEFAULT_SPEED     200

/**
 * @brief Default torque limit as a percentage of stall torque (0–100).
 * This caps the squeeze force.  At 12 V stall is ~30 kg·cm.
 *   20% ≈  6 kg·cm  — suitable for delicate objects
 *   40% ≈ 12 kg·cm  — general purpose
 *   60% ≈ 18 kg·cm  — firm grip
 * The driver converts this percentage to the 0–1000 register scale.
 */
#define GRIPPER_DEFAULT_TORQUE_PCT  30    /* 30% = ~9 kg·cm @ 12 V */

/* ============================================================
 * STS3215 Register Map (SMS1.0, low byte first)
 * ============================================================ */

/* --- EPROM region (requires unlock before writing) --- */
#define STS_ADDR_FIRMWARE_VER       0x00  /* 2 bytes, read-only          */
#define STS_ADDR_ID                 0x05  /* 1 byte                      */
#define STS_ADDR_BAUD_RATE          0x06  /* 1 byte, see BAUD_RATE_INDEX */
#define STS_ADDR_RETURN_DELAY       0x07  /* 1 byte, × 2 µs              */
#define STS_ADDR_RESPONSE_LEVEL     0x08  /* 1 byte, 0=always respond    */
#define STS_ADDR_MIN_ANGLE          0x09  /* 2 bytes                     */
#define STS_ADDR_MAX_ANGLE          0x0B  /* 2 bytes                     */
#define STS_ADDR_MAX_TEMP           0x0D  /* 1 byte, °C                  */
#define STS_ADDR_MAX_VOLTAGE        0x0E  /* 1 byte, × 0.1 V             */
#define STS_ADDR_MIN_VOLTAGE        0x0F  /* 1 byte, × 0.1 V             */
#define STS_ADDR_MAX_TORQUE         0x10  /* 2 bytes, 0–1000             */
#define STS_ADDR_UNLOAD_COND        0x13  /* 1 byte, protection bit mask */
#define STS_ADDR_LED_ALARM          0x14  /* 1 byte, alarm bit mask      */
#define STS_ADDR_PID_P              0x15  /* 1 byte, position Kp         */
#define STS_ADDR_PID_D              0x16  /* 1 byte, position Kd         */
#define STS_ADDR_PID_I              0x17  /* 1 byte, position Ki         */
#define STS_ADDR_CW_DEADBAND        0x1A  /* 1 byte                      */
#define STS_ADDR_CCW_DEADBAND       0x1B  /* 1 byte                      */
#define STS_ADDR_OCP_THRESHOLD      0x1C  /* 2 bytes, × 6.5 mA           */
#define STS_ADDR_OPERATING_MODE     0x21  /* 1 byte, 0/1/2/3             */
#define STS_ADDR_PROTECT_TORQUE     0x22  /* 2 bytes, % of stall         */
#define STS_ADDR_OVERLOAD_TORQUE    0x24  /* 1 byte, % of stall          */
#define STS_ADDR_SPEED_KP           0x25  /* 1 byte, speed loop Kp       */
#define STS_ADDR_OVERLOAD_TIME      0x26  /* 2 bytes, × 40 ms            */
#define STS_ADDR_OCP_TIME           0x2A  /* 2 bytes, × 40 ms            */
#define STS_ADDR_EPROM_LOCK         0x37  /* 1 byte, 1=locked 0=unlocked */

/* --- SRAM region (no unlock needed, volatile) --- */
#define STS_ADDR_TORQUE_ENABLE      0x28  /* 1 byte, 0=off 1=on 128=cal */
#define STS_ADDR_ACCELERATION       0x29  /* 1 byte, 0=max              */
#define STS_ADDR_GOAL_POSITION      0x2A  /* 2 bytes, 0–4095            */
#define STS_ADDR_GOAL_TIME          0x2C  /* 2 bytes, ms (open-loop)    */
#define STS_ADDR_GOAL_SPEED         0x2E  /* 2 bytes, steps/s, 0=max    */
#define STS_ADDR_TORQUE_LIMIT       0x30  /* 2 bytes, 0–1000            */
#define STS_ADDR_PRESENT_POSITION   0x38  /* 2 bytes, read-only         */
#define STS_ADDR_PRESENT_SPEED      0x3A  /* 2 bytes, read-only, signed */
#define STS_ADDR_PRESENT_LOAD       0x3C  /* 2 bytes, read-only 0–1000  */
#define STS_ADDR_PRESENT_VOLTAGE    0x3E  /* 1 byte,  read-only × 0.1 V */
#define STS_ADDR_PRESENT_TEMP       0x3F  /* 1 byte,  read-only °C      */

/* ============================================================
 * Protocol Instruction Codes
 * ============================================================ */
#define STS_PING                    0x01
#define STS_READ_DATA               0x02
#define STS_WRITE_DATA              0x03
#define STS_REG_WRITE               0x04
#define STS_ACTION                  0x05
#define STS_RESET                   0x06
#define STS_SYNC_READ               0x82
#define STS_SYNC_WRITE              0x83

#define STS_BROADCAST_ID            0xFE

/* ============================================================
 * Baud Rate Index (written to STS_ADDR_BAUD_RATE)
 * ============================================================ */
typedef enum {
    STS_BAUD_1M    = 0,
    STS_BAUD_500K  = 1,
    STS_BAUD_250K  = 2,
    STS_BAUD_128K  = 3,
    STS_BAUD_115K  = 4,
    STS_BAUD_76K8  = 5,
    STS_BAUD_57K6  = 6,
    STS_BAUD_38K4  = 7,
} STS_BaudRate;

/* ============================================================
 * Operating Modes (written to STS_ADDR_OPERATING_MODE)
 * ============================================================ */
typedef enum {
    STS_MODE_POSITION    = 0,  /**< Default. Absolute angle 0–4095.       */
    STS_MODE_SPEED_CL    = 1,  /**< Speed closed-loop (motor mode).       */
    STS_MODE_SPEED_OL    = 2,  /**< Speed open-loop (motor mode).         */
    STS_MODE_STEP        = 3,  /**< Step / incremental move.              */
} STS_OperatingMode;

/* ============================================================
 * Return Status
 * ============================================================ */
typedef enum {
    STS_OK           = 0,  /**< Success, no error.                        */
    STS_ERR_FRAME    = 1,  /**< Bad header or checksum in response.       */
    STS_ERR_MOTOR    = 2,  /**< Motor reported error (ERROR byte != 0).   */
    STS_ERR_TIMEOUT  = 3,  /**< No response received within timeout.      */
} STS_Status;

/* ============================================================
 * Real-time Status Struct
 * ============================================================ */

/**
 * @brief Snapshot of the gripper motor's live feedback registers.
 *
 * Populated by GRIPPER_ReadStatus().
 */
typedef struct {
    uint16_t position;   /**< Encoder position, 0–4095 (2048 = 180°).    */
    int16_t  speed;      /**< Present speed, signed steps/s.             */
    uint16_t load;       /**< Output load, 0–1000 (= 0–100% of stall).   */
    float    voltage;    /**< Supply voltage in Volts.                    */
    uint8_t  temp_c;     /**< Motor temperature in °C.                   */
} GripperStatus;

/* ============================================================
 * HAL Callbacks — user must implement these two functions
 * ============================================================ */

/**
 * @brief Transmit bytes on the half-duplex TTL bus.
 *
 * Switch bus to TX mode, send @p len bytes from @p buf, then
 * switch back to RX mode.  Add ~100 µs delay after last byte
 * before releasing the bus.
 *
 * Example (STM32 HAL):
 *   HAL_HalfDuplex_EnableTransmitter(&huart1);
 *   HAL_UART_Transmit(&huart1, buf, len, 10);
 *   HAL_HalfDuplex_EnableReceiver(&huart1);
 *
 * @param buf  Data to send.
 * @param len  Number of bytes.
 */
void STS_UART_Transmit(const uint8_t *buf, uint8_t len);

/**
 * @brief Receive bytes from the half-duplex TTL bus.
 *
 * Block until @p len bytes arrive or a timeout occurs.
 *
 * Example (STM32 HAL):
 *   HAL_UART_Receive(&huart1, buf, len, 5);  // 5 ms timeout
 *
 * @param buf  Buffer for received bytes.
 * @param len  Expected number of bytes.
 * @return     Actual bytes received.
 */
uint8_t STS_UART_Receive(uint8_t *buf, uint8_t len);

/* ============================================================
 * Low-Level Protocol API
 * ============================================================ */

/**
 * @brief Build and send a raw instruction packet.
 */
void STS_SendPacket(uint8_t id, uint8_t instruction,
                    const uint8_t *params, uint8_t param_len);

/**
 * @brief Receive and validate a response packet.
 *
 * @param id         Expected servo ID.
 * @param out_data   Buffer for returned parameter bytes (may be NULL).
 * @param out_len    In: buffer capacity.  Out: bytes actually received.
 * @return           STS_OK on success.
 */
STS_Status STS_ReceivePacket(uint8_t id, uint8_t *out_data, uint8_t *out_len);

/**
 * @brief Read @p len bytes from @p address on servo @p id.
 */
STS_Status STS_ReadData(uint8_t id, uint8_t address,
                        uint8_t len, uint8_t *out);

/**
 * @brief Write @p len bytes to @p address on servo @p id.
 */
STS_Status STS_WriteData(uint8_t id, uint8_t address,
                         const uint8_t *data, uint8_t len);

/**
 * @brief Unlock the EPROM region so configuration registers can be saved.
 * @note  Always re-lock after writing with STS_EPROMLock().
 */
STS_Status STS_EPROMUnlock(uint8_t id);

/**
 * @brief Re-lock the EPROM region after writing configuration.
 */
STS_Status STS_EPROMLock(uint8_t id);

/* ============================================================
 * Gripper High-Level API
 * ============================================================ */

/**
 * @brief Initialise the gripper motor.
 *
 * Verifies the servo is present, sets operating mode to position
 * control (Mode 0), applies the default torque limit, and enables
 * torque output.  Call once at startup.
 *
 * @param torque_pct  Squeeze force limit as % of stall (1–100).
 *                    Pass 0 to use GRIPPER_DEFAULT_TORQUE_PCT.
 * @return            STS_OK if the motor responded and was configured.
 */
STS_Status GRIPPER_Init(uint8_t torque_pct);

/**
 * @brief Close the gripper.
 *
 * Moves toward GRIPPER_POS_CLOSED at @p speed.  The motor stalls
 * when it grips an object and holds at the torque limit.
 *
 * @param speed  Closing speed in steps/s (0 = use GRIPPER_DEFAULT_SPEED).
 * @return       STS_OK if command was accepted.
 */
STS_Status GRIPPER_Close(uint16_t speed);

/**
 * @brief Open the gripper fully.
 *
 * Moves toward GRIPPER_POS_OPEN at @p speed.
 *
 * @param speed  Opening speed in steps/s (0 = use GRIPPER_DEFAULT_SPEED).
 * @return       STS_OK if command was accepted.
 */
STS_Status GRIPPER_Open(uint16_t speed);

/**
 * @brief Move the gripper to a specific position.
 *
 * Useful for partially open positions (e.g. pre-shape before grasp).
 *
 * @param position  Target encoder position (0–4095).
 * @param speed     Speed in steps/s (0 = use GRIPPER_DEFAULT_SPEED).
 * @return          STS_OK if command was accepted.
 */
STS_Status GRIPPER_MoveTo(uint16_t position, uint16_t speed);

/**
 * @brief Update the torque (squeeze force) limit at runtime.
 *
 * Can be called while the gripper is holding an object to adjust grip
 * force without reopening.
 *
 * @param torque_pct  New torque limit as % of stall (1–100).
 * @return            STS_OK on success.
 */
STS_Status GRIPPER_SetTorque(uint8_t torque_pct);

/**
 * @brief Release torque — gripper goes limp and can be moved by hand.
 * @return STS_OK on success.
 */
STS_Status GRIPPER_Release(void);

/**
 * @brief Re-enable torque after a GRIPPER_Release().
 * @return STS_OK on success.
 */
STS_Status GRIPPER_EnableTorque(void);

/**
 * @brief Check whether the gripper is currently holding an object.
 *
 * Reads the present load register.  If load exceeds @p load_threshold
 * the gripper is considered to be stalled on an object.
 *
 * @param load_threshold  Load value (0–1000) above which a grasp is
 *                        detected.  Typical: 100–200 (10–20% of stall).
 * @return                1 = object detected, 0 = free / open.
 */
uint8_t GRIPPER_IsGripping(uint16_t load_threshold);

/**
 * @brief Read all live feedback registers in one transaction.
 *
 * @param out  Pointer to a GripperStatus struct to fill.
 * @return     STS_OK on success.
 */
STS_Status GRIPPER_ReadStatus(GripperStatus *out);

/**
 * @brief Ping the gripper motor to verify it is present on the bus.
 * @return STS_OK if the motor responds.
 */
STS_Status GRIPPER_Ping(void);

/**
 * @brief Perform a one-button centre calibration.
 *
 * Sets the current shaft position as the new 2048 (180°) centre and
 * saves it to EPROM.  Position the shaft at the desired mechanical
 * neutral before calling this.
 *
 * @return STS_OK on success.
 */
STS_Status GRIPPER_Calibrate(void);

/**
 * @brief Scan the bus for any servo and return its ID.
 *
 * Useful during initial setup to find a freshly unboxed servo
 * (factory default ID = 1).  Pings IDs 0–20.
 *
 * @param out_id  Pointer that will receive the found ID.
 * @return        STS_OK if a servo was found, STS_ERR_TIMEOUT if none.
 */
STS_Status GRIPPER_ScanBus(uint8_t *out_id);

/**
 * @brief Change the servo's ID (saved to EPROM).
 *
 * Handles EPROM unlock/lock automatically.
 *
 * @param current_id  The servo's current ID.
 * @param new_id      Desired new ID (1–253).
 * @return            STS_OK on success.
 */
STS_Status GRIPPER_SetID(uint8_t current_id, uint8_t new_id);

#endif /* STS3215_GRIPPER_H_ */