/*
 * tcp_ip.c — ARHA TCP Server
 *
 * Binary protocol server over LwIP netconn API (FreeRTOS).
 * Every command receives a response: data payload or empty ACK.
 * Motor commands routed to motor_control.c (FDCAN).
 *
 * Frame format: [0xAA][CMD][LEN_LO][LEN_HI][PAYLOAD...][XOR_CS][0x55]
 *
 * Author: Kenneth Martinez
 */

#include "tcp/tcp_ip.h"
#include "motor/motor_control.h"
#include "motor/fdcan.h"
#include "motor/myactuator.h"
#include "motor/gripper_control.h"
#include "main.h"
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip/tcp.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* Map limb name string to index constant */
static uint8_t limb_name_to_index(const char *name, uint8_t len) {
    if (len == 8 && memcmp(name, "left_arm", 8) == 0) return LIMB_LEFT_ARM;
    if (len == 9 && memcmp(name, "right_arm", 9) == 0) return LIMB_RIGHT_ARM;
    if (len == 4 && memcmp(name, "neck", 4) == 0) return LIMB_NECK;
    return LIMB_UNKNOWN;
}

/* ─── Buffered netconn reader ─── */

typedef struct {
    struct netconn *conn;
    struct netbuf  *buf;
    uint8_t        *data;
    uint16_t        len;
    uint16_t        pos;
} NetReader_t;

static void reader_init(NetReader_t *r, struct netconn *conn) {
    r->conn = conn;
    r->buf  = NULL;
    r->data = NULL;
    r->len  = 0;
    r->pos  = 0;
}

static void reader_cleanup(NetReader_t *r) {
    if (r->buf) { netbuf_delete(r->buf); r->buf = NULL; }
}

/* Read exactly 'count' bytes. Returns false on disconnect/timeout. */
static bool reader_read(NetReader_t *r, uint8_t *dst, uint16_t count) {
    extern IWDG_HandleTypeDef hiwdg1; /* Refresh watchdog while waiting */
    
    uint16_t filled = 0;
    while (filled < count) {
        if (r->pos >= r->len) {
            if (r->buf) { netbuf_delete(r->buf); r->buf = NULL; }
            err_t err = netconn_recv(r->conn, &r->buf);
            if (err == ERR_TIMEOUT) {
                HAL_IWDG_Refresh(&hiwdg1);
                continue;
            }
            if (err != ERR_OK || r->buf == NULL) return false;
            netbuf_data(r->buf, (void **)&r->data, &r->len);
            r->pos = 0;
        }
        
        HAL_IWDG_Refresh(&hiwdg1); /* Do not let the board reset during idle */
        
        uint16_t avail = r->len - r->pos;
        uint16_t need  = count - filled;
        uint16_t chunk = (avail < need) ? avail : need;
        memcpy(dst + filled, r->data + r->pos, chunk);
        r->pos  += chunk;
        filled  += chunk;
    }
    return true;
}

static bool reader_read_byte(NetReader_t *r, uint8_t *out) {
    return reader_read(r, out, 1);
}

/* ─── Protocol encoding/decoding ─── */

/* XOR checksum: CMD ^ LEN_LO ^ LEN_HI ^ payload[0..N] */
static uint8_t compute_checksum(uint8_t cmd, const uint8_t *payload, uint16_t len) {
    uint8_t cs = cmd ^ (uint8_t)(len & 0xFF) ^ (uint8_t)(len >> 8);
    for (uint16_t i = 0; i < len; i++) cs ^= payload[i];
    return cs;
}

/* Extract [len(1)][string...] from buffer, resolve to limb index.
 * Returns bytes consumed, or 0 on parse error. */
static uint16_t parse_limb_name(const uint8_t *buf, uint16_t buf_len,
                                uint8_t *limb_index) {
    if (buf_len < 1) return 0;
    uint8_t slen = buf[0];
    if (1 + slen > buf_len) return 0;
    *limb_index = limb_name_to_index((const char *)&buf[1], slen);
    return 1 + slen;
}

static uint32_t read_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static float read_float(const uint8_t *p) {
    float val; memcpy(&val, p, 4); return val;
}

static void write_float(uint8_t *p, float val) {
    memcpy(p, &val, 4);
}

/* ─── Response framing ─── */

/* Build framed response and send. Static buffer avoids 2KB stack alloc. */
static err_t send_response(struct netconn *conn, uint8_t cmd,
                           const uint8_t *payload, uint16_t payload_len) {
    static uint8_t frame[4 + PROTO_MAX_PAYLOAD + 2];
    if (payload_len > PROTO_MAX_PAYLOAD) return ERR_MEM;

    uint16_t frame_len = 4 + payload_len + 2;
    frame[0] = PROTO_START_BYTE;
    frame[1] = cmd;
    frame[2] = (uint8_t)(payload_len & 0xFF);
    frame[3] = (uint8_t)(payload_len >> 8);
    if (payload_len > 0) memcpy(&frame[4], payload, payload_len);
    frame[4 + payload_len]     = compute_checksum(cmd, payload, payload_len);
    frame[4 + payload_len + 1] = PROTO_END_BYTE;

    return netconn_write(conn, frame, frame_len, NETCONN_COPY);
}

/* Empty ACK — echoes the command ID with zero-length payload */
static err_t send_ack(struct netconn *conn, uint8_t cmd) {
    return send_response(conn, cmd, NULL, 0);
}

/* ─── Command handlers ─── */

/* CMD 0x01/0x02/0x03: Set single motor position/velocity/effort */
static void handle_set_single(uint8_t cmd, const uint8_t *payload, uint16_t len,
                              struct netconn *conn) {
    uint8_t limb;
    uint16_t off = parse_limb_name(payload, len, &limb);
    if (off == 0 || off + 8 > len || limb == LIMB_UNKNOWN) {
        send_ack(conn, cmd); return;
    }

    uint32_t motor_id = read_u32(&payload[off]);
    float value = read_float(&payload[off + 4]);

    switch (cmd) {
        case CMD_SET_POSITION: motor_set_position(limb, motor_id, (double)value); break;
        case CMD_SET_VELOCITY: motor_set_velocity(limb, motor_id, (double)value); break;
        case CMD_SET_EFFORT:   motor_set_effort(limb, motor_id, (double)value);   break;
        default: break;
    }
    send_ack(conn, cmd);
}

/* CMD 0x04: Get single motor state */
static void handle_get_single(const uint8_t *payload, uint16_t len,
                               struct netconn *conn) {
    uint8_t limb;
    uint16_t off = parse_limb_name(payload, len, &limb);
    if (off == 0 || off + 4 > len) { send_ack(conn, CMD_GET_STATE); return; }

    uint32_t motor_id = read_u32(&payload[off]);
    double pos = 0, vel = 0, eff = 0, temp = 0;
    
    if (limb != LIMB_UNKNOWN) {
        motor_get_state(limb, motor_id, &pos, &vel, &eff, &temp);
    }

    uint8_t resp[12];
    write_float(&resp[0], (float)pos);
    write_float(&resp[4], (float)vel);
    write_float(&resp[8], (float)eff);
    send_response(conn, CMD_GET_STATE, resp, 12);
}

/* CMD 0x14/0x15/0x16: Set limb positions/velocities/efforts */
static void handle_set_limb(uint8_t cmd, const uint8_t *payload, uint16_t len,
                            struct netconn *conn) {
    uint8_t limb;
    uint16_t off = parse_limb_name(payload, len, &limb);
    if (off == 0 || off + 1 > len || limb == LIMB_UNKNOWN) {
        send_ack(conn, cmd); return;
    }

    uint8_t num_joints = payload[off++];
    if (off + num_joints * 8 > len) { send_ack(conn, cmd); return; }

    for (uint8_t i = 0; i < num_joints; i++) {
        uint32_t motor_id = read_u32(&payload[off]);
        float value = read_float(&payload[off + 4]);
        off += 8;
        switch (cmd) {
            case CMD_SET_LIMB_POSITIONS:  motor_set_position(limb, motor_id, (double)value); break;
            case CMD_SET_LIMB_VELOCITIES: motor_set_velocity(limb, motor_id, (double)value); break;
            case CMD_SET_LIMB_EFFORTS:    motor_set_effort(limb, motor_id, (double)value);   break;
            default: break;
        }
    }
    send_ack(conn, cmd);
}

/* CMD 0x17: Get limb motor states */
static void handle_get_limb(const uint8_t *payload, uint16_t len,
                            struct netconn *conn) {
    uint8_t limb;
    uint16_t off = parse_limb_name(payload, len, &limb);
    if (off == 0 || off + 1 > len) { send_ack(conn, CMD_GET_LIMB_STATES); return; }

    uint8_t num_joints = payload[off++];
    if (off + num_joints * 4 > len) { send_ack(conn, CMD_GET_LIMB_STATES); return; }

    uint8_t resp[MAX_MOTORS_PER_LIMB * 12];
    uint16_t resp_len = 0;

    for (uint8_t i = 0; i < num_joints && resp_len + 12 <= sizeof(resp); i++) {
        uint32_t motor_id = read_u32(&payload[off]);
        off += 4;
        
        double pos = 0, vel = 0, eff = 0, temp = 0;
        if (limb != LIMB_UNKNOWN) {
            motor_get_state(limb, motor_id, &pos, &vel, &eff, &temp);
        }
        
        write_float(&resp[resp_len],      (float)pos);
        write_float(&resp[resp_len + 4],  (float)vel);
        write_float(&resp[resp_len + 8],  (float)eff);
        resp_len += 12;
    }
    send_response(conn, CMD_GET_LIMB_STATES, resp, resp_len);
}

/* CMD 0x20: Emergency stop all motors */
static void handle_emergency_stop(struct netconn *conn) {
    motor_stop_all();
    send_ack(conn, CMD_EMERGENCY_STOP);
}

/* CMD 0x21: Emergency stop specific limb motors */
static void handle_emergency_stop_limb(const uint8_t *payload, uint16_t len,
                                       struct netconn *conn) {
    uint8_t limb;
    uint16_t off = parse_limb_name(payload, len, &limb);
    if (off == 0 || off + 1 > len || limb == LIMB_UNKNOWN) {
        send_ack(conn, CMD_EMERGENCY_STOP_LIMB); return;
    }

    uint8_t num = payload[off++];
    for (uint8_t i = 0; i < num && off + 4 <= len; i++) {
        motor_stop(limb, read_u32(&payload[off]));
        off += 4;
    }
    send_ack(conn, CMD_EMERGENCY_STOP_LIMB);
}

/* CMD 0x22: Clear all motor errors */
static void handle_reset_errors(struct netconn *conn) {
    motor_clear_errors_all();
    send_ack(conn, CMD_RESET_ERRORS);
}

/* CMD 0x23: Enable/disable all motors */
static void handle_enable_motors(const uint8_t *payload, uint16_t len,
                                 struct netconn *conn) {
    if (len < 1) { send_ack(conn, CMD_ENABLE_MOTORS); return; }
    motor_enable_all(payload[0] != 0);
    send_ack(conn, CMD_ENABLE_MOTORS);
}

/* CMD 0x24: Enable/disable specific limb motors */
static void handle_enable_limb_motors(const uint8_t *payload, uint16_t len,
                                      struct netconn *conn) {
    uint8_t limb;
    uint16_t off = parse_limb_name(payload, len, &limb);
    if (off == 0 || off + 2 > len || limb == LIMB_UNKNOWN) {
        send_ack(conn, CMD_ENABLE_LIMB_MOTORS); return;
    }

    uint8_t enable = payload[off++];
    uint8_t num = payload[off++];
    for (uint8_t i = 0; i < num && off + 4 <= len; i++) {
        motor_enable(limb, read_u32(&payload[off]), enable != 0);
        off += 4;
    }
    send_ack(conn, CMD_ENABLE_LIMB_MOTORS);
}

/* CMD 0x30: Set current motor positions as encoder zero */
static void handle_set_encoder_zero(const uint8_t *payload, uint16_t len,
                                     struct netconn *conn) {
    uint8_t limb;
    uint16_t off = parse_limb_name(payload, len, &limb);
    if (off == 0 || off + 1 > len || limb == LIMB_UNKNOWN) {
        uint8_t resp = 0;
        send_response(conn, CMD_SET_ENCODER_ZERO, &resp, 1); return;
    }

    uint8_t num = payload[off++];
    if (off + num * 4 > len) {
        uint8_t resp = 0;
        send_response(conn, CMD_SET_ENCODER_ZERO, &resp, 1); return;
    }

    uint32_t motor_ids[MAX_MOTORS_PER_LIMB];
    for (uint8_t i = 0; i < num && i < MAX_MOTORS_PER_LIMB; i++) {
        motor_ids[i] = read_u32(&payload[off]);
        off += 4;
    }

    bool ok = motor_set_encoder_zero(limb, motor_ids, num);
    uint8_t resp = ok ? 1 : 0;
    send_response(conn, CMD_SET_ENCODER_ZERO, &resp, 1);
}

/* ─── Gripper command handlers ─── */

/* CMD 0x40: Ping the gripper motor */
static void handle_gripper_ping(struct netconn *conn) {
    STS_Status st = gripper_ping();
    uint8_t resp = (st == STS_OK) ? 1 : 0;
    send_response(conn, CMD_GRIPPER_PING, &resp, 1);
}

/* CMD 0x41: Open the gripper */
static void handle_gripper_open(const uint8_t *payload, uint16_t len,
                                struct netconn *conn) {
    uint16_t speed = 0;
    if (len >= 2) speed = (uint16_t)(payload[0] | (payload[1] << 8));
    STS_Status st = gripper_open(speed);
    uint8_t resp = (st == STS_OK) ? 1 : 0;
    send_response(conn, CMD_GRIPPER_OPEN, &resp, 1);
}

/* CMD 0x42: Close the gripper */
static void handle_gripper_close(const uint8_t *payload, uint16_t len,
                                 struct netconn *conn) {
    uint16_t speed = 0;
    if (len >= 2) speed = (uint16_t)(payload[0] | (payload[1] << 8));
    STS_Status st = gripper_close(speed);
    uint8_t resp = (st == STS_OK) ? 1 : 0;
    send_response(conn, CMD_GRIPPER_CLOSE, &resp, 1);
}

/* CMD 0x43: Move gripper to a specific position */
static void handle_gripper_move_to(const uint8_t *payload, uint16_t len,
                                   struct netconn *conn) {
    if (len < 2) { send_ack(conn, CMD_GRIPPER_MOVE_TO); return; }
    uint16_t position = (uint16_t)(payload[0] | (payload[1] << 8));
    uint16_t speed = 0;
    if (len >= 4) speed = (uint16_t)(payload[2] | (payload[3] << 8));
    STS_Status st = gripper_move_to(position, speed);
    uint8_t resp = (st == STS_OK) ? 1 : 0;
    send_response(conn, CMD_GRIPPER_MOVE_TO, &resp, 1);
}

/* CMD 0x44: Read gripper status (position, speed, load, voltage, temp) */
static void handle_gripper_get_status(struct netconn *conn) {
    GripperStatus gs;
    STS_Status st = gripper_read_status(&gs);
    if (st != STS_OK) {
        uint8_t resp = 0;
        send_response(conn, CMD_GRIPPER_GET_STATUS, &resp, 1);
        return;
    }
    /* Pack: [ok(1)][pos_lo][pos_hi][speed_lo][speed_hi]
     *       [load_lo][load_hi][voltage(1)][temp(1)] = 9 bytes */
    uint8_t resp[9];
    resp[0] = 1;
    resp[1] = (uint8_t)(gs.position & 0xFF);
    resp[2] = (uint8_t)(gs.position >> 8);
    resp[3] = (uint8_t)((uint16_t)gs.speed & 0xFF);
    resp[4] = (uint8_t)((uint16_t)gs.speed >> 8);
    resp[5] = (uint8_t)(gs.load & 0xFF);
    resp[6] = (uint8_t)(gs.load >> 8);
    resp[7] = (uint8_t)(gs.voltage * 10.0f);  /* back to 0.1V units */
    resp[8] = gs.temp_c;
    send_response(conn, CMD_GRIPPER_GET_STATUS, resp, 9);
}

/* CMD 0x31: Read acceleration parameters for a limb.
 * Payload: [limb_name][num_joints(1)][motor_id(4)]...
 * Response: [motor_id(4) + accel(4)] per joint */
static void handle_read_accel(const uint8_t *payload, uint16_t len,
                              struct netconn *conn) {
    uint8_t limb;
    uint16_t off = parse_limb_name(payload, len, &limb);
    if (off == 0 || off + 1 > len || limb == LIMB_UNKNOWN) {
        send_ack(conn, CMD_READ_ACCEL); return;
    }

    uint8_t num_joints = payload[off++];
    if (off + num_joints * 4 > len || num_joints > 6) {
        send_ack(conn, CMD_READ_ACCEL); return;
    }

    FDCAN_HandleTypeDef *hfdcan = NULL;
    bool (*get_msg)(FDCAN_RxMessage_t *) = NULL;
    switch (limb) {
        case LIMB_LEFT_ARM:  hfdcan = &hfdcan2; get_msg = FDCAN2_Driver_GetMessage; break;
        case LIMB_RIGHT_ARM: hfdcan = &hfdcan1; get_msg = FDCAN_Driver_GetMessage;  break;
        case LIMB_NECK:      hfdcan = &hfdcan3; get_msg = FDCAN3_Driver_GetMessage; break;
        default:             hfdcan = &hfdcan1; get_msg = FDCAN_Driver_GetMessage;  break;
    }

    uint8_t resp[6 * 8];
    uint16_t roff = 0;

    for (uint8_t i = 0; i < num_joints; i++) {
        uint32_t motor_id = read_u32(&payload[off]);
        off += 4;

        /* Drain stale CAN messages */
        FDCAN_RxMessage_t discard;
        while (get_msg(&discard)) {}

        MYACTUATOR_READ_ACCEL(hfdcan, motor_id, POS_PLAN_ACCEL);

        uint32_t accel_val = 0;
        uint32_t start = HAL_GetTick();
        while ((HAL_GetTick() - start) < 50) {
            FDCAN_RxMessage_t msg;
            if (get_msg(&msg)) {
                if (msg.motor_id == motor_id && msg.command == 0x42) {
                    memcpy(&accel_val, &msg.data[4], 4);
                    break;
                }
            }
        }

        resp[roff++] = motor_id & 0xFF;
        resp[roff++] = (motor_id >> 8) & 0xFF;
        resp[roff++] = (motor_id >> 16) & 0xFF;
        resp[roff++] = (motor_id >> 24) & 0xFF;
        resp[roff++] = accel_val & 0xFF;
        resp[roff++] = (accel_val >> 8) & 0xFF;
        resp[roff++] = (accel_val >> 16) & 0xFF;
        resp[roff++] = (accel_val >> 24) & 0xFF;
    }

    send_response(conn, CMD_READ_ACCEL, resp, roff);
}

/* CMD 0x32: Write acceleration to ROM+RAM for a limb.
 * Payload: [limb_name][num_joints(1)][motor_id(4) + accel(4)]... */
static void handle_write_accel(const uint8_t *payload, uint16_t len,
                               struct netconn *conn) {
    uint8_t limb;
    uint16_t off = parse_limb_name(payload, len, &limb);
    if (off == 0 || off + 1 > len || limb == LIMB_UNKNOWN) {
        send_ack(conn, CMD_WRITE_ACCEL); return;
    }

    uint8_t num_joints = payload[off++];
    if (off + num_joints * 8 > len || num_joints > 6) {
        send_ack(conn, CMD_WRITE_ACCEL); return;
    }

    FDCAN_HandleTypeDef *hfdcan = NULL;
    switch (limb) {
        case LIMB_LEFT_ARM:  hfdcan = &hfdcan2; break;
        case LIMB_RIGHT_ARM: hfdcan = &hfdcan1; break;
        case LIMB_NECK:      hfdcan = &hfdcan3; break;
        default:             hfdcan = &hfdcan1; break;
    }

    for (uint8_t i = 0; i < num_joints; i++) {
        uint32_t motor_id = read_u32(&payload[off]);
        uint32_t accel_val = read_u32(&payload[off + 4]);
        off += 8;
        MYACTUATOR_WRITE_ACCEL_TO_ROM_RAM(hfdcan, motor_id, POS_PLAN_ACCEL, accel_val);
        osDelay(10);
    }

    send_ack(conn, CMD_WRITE_ACCEL);
}

/* ─── Command dispatch ─── */

static void dispatch_command(uint8_t cmd, const uint8_t *payload,
                             uint16_t len, struct netconn *conn) {
    switch (cmd) {
        /* Single motor commands (0x01–0x04) */
        case CMD_SET_POSITION:
        case CMD_SET_VELOCITY:
        case CMD_SET_EFFORT:
            handle_set_single(cmd, payload, len, conn);  break;
        case CMD_GET_STATE:
            handle_get_single(payload, len, conn);       break;

        /* Limb batch commands (0x14–0x17) */
        case CMD_SET_LIMB_POSITIONS:
        case CMD_SET_LIMB_VELOCITIES:
        case CMD_SET_LIMB_EFFORTS:
            handle_set_limb(cmd, payload, len, conn);    break;
        case CMD_GET_LIMB_STATES:
            handle_get_limb(payload, len, conn);         break;

        /* Safety & control (0x20–0x24) */
        case CMD_EMERGENCY_STOP:
            handle_emergency_stop(conn);                 break;
        case CMD_EMERGENCY_STOP_LIMB:
            handle_emergency_stop_limb(payload, len, conn); break;
        case CMD_RESET_ERRORS:
            handle_reset_errors(conn);                   break;
        case CMD_ENABLE_MOTORS:
            handle_enable_motors(payload, len, conn);    break;
        case CMD_ENABLE_LIMB_MOTORS:
            handle_enable_limb_motors(payload, len, conn); break;
        case CMD_SET_ENCODER_ZERO:
            handle_set_encoder_zero(payload, len, conn);   break;
        case CMD_READ_ACCEL:
            handle_read_accel(payload, len, conn);         break;
        case CMD_WRITE_ACCEL:
            handle_write_accel(payload, len, conn);        break;

        /* Gripper commands (0x40–0x44) */
        case CMD_GRIPPER_PING:
            handle_gripper_ping(conn);                   break;
        case CMD_GRIPPER_OPEN:
            handle_gripper_open(payload, len, conn);     break;
        case CMD_GRIPPER_CLOSE:
            handle_gripper_close(payload, len, conn);    break;
        case CMD_GRIPPER_MOVE_TO:
            handle_gripper_move_to(payload, len, conn);  break;
        case CMD_GRIPPER_GET_STATUS:
            handle_gripper_get_status(conn);             break;

        /* Keepalive */
        case CMD_PING:
            send_ack(conn, CMD_PING);                    break;

        default:
            send_ack(conn, cmd);                         break;
    }
}

/* ─── Client session handler ─── */

static void tcp_handle_client(struct netconn *conn) {
    /* Large static buffer to avoid stack overflow */
    static uint8_t payload_buf[PROTO_MAX_PAYLOAD];
    NetReader_t reader;
    reader_init(&reader, conn);

    netconn_set_recvtimeout(conn, TCP_RECV_TIMEOUT_MS);

    /* Disable Nagle for low-latency command responses */
    struct tcp_pcb *pcb = conn->pcb.tcp;
    if (pcb) tcp_nagle_disable(pcb);

    while (1) {
        uint8_t byte;

        /* Sync: scan for start byte */
        if (!reader_read_byte(&reader, &byte)) break;
        if (byte != PROTO_START_BYTE) continue;

        /* Header: [CMD(1)] [LEN(2)] */
        uint8_t header[3];
        if (!reader_read(&reader, header, 3)) break;
        uint8_t cmd = header[0];
        uint16_t payload_len = (uint16_t)header[1] | ((uint16_t)header[2] << 8);
        if (payload_len > PROTO_MAX_PAYLOAD) continue;

        /* Payload */
        if (payload_len > 0)
            if (!reader_read(&reader, payload_buf, payload_len)) break;

        /* Checksum verify */
        uint8_t received_cs;
        if (!reader_read_byte(&reader, &received_cs)) break;
        if (received_cs != compute_checksum(cmd, payload_buf, payload_len)) continue;

        /* End byte */
        uint8_t end;
        if (!reader_read_byte(&reader, &end)) break;
        if (end != PROTO_END_BYTE) continue;

        /* Valid frame — dispatch and toggle activity LED */
        dispatch_command(cmd, payload_buf, payload_len, conn);
        HAL_GPIO_TogglePin(LED_YELLOW_Port, LED_YELLOW_Pin);
    }

    reader_cleanup(&reader);
}

/* ─── Server task ─── */

static void tcp_server_task(void *pvParameters) {
    (void)pvParameters;
    struct netconn *listener, *client;
    err_t err;

    /* Allow LwIP stack to fully initialize */
    vTaskDelay(pdMS_TO_TICKS(2000));

    motor_control_init();

    listener = netconn_new(NETCONN_TCP);
    if (listener == NULL) { vTaskDelete(NULL); return; }

    err = netconn_bind(listener, IP_ADDR_ANY, TCP_SERVER_PORT);
    if (err != ERR_OK) {
        HAL_GPIO_WritePin(LED_RED_Port, LED_RED_Pin, GPIO_PIN_SET);
        netconn_delete(listener);
        vTaskDelete(NULL);
        return;
    }

    netconn_listen_with_backlog(listener, 1);
    HAL_GPIO_WritePin(LED_GREEN_Port, LED_GREEN_Pin, GPIO_PIN_SET);

    while (1) {
        err = netconn_accept(listener, &client);
        if (err == ERR_OK) {
            HAL_GPIO_WritePin(LED_YELLOW_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_RED_Port, LED_RED_Pin, GPIO_PIN_RESET);

            tcp_handle_client(client);
            netconn_close(client);
            netconn_delete(client);

            HAL_GPIO_WritePin(LED_YELLOW_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_RED_Port, LED_RED_Pin, GPIO_PIN_SET);
        }
    }
}

/* ─── Public API ─── */

void tcp_server_init(void) {
    xTaskCreate(tcp_server_task, "TCPServer",
                TCP_TASK_STACK, NULL, TCP_TASK_PRIORITY, NULL);
}
