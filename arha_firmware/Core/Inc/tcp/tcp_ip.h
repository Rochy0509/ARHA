/*
 * tcp_ip.h - TCP Server Protocol for ARHA Robot
 *
 * Binary packet protocol for PC <-> STM32 communication.
 * All multi-byte values are little-endian.
 * Doubles are IEEE 754, 8 bytes.
 *
 * Author: Kenneth Martinez
 */

#ifndef TCP_IP_H
#define TCP_IP_H

#include "lwip/api.h"
#include <stdint.h>
#include <stdbool.h>

// ── Protocol framing ──
#define PROTO_START_BYTE    0xAA
#define PROTO_END_BYTE      0x55
#define PROTO_MAX_PAYLOAD   2048

// ── Network config ──
#define TCP_SERVER_PORT     5000
#define TCP_TASK_STACK      1024
#define TCP_TASK_PRIORITY   24  // osPriorityNormal
#define TCP_RECV_TIMEOUT_MS 500

// ── Robot topology ──
#include "motor/motor_control.h"

// ── Robot topology ──
// defined in motor_control.h

// ── Command IDs (PC -> STM32) ──
typedef enum {
    CMD_SET_POSITION        = 0x01,
    CMD_SET_VELOCITY        = 0x02,
    CMD_SET_EFFORT          = 0x03,
    CMD_GET_STATE           = 0x04,

    CMD_SET_LIMB_POSITIONS  = 0x14,
    CMD_SET_LIMB_VELOCITIES = 0x15,
    CMD_SET_LIMB_EFFORTS    = 0x16,
    CMD_GET_LIMB_STATES     = 0x17,

    CMD_EMERGENCY_STOP      = 0x20,
    CMD_EMERGENCY_STOP_LIMB = 0x21,
    CMD_RESET_ERRORS        = 0x22,
    CMD_ENABLE_MOTORS       = 0x23,
    CMD_ENABLE_LIMB_MOTORS  = 0x24,
    CMD_SET_ENCODER_ZERO    = 0x30,
    CMD_READ_ACCEL          = 0x31,
    CMD_WRITE_ACCEL         = 0x32,

    /* Gripper commands (0x40–0x44) */
    CMD_GRIPPER_PING        = 0x40,
    CMD_GRIPPER_OPEN        = 0x41,
    CMD_GRIPPER_CLOSE       = 0x42,
    CMD_GRIPPER_MOVE_TO     = 0x43,
    CMD_GRIPPER_GET_STATUS  = 0x44,

    CMD_PING                = 0xFF,
} ProtocolCmd_t;

// Public API
void tcp_server_init(void);

#endif // TCP_IP_H
