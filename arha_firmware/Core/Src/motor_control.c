/*
 * motor_control.c
 *
 * Motor abstraction layer for ARHA Robot.
 * Handles:
 * - TCP Radians <-> MyActuator Degrees/LSB conversion
 * - Limb -> FDCAN Channel mapping
 * - CAN Read/Write via myactuator.c and fdcan.c
 *
 * Default config: Right Arm (ID 1) uses FDCAN1.
 */

#include "motor_control.h"
#include "myactuator.h"
#include "fdcan.h"
#include "main.h" /* For HAL_Delay */
#include "cmsis_os.h" /* For osDelay */
#include <math.h>
#include <string.h>

/* Number of joints per limb */
static const uint8_t limb_joint_counts[NUM_LIMBS] = {6, 6, 2};

/* Radians to Degrees conversion factor
 * 1 rad = 180 / PI degrees
 */
#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)

/* MyActuator uses 0.01 degree units for position */
#define DEG_TO_MYA_LSB 100.0
#define MYA_LSB_TO_DEG 0.01

/* Torque constants (Nm/A) per motor ID — both arms share the same layout */
static const double arm_torque_constants[7] = {
    0.0,    /* unused (IDs are 1-indexed) */
    7.46,   /* ID 1: X10S2V3 - Shoulder U */
    7.50,   /* ID 2: X8-60   - Shoulder D */
    1.92,   /* ID 3: X8-20   - Elbow U    */
    7.50,   /* ID 4: X8-60   - Elbow D    */
    1.92,   /* ID 5: X8-20   - Wrist U    */
    1.25,   /* ID 6: X6-8    - Wrist D    */
};

/* Neck motors are both X6-8 (Kt = 1.25 Nm/A) */
static const double neck_torque_constant = 1.25;

static double get_torque_constant(uint8_t limb, uint32_t motor_id) {
    if (limb == LIMB_NECK) return neck_torque_constant;
    if (motor_id >= 1 && motor_id <= 6) return arm_torque_constants[motor_id];
    return 1.0; /* fallback */
}

/* Timeout for waiting for CAN response (ms) */
#define CAN_RESPONSE_TIMEOUT_MS 50

/* Flush stale CAN responses before polling for a specific response */
static void drain_can_queue(void) {
    FDCAN_RxMessage_t discard;
    while (FDCAN_Driver_GetMessage(&discard)) { /* drain */ }
}

/* Helper: Convert MyActuator Position LSB to Radians */
static double mya_pos_to_rad(int32_t lsb) {
    double deg = (double)lsb * MYA_LSB_TO_DEG;
    return deg * DEG_TO_RAD;
}

/* Convert Rad/s to MyActuator Velocity (dps) */
static int32_t rads_to_mya_vel(double rad_s) {
    double dps = rad_s * RAD_TO_DEG;
    return (int32_t)dps; 
}

/* Convert Effort (Amps) to MyActuator Current LSB */
static int16_t effort_to_mya_current(double effort) {
    return (int16_t)(effort * 100.0); /* Assume 0.01A per LSB */
}

/*
 * Initialization
 */
void motor_control_init(void) {
    /* Initialize FDCAN driver handled in main.c */
}

/*
 * Motor Enable/Disable
 */
void motor_enable(uint8_t limb, uint32_t motor_id, bool enable) {
    if (enable) {
        /* Motors usually auto-enable on first command */
        MYACTUATOR_MOTOR_STOP(motor_id); 
        HAL_Delay(1);
    } else {
        MYACTUATOR_MOTOR_SHUTDOWN(motor_id);
    }
}

void motor_enable_all(bool enable) {
    for (uint8_t l = 0; l < NUM_LIMBS; l++) {
        for (uint32_t m = 1; m <= limb_joint_counts[l]; m++) {
            motor_enable(l, m, enable);
        }
    }
}

void motor_stop(uint8_t limb, uint32_t motor_id) {
    MYACTUATOR_MOTOR_STOP(motor_id);
}

void motor_stop_all(void) {
    for (uint8_t l = 0; l < NUM_LIMBS; l++) {
        for (uint32_t m = 1; m <= limb_joint_counts[l]; m++) {
            motor_stop(l, m);
        }
    }
}

void motor_clear_errors(uint8_t limb, uint32_t motor_id) {
    MYACTUATOR_RESET_MOTOR(motor_id);
}

void motor_clear_errors_all(void) {
    for (uint8_t l = 0; l < NUM_LIMBS; l++) {
        for (uint32_t m = 1; m <= limb_joint_counts[l]; m++) {
            motor_clear_errors(l, m);
        }
    }
}

/* Set position (rads) with default speed limit */
void motor_set_position(uint8_t limb, uint32_t motor_id, double position_rad) {
    float deg = (float)(position_rad * RAD_TO_DEG);
    MYACTUATOR_ABS_POS_CL_CONTROL(motor_id, (int16_t)DEFAULT_SPEED_LIMIT_DPS, deg);
}

void motor_set_velocity(uint8_t limb, uint32_t motor_id, double velocity_rad_s) {
    /* Convert Rad/s to Degrees/s (dps) */
    int32_t dps = rads_to_mya_vel(velocity_rad_s);
    MYACTUATOR_SPEED_CL_CONTROL(motor_id, dps);
}

void motor_set_effort(uint8_t limb, uint32_t motor_id, double effort) {
    /* Convert Effort to Current LSB */
    int16_t current = effort_to_mya_current(effort);
    MYACTUATOR_TORQUE_CL_CONTROL(motor_id, current);
}

/* Read motor state using 0x92 (position) + 0x9C (velocity, current, temp).
 * Returns true if at least position was read successfully. */
bool motor_get_state(uint8_t limb_index, uint32_t motor_id,
                     double *pos, double *vel, double *eff, double *temp) {

    FDCAN_RxMessage_t msg;
    uint32_t start_tick;
    bool got_pos = false;

    /* Default outputs */
    if (pos)  *pos  = 0.0;
    if (vel)  *vel  = 0.0;
    if (eff)  *eff  = 0.0;
    if (temp) *temp = 0.0;

    /* ── Step 1: Read multi-turn angle via 0x92 (0.01 deg/LSB, int32) ── */
    drain_can_queue();
    MYACTUATOR_READ_MULTI_ENC_ANGLE(motor_id);

    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < CAN_RESPONSE_TIMEOUT_MS) {
        if (FDCAN_Driver_GetMessage(&msg)) {
            if (msg.motor_id == motor_id && msg.command == 0x92) {
                int32_t angle_lsb = (int32_t)(msg.data[4] | (msg.data[5] << 8) |
                                              (msg.data[6] << 16) | (msg.data[7] << 24));
                if (pos) *pos = mya_pos_to_rad(angle_lsb);
                got_pos = true;
                break;
            }
        }
        osDelay(1);
    }

    /* ── Step 2: Read velocity, current, temp via 0x9C ── */
    drain_can_queue();
    MYACTUATOR_READ_MOTOR_STATUS_2(motor_id);

    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < CAN_RESPONSE_TIMEOUT_MS) {
        if (FDCAN_Driver_GetMessage(&msg)) {
            if (msg.motor_id == motor_id && msg.command == 0x9C) {
                /* DATA[1] = temperature (int8_t, 1°C/LSB) */
                if (temp) *temp = (double)(int8_t)msg.data[1];

                /* DATA[2-3] = torque current iq (int16_t, 0.01A/LSB) */
                int16_t iq = (int16_t)(msg.data[2] | (msg.data[3] << 8));
                double current_A = (double)iq * 0.01;
                if (eff) *eff = current_A * get_torque_constant(limb_index, motor_id);

                /* DATA[4-5] = speed (int16_t, 1dps/LSB) */
                int16_t speed_dps = (int16_t)(msg.data[4] | (msg.data[5] << 8));
                if (vel) *vel = (double)speed_dps * DEG_TO_RAD;

                break;
            }
        }
        osDelay(1);
    }

    return got_pos;
}

/* Zero encoder for all specified motors in a limb.
 * Sequence: set zero (0x64) → reset (0x76) → wait reboot → verify */
bool motor_set_encoder_zero(uint8_t limb, const uint32_t *motor_ids, uint8_t num_motors) {
    #define ZERO_TOLERANCE_DEG 0.1
    #define MOTOR_REBOOT_MS    5000

    /* Set current position as encoder zero */
    for (uint8_t i = 0; i < num_motors; i++) {
        drain_can_queue();
        MYACTUATOR_WRITE_CURRENT_MULTI_POS_ENC_TO_ROM_AS_MOTOR_ZERO(motor_ids[i]);
        osDelay(100);
    }

    /* Reset motors */
    for (uint8_t i = 0; i < num_motors; i++) {
        MYACTUATOR_RESET_MOTOR(motor_ids[i]);
        osDelay(10);
    }

    /* Wait for reboot */
    osDelay(MOTOR_REBOOT_MS);

    /* Verify positions near zero */
    bool all_ok = true;
    for (uint8_t i = 0; i < num_motors; i++) {
        double pos = 0;
        if (motor_get_state(limb, motor_ids[i], &pos, NULL, NULL, NULL)) {
            if (fabs(pos * RAD_TO_DEG) > ZERO_TOLERANCE_DEG)
                all_ok = false;
        } else {
            all_ok = false;
        }
    }
    return all_ok;
}
