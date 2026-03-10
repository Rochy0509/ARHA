#include "motor/motor_control.h"
#include "motor/myactuator.h"
#include "motor/fdcan.h"
#include "main.h" /* For HAL_Delay */
#include "cmsis_os.h" /* For osDelay */
#include <math.h>
#include <string.h>

extern IWDG_HandleTypeDef hiwdg1;

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)

/* MyActuator uses 0.01 degree units for position */
#define DEG_TO_MYA_LSB 100.0
#define MYA_LSB_TO_DEG 0.01

#define BLINK_DELAY_MS 100

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

const uint8_t limb_joint_counts[NUM_LIMBS] = {6, 6, 0};

// Neck motors are X6-8 (Kt = 1.25 Nm/A).
static const double neck_torque_constant = 1.25;

static double get_torque_constant(uint8_t limb, uint32_t motor_id) {
    if (limb == LIMB_NECK) return neck_torque_constant;
    if (motor_id >= 1 && motor_id <= 6) return arm_torque_constants[motor_id];
    return 1.0; /* fallback */
}

#define CAN_RESPONSE_TIMEOUT_MS 50

/* ─── Limb → FDCAN bus routing ─── */

static FDCAN_HandleTypeDef* get_fdcan_for_limb(uint8_t limb) {
    switch (limb) {
        case LIMB_LEFT_ARM:  return &hfdcan2;
        case LIMB_RIGHT_ARM: return &hfdcan1;
        case LIMB_NECK:      return &hfdcan3;
        default:             return &hfdcan1;
    }
}

typedef bool (*GetMessageFn)(FDCAN_RxMessage_t *msg);

static GetMessageFn get_rx_fn_for_limb(uint8_t limb) {
    switch (limb) {
        case LIMB_LEFT_ARM:  return FDCAN2_Driver_GetMessage;
        case LIMB_RIGHT_ARM: return FDCAN_Driver_GetMessage;
        case LIMB_NECK:      return FDCAN3_Driver_GetMessage;
        default:             return FDCAN_Driver_GetMessage;
    }
}

static void drain_can_queue_for_limb(uint8_t limb) {
    FDCAN_RxMessage_t discard;
    GetMessageFn get_msg = get_rx_fn_for_limb(limb);
    while (get_msg(&discard)) { /* drain */ }
}

static int32_t rads_to_mya_vel(double rad_s) {
    double dps = rad_s * RAD_TO_DEG;
    return (int32_t)dps; 
}

static int16_t effort_to_mya_current(double effort) {
    // Assumes 0.01A per LSB.
    return (int16_t)(effort * 100.0);
}

/*
 * Initialization
 */
void motor_control_init(void) {
    /* Initializes FDCAN driver handled in main.c */
}

void motor_enable(uint8_t limb, uint32_t motor_id, bool enable) {
    if (enable) {
    } else {
        MYACTUATOR_MOTOR_SHUTDOWN(get_fdcan_for_limb(limb), motor_id);
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
    MYACTUATOR_MOTOR_STOP(get_fdcan_for_limb(limb), motor_id);
}

void motor_stop_all(void) {
    for (uint8_t l = 0; l < NUM_LIMBS; l++) {
        for (uint32_t m = 1; m <= limb_joint_counts[l]; m++) {
            motor_stop(l, m);
        }
    }
}

void motor_clear_errors(uint8_t limb, uint32_t motor_id) {
    MYACTUATOR_RESET_MOTOR(get_fdcan_for_limb(limb), motor_id);
}

void motor_clear_errors_all(void) {
    for (uint8_t l = 0; l < NUM_LIMBS; l++) {
        for (uint32_t m = 1; m <= limb_joint_counts[l]; m++) {
            motor_clear_errors(l, m);
        }
    }
}

void motor_set_position(uint8_t limb, uint32_t motor_id, double position_rad) {
    FDCAN_HandleTypeDef *hfdcan = get_fdcan_for_limb(limb);
    float deg = (float)(position_rad * RAD_TO_DEG);

    // Fire-and-forget: sends position command, no response wait needed
    MYACTUATOR_ABS_POS_CL_CONTROL(hfdcan, motor_id, (int16_t)DEFAULT_SPEED_LIMIT_DPS, deg);
}

void motor_set_velocity(uint8_t limb, uint32_t motor_id, double velocity_rad_s) {
    FDCAN_HandleTypeDef *hfdcan = get_fdcan_for_limb(limb);
    int32_t dps = rads_to_mya_vel(velocity_rad_s);

    // Fire-and-forget
    MYACTUATOR_SPEED_CL_CONTROL(hfdcan, motor_id, dps);
}

void motor_set_effort(uint8_t limb, uint32_t motor_id, double effort) {
    FDCAN_HandleTypeDef *hfdcan = get_fdcan_for_limb(limb);
    int16_t current = effort_to_mya_current(effort);

    // Fire-and-forget
    MYACTUATOR_TORQUE_CL_CONTROL(hfdcan, motor_id, current);
}

// Requests 0x9C for vitals and 0x92 for the absolute multi-turn angle.
bool motor_get_state(uint8_t limb_index, uint32_t motor_id,
                     double *pos, double *vel, double *eff, double *temp) {
    FDCAN_HandleTypeDef *hfdcan = get_fdcan_for_limb(limb_index);
    GetMessageFn get_msg = get_rx_fn_for_limb(limb_index);
    FDCAN_RxMessage_t msg;
    uint32_t start_tick;
    bool got_9c = false;
    bool got_92 = false;

    drain_can_queue_for_limb(limb_index);

    // Requests both packets immediately so the motor can prepare the second while we read the first
    MYACTUATOR_READ_MOTOR_STATUS_2(hfdcan, motor_id);
    MYACTUATOR_READ_MULTI_ENC_ANGLE(hfdcan, motor_id);

    // Waits a combined maximum of 10ms for both responses
    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < 10) {
        if (get_msg(&msg)) {
            if (msg.motor_id == motor_id) {
                if (msg.command == 0x9C) {
                    if (temp) *temp = (double)(int8_t)msg.data[1];
                    if (eff) {
                        int16_t iq = (int16_t)(msg.data[2] | (msg.data[3] << 8));
                        *eff = (double)iq * 0.01 * get_torque_constant(limb_index, motor_id);
                    }
                    if (vel) {
                        int16_t speed_dps = (int16_t)(msg.data[4] | (msg.data[5] << 8));
                        *vel = (double)speed_dps * DEG_TO_RAD;
                    }
                    got_9c = true;
                }
                else if (msg.command == 0x92) {
                    if (pos) {
                        int32_t angle = 0;
                        memcpy(&angle, &msg.data[4], 4);
                        *pos = (double)angle * 0.01 * DEG_TO_RAD;
                    }
                    got_92 = true;
                }
            }
        }
        if (got_9c && got_92) {
            break;
        }
    }

    return (got_9c && got_92);
}

// Writes zero-offset to ROM, reboots motors, and validates
bool motor_set_encoder_zero(uint8_t limb, const uint32_t *motor_ids, uint8_t num_motors) {
    #define ZERO_TOLERANCE_DEG 0.05
    #define MOTOR_REBOOT_MS    5000
    #define MAX_ZERO_ATTEMPTS  5

    FDCAN_HandleTypeDef *hfdcan = get_fdcan_for_limb(limb);

    for (uint8_t attempt = 0; attempt < MAX_ZERO_ATTEMPTS; attempt++) {
        drain_can_queue_for_limb(limb);
        for (uint8_t i = 0; i < num_motors; i++) {
            MYACTUATOR_WRITE_CURRENT_MULTI_POS_ENC_TO_ROM_AS_MOTOR_ZERO(hfdcan, motor_ids[i]);
            osDelay(100);
        }

        for (uint8_t i = 0; i < num_motors; i++) {
            MYACTUATOR_RESET_MOTOR(hfdcan, motor_ids[i]);
            osDelay(100);
            HAL_IWDG_Refresh(&hiwdg1);
        }

        for (int w = 0; w < (MOTOR_REBOOT_MS / 100); w++) {
            osDelay(100);
            HAL_IWDG_Refresh(&hiwdg1);
        }

        bool all_ok = true;
        for (uint8_t i = 0; i < num_motors; i++) {
            double pos = 0;
            if (motor_get_state(limb, motor_ids[i], &pos, NULL, NULL, NULL)) {
                if (fabs(pos * RAD_TO_DEG) > ZERO_TOLERANCE_DEG) {
                    all_ok = false;
                }
            } else {
                all_ok = false;
            }
            HAL_IWDG_Refresh(&hiwdg1);
        }

        if (all_ok) return true;
    }

    return false;
}
