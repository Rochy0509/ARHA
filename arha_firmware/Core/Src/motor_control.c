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

/* Torque constant approx 1 A per unit (placeholder) */
#define EFFORT_SCALE 1.0

/* Timeout for waiting for CAN response (ms) */
#define CAN_RESPONSE_TIMEOUT_MS 10

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

/* Read motor state using 0x92 (multi-turn angle). Returns true on success. */
bool motor_get_state(uint8_t limb_index, uint32_t motor_id,
                     double *pos, double *vel, double *eff, double *temp) {
    
    FDCAN_RxMessage_t msg;
    uint32_t start_tick = HAL_GetTick();

    /* Use 0x92 command as per protocol doc (0.01 deg units) */
    MYACTUATOR_READ_MULTI_ENC_ANGLE(motor_id);

    /* Poll for response with timeout */
    while ((HAL_GetTick() - start_tick) < CAN_RESPONSE_TIMEOUT_MS) {
        if (FDCAN_Driver_GetMessage(&msg)) {
            if (msg.motor_id == motor_id && msg.command == 0x92) {
                
                /* Parse 0x92: Bytes 4-7 is int32 angle (0.01 deg/LSB) */
                int32_t angle_int = (int32_t)(msg.data[4] | (msg.data[5] << 8) | 
                                            (msg.data[6] << 16) | (msg.data[7] << 24));
                
                if (pos) *pos = mya_pos_to_rad(angle_int);
                
                /* Velocity/Effort not available in 0x92 */
                if (vel) *vel = 0.0;
                if (eff) *eff = 0.0;
                if (temp) *temp = 0.0;

                return true;
            }
        }
        /* Yield to other tasks while waiting */
        osDelay(1);
    }

    return false; /* Timeout */
}
