/*
 * motor_control.h
 *
 * Motor abstraction layer for ARHA Robot.
 * Handles unit conversion (Rad <-> Deg) and hardware abstraction.
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/* Robot Topology */
#define NUM_LIMBS 3
#define MAX_MOTORS_PER_LIMB 6

/* Limb Indices (match TCP protocol string parsing) */
#define LIMB_LEFT_ARM   0
#define LIMB_RIGHT_ARM  1
#define LIMB_NECK       2
#define LIMB_UNKNOWN    0xFF

/* Hardware Constants */
#define DEFAULT_SPEED_LIMIT_DPS 360

/* Public API */

/* Initialize motor configuration */
void motor_control_init(void);

/* Set single motor setpoints.
 * Units: position (radians), velocity (rad/s), effort (Amps - approx)
 */
void motor_set_position(uint8_t limb, uint32_t motor_id, double position_rad);
bool motor_set_position_and_wait(uint8_t limb, uint32_t motor_id, double position_rad, uint32_t timeout_ms, double tolerance_rad);
void motor_set_velocity(uint8_t limb, uint32_t motor_id, double velocity_rad_s);
void motor_set_effort(uint8_t limb, uint32_t motor_id, double effort);

/* Get single motor state.
 * Returns true if successful, false on timeout/error.
 * Units: position (radians), velocity (rad/s), effort (Amps), temp (C)
 */
bool motor_get_state(uint8_t limb, uint32_t motor_id,
                     double *pos, double *vel, double *eff, double *temp);

/* Read raw motor-status (0x9A) into an 8-byte buffer. Returns true on success. */
bool motor_get_status_raw(uint8_t limb, uint32_t motor_id, uint8_t *status_buf, uint8_t *status_len);

/* Enable/Disable/Stop */
void motor_stop(uint8_t limb, uint32_t motor_id);
void motor_stop_all(void);
void motor_enable(uint8_t limb, uint32_t motor_id, bool enable);
void motor_enable_all(bool enable);
void motor_clear_errors(uint8_t limb, uint32_t motor_id);
void motor_clear_errors_all(void);

void motor_rezero_position(uint8_t limb, uint32_t motor_id);

#endif /* MOTOR_CONTROL_H */
