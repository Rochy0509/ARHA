/*
 * gripper_control.h — ARHA Gripper Control
 *
 * Wraps the STS3215 driver with TCP-callable API and
 * provides the STS_UART HAL callbacks for USART6.
 *
 * Author: Kenneth Martinez
 */

#ifndef GRIPPER_CONTROL_H_
#define GRIPPER_CONTROL_H_

#include <stdint.h>
#include "motor/STS3215.h"

/* ─── Public API ─── */

/**
 * @brief Initialise the gripper subsystem.
 *
 * Enables the USART6 receiver and pings the STS3215
 * motor.  Call once from main before starting the TCP server.
 *
 * @return STS_OK if the motor responds.
 */
STS_Status gripper_init(void);

/**
 * @brief Command the gripper to close.
 * @param speed  Closing speed (steps/s), 0 = default.
 */
STS_Status gripper_close(uint16_t speed);

/**
 * @brief Command the gripper to open fully.
 * @param speed  Opening speed (steps/s), 0 = default.
 */
STS_Status gripper_open(uint16_t speed);

/**
 * @brief Move the gripper to an arbitrary encoder position.
 * @param position  Target position (0–4095).
 * @param speed     Speed (steps/s), 0 = default.
 */
STS_Status gripper_move_to(uint16_t position, uint16_t speed);

/**
 * @brief Read all gripper feedback registers.
 */
STS_Status gripper_read_status(GripperStatus *out);

/**
 * @brief Read only the current encoder position.
 */
STS_Status gripper_read_position(uint16_t *out);

/**
 * @brief Set the torque limit (0–100%).
 */
STS_Status gripper_set_torque(uint8_t torque_pct);

/**
 * @brief Ping the gripper motor.
 */
STS_Status gripper_ping(void);

#endif /* GRIPPER_CONTROL_H_ */
