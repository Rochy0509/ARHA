#ifndef MYACTUATOR_H_
#define MYACTUATOR_H_

#include "stm32f1xx_hal.h"

#define READ_PID                              0x30
#define WRITE_PID_TO_RAM                      0x31
#define WRITE_PID_TO_ROM                      0x32
#define READ_ACCEL                            0x42
#define WRITE_ACCEL_TO_ROM_RAM                0x43
#define READ_MULTI_ENC_POS_DATA               0x60
#define READ_MULTI_ORIG_POS                   0x61
#define READ_MULTI_ENC_ZERO_OFFSET            0x62
#define WRITE_ENC_MULTI_VALUE_TO_ROM_AS_MZERO 0x63
#define WRITE_CURRENT_MULTI_POS_ENC_ROM_MZERO 0x64
#define READ_SINGLE_ENC                       0x90
#define READ_MULTI_ENC_ANGLE                  0x92
#define READ_SINGLE_T_ANGLE                   0x94
#define READ_MOTOR_STATUS_1                   0x9A
#define READ_MOTOR_STATUS_2                   0x9C
#define READ_MOTOR_STATUS_3                   0x9D
#define MOTOR_SHUTDOWN                        0x80
#define MOTOR_STOP                            0x81
#define TORQUE_CL_CONTROL                     0xA1
#define SPEED_CL_CONTROL                      0xA2
#define ABSOLUTE_POS_CL_CONTROL               0xA4
#define SINGLE_POSITION_CONTROL               0xA6
#define INC_POS_CL_CONTROL                    0xA8
#define SYS_OP_MODE                           0x70
#define SYS_RESET                             0x76
#define READ_SYS_RUNTIME                      0xB1
#define SET_COMM_BAUD_RATE                    0xB4
#define FUNCTION_CONTROL                      0x20

/**
 * @brief Maps readable names to the index values for PID parameters.
 */
typedef enum{
	PID_CURRENT_KP = 0x01,
	PID_CURRENT_KI = 0x02,
	PID_SPEED_KP   = 0x04,
	PID_SPEED_KI   = 0x05,
	PID_POS_KP     = 0x07,
	PID_POS_KI     = 0x08,
	PID_POS_KD     = 0x09
}PID_PARAM_INDEX;

/**
 * @brief Maps readable names to the index values for acceleration parameters.
 */
typedef enum{
	POS_PLAN_ACCEL     = 0x00,
	POS_PLAN_DECEL     = 0x01,
	SPEED_PLAN_ACCEL   = 0x02,
	SPEED_PLAN_DECEL   = 0x03,
}ACCEL_INDEX;

/**
 * @brief Maps readable names to the index values for CAN bus baud rates.
 */
typedef enum{
	FIVE_HUNDRED_Kbps  = 0x00,
	ONE_Mbps           = 0x01,
}BAUD_RATE_INDEX;

/**
 * @brief Maps readable names to the index values for the Function Control command.
 */
typedef enum{
	CLEAR_MULTI_TURN        = 0x01,
	CANID_FILTER            = 0x02,
	ERROR_STATUS_TRANS      = 0x03,
	SAVE_MULTI_TURN         = 0x04,
	SET_CAN_ID              = 0x05,
	SET_MAX_POSITIVE_ANGLE  = 0x06,
	SET_MAX_NEGATIVE_ANGLE  = 0x07
}FUNCTION_CONTROL_INDEX;

/**
 * @brief Defines the motor's rotation direction for single-turn control.
 */
typedef enum {
    CLOCKWISE = 0x00,
    COUNTER_CLOCKWISE = 0x01
} SpinDirection;

// --- Global Variables ---
extern CAN_HandleTypeDef hcan;

/**
 * @brief Read the PID parameters of the current, speed, position as float data.
 * @param motor_id ID of the motor (1-5)
 * @param pid_index Index of the parameter to read, look at PID_INDEX_ENUM
 */

void MYACTUATOR_READ_PID(uint8_t motor_id,  PID_PARAM_INDEX pid_index);

/**
 * @brief Write the PID parameters of the current, speed, position as float data.
 * @param motor_id ID of the motor (1-5)
 * @param pid_index Index of the parameter to read, look at  PID_PARAM_INDEX
 * @param value the float value to be write to RAM.
 */

void MYACTUATOR_WRITE_PID_TO_RAM(uint8_t motor_id, PID_PARAM_INDEX pid_index, float value);

/**
 * @brief Write the PID parameters of the current, speed, position as float data.
 * @param motor_id ID of the motor (1-5)
 * @param pid_index Index of the parameter to read, look at  PID_PARAM_INDEX
 * @param value the float value to be write to ROM.
 */

void MYACTUATOR_WRITE_PID_TO_ROM(uint8_t motor_id, PID_PARAM_INDEX pid_index, float value);

/**
 * @brief Read acceleration/deceleration for position and speed planning.
 * @param motor_id ID of the motor (1-5)
 * @param pid_index Index of the acceleration/deceleration parameter to read, look at  ACCEL_INDEX
 */

void MYACTUATOR_READ_ACCEL(uint8_t motor_id, ACCEL_INDEX accel_index);

/**
 * @brief Write acceleration/deceleration for position and speed planning to RAM and ROM.
 * @param motor_id ID of the motor (1-5)
 * @param pid_index Index of the acceleration/deceleration parameter to read, look at  ACCEL_INDEX
 * @param value The value to write
 */

void MYACTUATOR_WRITE_ACCEL_TO_ROM_RAM(uint8_t motor_id, ACCEL_INDEX accel_index, int32_t value);

/**
 * @brief Sends a command to read the multi-turn encoder position data.
 * @param motor_id The ID of the target motor (1-5).
 */
void MYACTUATOR_READ_MULTI_ENC_POS(uint8_t motor_id);

/**
 * @brief Sends a command to read the multi-turn encoder's original (raw) position data.
 * @param motor_id The ID of the target motor (1-5).
 */
void MYACTUATOR_READ_MULTI_ENC_ORIGINAL_POS(uint8_t motor_id);

/**
 * @brief Sends a command to read the multi-turn encoder's currently stored zero offset value.
 * @param motor_id The ID of the target motor (1-32).
 */
void MYACTUATOR_READ_MULTI_ENC_ZERO_OFFSET(uint8_t motor_id);

/**
 * @brief Writes a specific multi-turn encoder value to ROM to serve as the motor's new zero position.
 * @param motor_id The ID of the target motor (1-5).
 * @param value The specific encoder value (as an integer) to set as the zero offset.
 * @note The motor must be restarted for the new zero point to take effect.
 */
void MYACTUATOR_WRITE_ENC_MULTI_TO_ROM_AS_MOTOR_ZERO(uint8_t motor_id, int32_t enc_offset);

/**
 * @brief Sets the motor's current multi-turn encoder position as the new zero offset, saving it to ROM.
 * @param motor_id The ID of the target motor (1-5).
 * @note The motor must be restarted for the new zero point to take effect.
 */
void MYACTUATOR_WRITE_CURRENT_MULTI_POS_ENC_TO_ROM_AS_MOTOR_ZERO(uint8_t motor_id);

/**
 * @brief Sends a command to read single-turn encoder data (position, raw position, and offset).
 * @param motor_id The ID of the target motor (1-5).
 */
void MYACTUATOR_READ_SINGLE_ENC(uint8_t motor_id);

/**
 * @brief Sends a command to read the multi-turn absolute angle of the motor output shaft.
 * @param motor_id The ID of the target motor (1-5).
 */
void MYACTUATOR_READ_MULTI_ENC_ANGLE(uint8_t motor_id);

/**
 * @brief Sends a command to read the single-turn angle of the motor output shaft (0-359.99 degrees).
 * @param motor_id The ID of the target motor (1-5).
 */
void MYACTUATOR_READ_SINGLE_ENC_ANGLE(uint8_t motor_id);

/**
 * @brief Reads motor status 1, which includes temperature, voltage, and error flags.
 * @param motor_id The ID of the target motor (1-5).
 */
void MYACTUATOR_READ_MOTOR_STATUS_1(uint8_t motor_id);

/**
 * @brief Reads motor status 2, which includes temperature, torque current, speed, and encoder position.
 * @param motor_id The ID of the target motor (1-5).
 */
void MYACTUATOR_READ_MOTOR_STATUS_2(uint8_t motor_id);

/**
 * @brief Reads motor status 3, which includes temperature and phase A/B/C current data.
 * @param motor_id The ID of the target motor (1-5).
 */
void MYACTUATOR_READ_MOTOR_STATUS_3(uint8_t motor_id);

/**
 * @brief Turns off the motor output and clears its running state.
 * @param motor_id The ID of the target motor (1-5).
 */
void MYACTUATOR_MOTOR_SHUTDOWN(uint8_t motor_id);

/**
 * @brief Stops the motor's motion but keeps it in a closed-loop mode.
 * @param motor_id The ID of the target motor (1-5).
 */
void MYACTUATOR_MOTOR_STOP(uint8_t motor_id);

/**
 * @brief Controls the motor using torque closed-loop control.
 * @param motor_id The ID of the target motor (1-5).
 * @param torque_value The target torque current. Unit: Amperes (A).
 */
void MYACTUATOR_TORQUE_CL_CONTROL(uint8_t motor_id, float torque_value);

/**
 * @brief Controls the motor using speed closed-loop control.
 * @param motor_id The ID of the target motor (1-5).
 * @param speed_value The target speed of the motor output shaft. Unit: degrees per second (dps).
 */
void MYACTUATOR_SPEED_CL_CONTROL(uint8_t motor_id, float speed_value);

/**
 * @brief Controls the motor's absolute multi-turn position.
 * @param motor_id The ID of the target motor (1-5).
 * @param speed_limit The maximum speed for the move. Unit: dps.
 * @param pos The target absolute angle. Unit: degrees.
 */
void MYACTUATOR_ABS_POS_CL_CONTROL(uint8_t motor_id, float speed_limit, float pos);

/**
 * @brief Controls the motor's single-turn position (0-359.99 degrees).
 * @param motor_id The ID of the target motor (1-5).
 * @param rotation The desired direction of rotation.
 * @param speed_limit The maximum speed for the move. Unit: dps.
 * @param position The target angle within a single turn. Unit: degrees (0-359.99).
 */
void MYACTUATOR_SINGLE_POS_CONTROL(uint8_t motor_id, SpinDirection direction, uint16_t speed_limit, float position);

/**
 * @brief Controls the motor's position relative to its current position (incremental move).
 * @param motor_id The ID of the target motor (1-5).
 * @param speed_limit The maximum speed for the move. Unit: dps.
 * @param pos The incremental angle to move. Unit: degrees.
 */
void MYACTUATOR_INC_POS_CL_CONTROL(uint8_t motor_id, float speed_limit, float pos);

/**
 * @brief Sends a command to get the system's current operating mode (current, speed, or position loop).
 * @param motor_id The ID of the target motor (1-5).
 */
void MYACTUATOR_READ_SYS_OP_MODE(uint8_t motor_id);

/**
 * @brief Sends a command to read the system's total runtime since the last reset.
 * @param motor_id The ID of the target motor (1-5).
 */
void MYACTUATOR_READ_SYS_RUNTIME(uint8_t motor_id);

/**
 * @brief Sets the communication baud rate for the CAN bus.
 * @param motor_id The ID of the target motor (1-5).
 * @param baud The desired baud rate from the BAUD_RATE_INDEX enum.
 * @note This value is saved to ROM and takes effect on the next power cycle.
 */
void MYACTUATOR_SET_BAUD_RATE(uint8_t motor_id, BAUD_RATE_INDEX baud);

/**
 * @brief Executes a special system function based on the control index.
 * @param motor_id The ID of the target motor (1-32).
 * @param control The function to execute from the FUNCTION_CONTROL_INDEX enum.
 * @param value The value associated with the chosen function.
 * @note BUG: The 'value' parameter should be uint32_t to handle all cases correctly.
 */
void MYACTUATOR_FUNCTION_CONTROL(uint8_t motor_id, FUNCTION_CONTROL_INDEX control, uint32_t value);

#endif /* MYACTUATOR_H_ */
