#ifndef MYACTUATOR_H_
#define MYACTUATOR_H_

#include "stm32h7xx_hal.h"

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

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

void sendCANPacket(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, uint8_t* data);

void MYACTUATOR_READ_PID(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, PID_PARAM_INDEX pid_index);
void MYACTUATOR_WRITE_PID_TO_RAM(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, PID_PARAM_INDEX pid_index, float value);
void MYACTUATOR_WRITE_PID_TO_ROM(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, PID_PARAM_INDEX pid_index, float value);
void MYACTUATOR_READ_ACCEL(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, ACCEL_INDEX accel_index);
void MYACTUATOR_WRITE_ACCEL_TO_ROM_RAM(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, ACCEL_INDEX accel_index, uint32_t accel_value);
void MYACTUATOR_READ_MULTI_ENC_POS(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void MYACTUATOR_READ_MULTI_ENC_ORIGINAL_POS(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void MYACTUATOR_READ_MULTI_ENC_ZERO_OFFSET(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void MYACTUATOR_WRITE_ENC_MULTI_TO_ROM_AS_MOTOR_ZERO(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t enc_offset);
void MYACTUATOR_WRITE_CURRENT_MULTI_POS_ENC_TO_ROM_AS_MOTOR_ZERO(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void MYACTUATOR_READ_SINGLE_ENC(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void MYACTUATOR_READ_MULTI_ENC_ANGLE(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void MYACTUATOR_READ_SINGLE_ENC_ANGLE(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void MYACTUATOR_READ_MOTOR_STATUS_1(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void MYACTUATOR_READ_MOTOR_STATUS_2(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void MYACTUATOR_READ_MOTOR_STATUS_3(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void MYACTUATOR_MOTOR_SHUTDOWN(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void MYACTUATOR_MOTOR_STOP(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void MYACTUATOR_TORQUE_CL_CONTROL(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int16_t torque_value);
void MYACTUATOR_SPEED_CL_CONTROL(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t speed_value);
void MYACTUATOR_ABS_POS_CL_CONTROL(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int16_t speed_limit, float pos);
void MYACTUATOR_SINGLE_POS_CONTROL(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, SpinDirection direction, uint16_t speed_limit, uint16_t position);
void MYACTUATOR_INC_POS_CL_CONTROL(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, uint16_t speed_limit, int32_t pos);
void MYACTUATOR_READ_SYS_OP_MODE(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void MYACTUATOR_RESET_MOTOR(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void MYACTUATOR_READ_SYS_RUNTIME(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void MYACTUATOR_SET_BAUD_RATE(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, BAUD_RATE_INDEX baud);

#endif /* MYACTUATOR_H_ */
