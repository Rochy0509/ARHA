/*
 * fdcan.h
 *
 *  Created on: Jan 9, 2026
 *      Author: Kenneth Martinez
 */

#ifndef INC_FDCAN_H_
#define INC_FDCAN_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

#define FDCAN_MOTOR_ID_BASE 0x140
#define FDCAN_MOTOR_ID_MIN 1
#define FDCAN_MOTOR_ID_MAX 6

typedef struct {
	uint32_t can_id;
	uint8_t  motor_id;
	uint8_t  command;
	uint8_t  data[8];
	uint8_t  data_length;
} FDCAN_RxMessage_t;

/* FDCAN1 — right_arm */
HAL_StatusTypeDef FDCAN_Driver_init(void);
HAL_StatusTypeDef FDCAN_Driver_Start(void);
bool FDCAN_Driver_GetMessage(FDCAN_RxMessage_t *msg);

/* FDCAN2 — left_arm */
HAL_StatusTypeDef FDCAN2_Driver_Init(void);
HAL_StatusTypeDef FDCAN2_Driver_Start(void);
bool FDCAN2_Driver_GetMessage(FDCAN_RxMessage_t *msg);

/* FDCAN3 — neck */
HAL_StatusTypeDef FDCAN3_Driver_Init(void);
HAL_StatusTypeDef FDCAN3_Driver_Start(void);
bool FDCAN3_Driver_GetMessage(FDCAN_RxMessage_t *msg);

#endif /* INC_FDCAN_H_ */
