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

//IDs definitions for filtering them from 1 to 6
#define FDCAN_MOTOR_ID_BASE 0x140
#define FDCAN_MOTOR_ID_MIN 1
#define FDCAN_MOTOR_ID_MAX 6

//Data structure to store the messages upcoming into Rx
typedef struct {
	uint32_t can_id;
	uint8_t  motor_id;
	uint8_t  command;
	uint8_t  data[8];
	uint8_t  data_length;
} FDCAN_RxMessage_t;

//Initialization and control functions
HAL_StatusTypeDef FDCAN_Driver_init(void); //for configuring RX filters
HAL_StatusTypeDef FDCAN_Driver_Start(void); // Call HAL_FDCAN_start() to start TX/RX

//To check if there is any message available
bool FDCAN_Driver_GetMessage(FDCAN_RxMessage_t *msg);

#endif /* INC_FDCAN_H_ */
