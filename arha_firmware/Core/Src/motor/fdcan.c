/*
 * fdcan.c
 *
 *  Created on: Jan 9, 2026
 *      Author: Kenneth Martinez
 */

#include "motor/fdcan.h"
#include "string.h"

#define RX_QUEUE_SIZE 16

static FDCAN_RxMessage_t rx_queue[RX_QUEUE_SIZE];
static volatile uint8_t rx_head = 0; //Write position
static volatile uint8_t rx_tail = 0; //Read position

HAL_StatusTypeDef FDCAN_Driver_init(void){
	FDCAN_FilterTypeDef filter_config;

    /* Filter range 0x241-0x246 for motor responses */
	filter_config.IdType = FDCAN_STANDARD_ID;
	filter_config.FilterIndex = 0;
	filter_config.FilterType = FDCAN_FILTER_RANGE;
	filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	filter_config.FilterID1 = 0x240 + FDCAN_MOTOR_ID_MIN; // 0x241
	filter_config.FilterID2 = 0x240 + FDCAN_MOTOR_ID_MAX; // 0x246

	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter_config) != HAL_OK){
		return HAL_ERROR;
	}

    /* Reject non-matching standard/extended IDs and remote frames */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_REJECT, FDCAN_REJECT, 
                                     FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK){
		return HAL_ERROR;
	}

	// Configure interrupt lines
	if (HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0) != HAL_OK){
		return HAL_ERROR;
	}

    /* Enable FIFO0 new message interrupt */
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
		return HAL_ERROR;
	}

	// Enable the NVIC Interrupt
	HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

	return HAL_OK;
}

HAL_StatusTypeDef FDCAN_Driver_Start(void){
	return HAL_FDCAN_Start(&hfdcan1);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){

	if (hfdcan == &hfdcan1 && (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)){
		
		FDCAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];

        /* Read message from hardware FIFO */
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK){

            /* Advance write index (circular buffer) */
			uint8_t next_head = (rx_head + 1) % RX_QUEUE_SIZE;

            /* Store if queue not full */
			if (next_head != rx_tail) {
				rx_queue[rx_head].can_id = rx_header.Identifier;
				rx_queue[rx_head].motor_id = rx_header.Identifier - 0x240;
				rx_queue[rx_head].command = rx_data[0];
				memcpy(rx_queue[rx_head].data, rx_data, 8);
				rx_queue[rx_head].data_length = (rx_header.DataLength >> 16);

				rx_head = next_head;
			}
		}
	}
}

bool FDCAN_Driver_GetMessage(FDCAN_RxMessage_t *msg){

    /* Check if queue empty */
	if (rx_head == rx_tail){
		return false;
	}

	memcpy(msg, &rx_queue[rx_tail], sizeof(FDCAN_RxMessage_t));

	rx_tail = (rx_tail + 1) % RX_QUEUE_SIZE;

	return true;
}

