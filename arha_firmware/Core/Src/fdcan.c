/*
 * fdcan.c
 *
 *  Created on: Jan 9, 2026
 *      Author: Kenneth Martinez
 */

#include "fdcan.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"

#define RX_QUEUE_SIZE 64

/* ─── Per-bus RX ring buffers ─── */

static FDCAN_RxMessage_t rx_queue1[RX_QUEUE_SIZE];
static volatile uint8_t rx_head1 = 0;
static volatile uint8_t rx_tail1 = 0;

static FDCAN_RxMessage_t rx_queue2[RX_QUEUE_SIZE];
static volatile uint8_t rx_head2 = 0;
static volatile uint8_t rx_tail2 = 0;

static FDCAN_RxMessage_t rx_queue3[RX_QUEUE_SIZE];
static volatile uint8_t rx_head3 = 0;
static volatile uint8_t rx_tail3 = 0;

/* ─── Shared filter/interrupt setup per FDCAN instance ─── */

static HAL_StatusTypeDef fdcan_driver_setup(FDCAN_HandleTypeDef *hfdcan, IRQn_Type irqn) {
	FDCAN_FilterTypeDef filter_config;

	filter_config.IdType = FDCAN_STANDARD_ID;
	filter_config.FilterIndex = 0;
	filter_config.FilterType = FDCAN_FILTER_RANGE;
	filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	filter_config.FilterID1 = 0x240 + FDCAN_MOTOR_ID_MIN;
	filter_config.FilterID2 = 0x240 + FDCAN_MOTOR_ID_MAX;

	if (HAL_FDCAN_ConfigFilter(hfdcan, &filter_config) != HAL_OK) {
		return HAL_ERROR;
	}

	if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
	                                  FDCAN_REJECT, FDCAN_REJECT,
	                                  FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
		return HAL_ERROR;
	}

	if (HAL_FDCAN_ConfigInterruptLines(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0) != HAL_OK) {
		return HAL_ERROR;
	}

	if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
	                                            FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
		return HAL_ERROR;
	}

	HAL_NVIC_SetPriority(irqn, 1, 0);
	HAL_NVIC_EnableIRQ(irqn);

	return HAL_OK;
}

/* ─── FDCAN1 — right_arm ─── */

HAL_StatusTypeDef FDCAN_Driver_init(void) {
	return fdcan_driver_setup(&hfdcan1, FDCAN1_IT0_IRQn);
}

HAL_StatusTypeDef FDCAN_Driver_Start(void) {
	return HAL_FDCAN_Start(&hfdcan1);
}

bool FDCAN_Driver_GetMessage(FDCAN_RxMessage_t *msg) {
	if (rx_head1 == rx_tail1) {
		return false;
	}
	memcpy(msg, &rx_queue1[rx_tail1], sizeof(FDCAN_RxMessage_t));
	rx_tail1 = (rx_tail1 + 1) % RX_QUEUE_SIZE;
	return true;
}

/* ─── FDCAN2 — left_arm ─── */

HAL_StatusTypeDef FDCAN2_Driver_Init(void) {
	return fdcan_driver_setup(&hfdcan2, FDCAN2_IT0_IRQn);
}

HAL_StatusTypeDef FDCAN2_Driver_Start(void) {
	return HAL_FDCAN_Start(&hfdcan2);
}

bool FDCAN2_Driver_GetMessage(FDCAN_RxMessage_t *msg) {
	if (rx_head2 == rx_tail2) {
		return false;
	}
	memcpy(msg, &rx_queue2[rx_tail2], sizeof(FDCAN_RxMessage_t));
	rx_tail2 = (rx_tail2 + 1) % RX_QUEUE_SIZE;
	return true;
}

/* ─── FDCAN3 — neck ─── */

HAL_StatusTypeDef FDCAN3_Driver_Init(void) {
	return fdcan_driver_setup(&hfdcan3, FDCAN3_IT0_IRQn);
}

HAL_StatusTypeDef FDCAN3_Driver_Start(void) {
	return HAL_FDCAN_Start(&hfdcan3);
}

bool FDCAN3_Driver_GetMessage(FDCAN_RxMessage_t *msg) {
	if (rx_head3 == rx_tail3) {
		return false;
	}
	memcpy(msg, &rx_queue3[rx_tail3], sizeof(FDCAN_RxMessage_t));
	rx_tail3 = (rx_tail3 + 1) % RX_QUEUE_SIZE;
	return true;
}

/* ─── Shared RX callback — routes to per-bus queue ─── */

static void push_to_queue(FDCAN_HandleTypeDef *hfdcan,
                          FDCAN_RxMessage_t *queue, volatile uint8_t *head, volatile uint8_t *tail) {
	while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
		FDCAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];

		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
			uint8_t next_head = (*head + 1) % RX_QUEUE_SIZE;
			if (next_head != *tail) {
				queue[*head].can_id = rx_header.Identifier;
				queue[*head].motor_id = rx_header.Identifier - 0x240;
				queue[*head].command = rx_data[0];
				memcpy(queue[*head].data, rx_data, 8);
				queue[*head].data_length = (rx_header.DataLength >> 16);
				*head = next_head;
			}
		} else {
			break;
		}
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) return;

	if (hfdcan == &hfdcan1) {
		push_to_queue(hfdcan, rx_queue1, &rx_head1, &rx_tail1);
	} else if (hfdcan == &hfdcan2) {
		push_to_queue(hfdcan, rx_queue2, &rx_head2, &rx_tail2);
	} else if (hfdcan == &hfdcan3) {
		push_to_queue(hfdcan, rx_queue3, &rx_head3, &rx_tail3);
	}
}

/* ─── Error callbacks ─── */

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan) {
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
	// Only tears down if bus is OFF. Restarting on WARNING wipes TX FIFO.
	if (ErrorStatusITs & FDCAN_IT_BUS_OFF) {
		HAL_FDCAN_Stop(hfdcan);
		HAL_FDCAN_Start(hfdcan);
	}
}
