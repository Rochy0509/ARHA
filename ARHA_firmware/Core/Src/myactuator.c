#include "myactuator.h"
#include "string.h"


void sendCANPacket(uint8_t motor_id, uint8_t* data){
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t            TxMailbox;

	TxHeader.StdId = 0x140 + motor_id;
	TxHeader.ExtId = 0;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;

	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox);
}

void MYACTUATOR_READ_PID(uint8_t motor_id,  PID_PARAM_INDEX pid_index){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = WRITE_PID_TO_RAM;
	data[1] = pid_index;

	sendCANPacket(motor_id, data);
}

void MYACTUATOR_WRITE_PID_TO_RAM(uint8_t motor_id, PID_PARAM_INDEX pid_index, float value){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_PID;
	data[1] = pid_index;

	union {
		float f;
		uint32_t u32;
	} converter;

	converter.f = value;

	data[4] = (uint8_t)(converter.u32 & 0xFF);
	data[5] = (uint8_t)((converter.u32 >> 8) & 0xFF);
	data[6] = (uint8_t)((converter.u32 >> 16)& 0xFF);
	data[7] = (uint8_t)((converter.u32 >> 24)& 0xFF);

	sendCANPacket(motor_id, data);
}

void MYACTUATOR_WRITE_PID_TO_ROM(uint8_t motor_id, PID_PARAM_INDEX pid_index, float value){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = WRITE_PID_TO_ROM;
	data[1] = pid_index;

	union{
		float f;
		uint32_t u32;
	} converter;

	converter.f = value;

	data[4] = (uint8_t)(converter.u32 & 0xFF);
	data[5] = (uint8_t)((converter.u32 >> 8) & 0xFF);
	data[6] = (uint8_t)((converter.u32 >> 16) & 0xFF);
	data[7] = (uint8_t)((converter.u32 >> 24) & 0xFF);

	sendCANPacket(motor_id, data);
}


void MYACTUATOR_READ_MOTOR_STATUS_1(uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_MOTOR_STATUS_1;

	sendCANPacket(motor_id, data);
}
