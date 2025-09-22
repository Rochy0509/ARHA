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

void MYACTUATOR_READ_ACCEL(uint8_t motor_id, ACCEL_INDEX accel_index){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_ACCEL;
	data[1] = accel_index;

	sendCANPacket(motor_id, data);
}

void MYACTUATOR_WRITE_ACCEL_TO_ROM_RAM(uint8_t motor_id, ACCEL_INDEX accel_index, uint32_t accel_value){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = WRITE_ACCEL_TO_ROM_RAM;
	data[1] = accel_index;
	data[4] = (uint8_t)(accel_value & 0xFF);
	data[5] = (uint8_t)((accel_value >> 8) & 0xFF);
	data[6] = (uint8_t)((accel_value >> 16) & 0xFF);
	data[7] = (uint8_t)((accel_value >> 24) & 0xFF);

	sendCANPacket(motor_id, data);
}

void MYACTUATOR_READ_MULTI_ENC_POS(uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_MULTI_ENC_POS_DATA;

	sendCANPacket(motor_id, data);
}

void MYACTUATOR_READ_MULTI_ENC_ORIGINAL_POS(uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_MULTI_ORIG_POS;

	sendCANPacket(motor_id, data);
}

void MYACTUATOR_READ_MULTI_ENC_ZERO_OFFSET(uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_MULTI_ENC_ZERO_OFFSET;

	sendCANPacket(motor_id, data);
}

void MYACTUATOR_WRITE_ENC_MULTI_TO_ROM_AS_MOTOR_ZERO(uint8_t motor_id, int32_t enc_offset){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = WRITE_ENC_MULTI_VALUE_TO_ROM_AS_MZERO;

	union{
		int32_t i32;
		uint32_t u32;
	} converter;

	converter.i32 = enc_offset;
	data[4] = (uint8_t)(converter.u32 & 0xFF);
	data[5] = (uint8_t)((converter.u32 >> 8) & 0xFF);
	data[6] = (uint8_t)((converter.u32 >> 8) & 0xFF);
	data[7] = (uint8_t)((converter.u32 >> 8) & 0xFF);

	sendCANPacket(motor_id, data);
}

void MYACTUATOR_WRITE_CURRENT_MULTI_POS_ENC_TO_ROM_AS_MOTOR_ZERO(uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = WRITE_CURRENT_MULTI_POS_ENC_ROM_MZERO;

	sendCANPacket(motor_id, data);
}

void MYACTUATOR_READ_SINGLE_ENC(uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_SINGLE_ENC;

	sendCANPacket(motor_id, data);
}

void MYACTUATOR_READ_MULTI_ENC_ANGLE(uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_MULTI_ENC_ANGLE;

	sendCANPacket(motor_id, data);
}

void MYACTUATOR_READ_SINGLE_ENC_ANGLE(uint8_t motor_id){{
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_SINGLE_T_ANGLE;

	sendCANPacket(motor_id, data);
}

void MYACTUATOR_READ_MOTOR_STATUS_1(uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_MOTOR_STATUS_1;

	sendCANPacket(motor_id, data);
}

void MYACTUATOR_TORQUE_CL_CONTROL(uint8_t motor_id, int16_t torque_value){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = TORQUE_CL_CONTROL;

	union{
		int16_t i16;
		uint16_t u16;
	} converter;

	converter.f = torque_value;

	data[4] = (uint8_t)(converter.u16 & 0xFF);
	data[5] = (uint8_t)((converter.u16 >> 8) & 0xFF);

	sendCANPacket(motor_id, data);
}

void MYACTUATOR_SPEED_CL_CONTROL(uint8_t motor_id, int32_t speed_value){

	if (speed_value == 0) {
		char error_msg[] = "ERROR: Speed cannot be 0.\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
		return;
	}

	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = SPEED_CL_CONTROL;

	union{
		int32_t i32;
		uint32_t u32;
	} converter;

	converter.f = speed_value;

	data[4] = (uint8_t)(converter.u32 & 0xFF);
	data[5] = (uint8_t)((converter.u32 >> 8) & 0xFF);
	data[6] = (uint8_t)((converter.u32 >> 16) & 0xFF);
	data[7] = (uint8_t)((converter.u32 >> 24) & 0xFF);

	sendCANPacket(motor_id, data);
}

void MYACTUATOR_ABS_POS_CL_CONTROL(uint8_t motor_id, uint16_t speed_limit, int32_t pos){

	if (speed_limit == 0) {
		char error_msg[] = "ERROR: Speed cannot be 0.\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
		return;
	}

	if (pos > 36000000 || pos < -36000000) {
		char error_msg[] = "ERROR: Position out of range value must be between 0 and 360\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
		return;
	}

	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = ABSOLUTE_POS_CL_CONTROL;

	union {
		uint16_t u16;
	} speed_converter;

	speed_converter.u16 = speed_limit;
	data[2] = (uint8_t)(speed_converter.u16 & 0xFF);
	data[3] = (uint8_t)((speed_converter.u16 >> 8) & 0xFF);

	union {
		int32_t i32;
		uint32_t u32;
	} pos_converter;

	pos_converter.i32 = pos;
	data[4] = (uint8_t)(pos_converter.u32 & 0xFF);
	data[5] = (uint8_t)((pos_converter.u32 >> 8) & 0xFF);
	data[6] = (uint8_t)((pos_converter.u32 >> 16) & 0xFF);
	data[7] = (uint8_t)((pos_converter.u32 >> 24) & 0xFF);

	sendCANPacket(motor_id, data);
}
