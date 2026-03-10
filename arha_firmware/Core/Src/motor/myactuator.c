#include "motor/myactuator.h"
#include "string.h"
#include "main.h"
#include "cmsis_os.h"

extern IWDG_HandleTypeDef hiwdg1;

void sendCANPacket(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, uint8_t* data){
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = 0x140 + motor_id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // Retries sending up to 5 times with a 1ms RTOS yield
    for (int retries = 0; retries < 5; retries++) {
        if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0) {
            if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) == HAL_OK) {
                return;
            }
        }
        // If buffer is full or hardware busy, yields to RTOS / LwIP / Watchdog
        HAL_IWDG_Refresh(&hiwdg1);
        osDelay(1);
    }
}

void MYACTUATOR_READ_PID(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, PID_PARAM_INDEX pid_index){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_PID;
	data[1] = pid_index;

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_WRITE_PID_TO_RAM(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, PID_PARAM_INDEX pid_index, float value){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = WRITE_PID_TO_RAM;
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

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_WRITE_PID_TO_ROM(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, PID_PARAM_INDEX pid_index, float value){
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

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_READ_ACCEL(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, ACCEL_INDEX accel_index){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_ACCEL;
	data[1] = accel_index;

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_WRITE_ACCEL_TO_ROM_RAM(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, ACCEL_INDEX accel_index, uint32_t accel_value){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = WRITE_ACCEL_TO_ROM_RAM;
	data[1] = accel_index;
	data[4] = (uint8_t)(accel_value & 0xFF);
	data[5] = (uint8_t)((accel_value >> 8) & 0xFF);
	data[6] = (uint8_t)((accel_value >> 16) & 0xFF);
	data[7] = (uint8_t)((accel_value >> 24) & 0xFF);

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_READ_MULTI_ENC_POS(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_MULTI_ENC_POS_DATA;

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_READ_MULTI_ENC_ORIGINAL_POS(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_MULTI_ORIG_POS;

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_READ_MULTI_ENC_ZERO_OFFSET(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_MULTI_ENC_ZERO_OFFSET;

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_WRITE_ENC_MULTI_TO_ROM_AS_MOTOR_ZERO(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t enc_offset){
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
	data[6] = (uint8_t)((converter.u32 >> 16) & 0xFF);
	data[7] = (uint8_t)((converter.u32 >> 24) & 0xFF);

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_WRITE_CURRENT_MULTI_POS_ENC_TO_ROM_AS_MOTOR_ZERO(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = WRITE_CURRENT_MULTI_POS_ENC_ROM_MZERO;

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_READ_SINGLE_ENC(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_SINGLE_ENC;

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_READ_MULTI_ENC_ANGLE(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_MULTI_ENC_ANGLE;

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_READ_SINGLE_ENC_ANGLE(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_SINGLE_T_ANGLE;

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_READ_MOTOR_STATUS_1(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_MOTOR_STATUS_1;

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_READ_MOTOR_STATUS_2(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_MOTOR_STATUS_2;

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_READ_MOTOR_STATUS_3(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_MOTOR_STATUS_3;

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_MOTOR_SHUTDOWN(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = MOTOR_SHUTDOWN;

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_MOTOR_STOP(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = MOTOR_STOP;

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_TORQUE_CL_CONTROL(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int16_t torque_value){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = TORQUE_CL_CONTROL;

	union{
		int16_t i16;
		uint16_t u16;
	} converter;

	converter.i16 = torque_value;

	data[4] = (uint8_t)(converter.u16 & 0xFF);
	data[5] = (uint8_t)((converter.u16 >> 8) & 0xFF);

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_SPEED_CL_CONTROL(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t speed_value){

	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = SPEED_CL_CONTROL;

	union{
		int32_t i32;
		uint32_t u32;
	} converter;

	converter.i32 = speed_value;

	data[4] = (uint8_t)(converter.u32 & 0xFF);
	data[5] = (uint8_t)((converter.u32 >> 8) & 0xFF);
	data[6] = (uint8_t)((converter.u32 >> 16) & 0xFF);
	data[7] = (uint8_t)((converter.u32 >> 24) & 0xFF);

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_ABS_POS_CL_CONTROL(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int16_t speed_limit, float pos){
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

	pos_converter.i32 = (int32_t)(pos * 100.0f);
	data[4] = (uint8_t)(pos_converter.u32 & 0xFF);
	data[5] = (uint8_t)((pos_converter.u32 >> 8) & 0xFF);
	data[6] = (uint8_t)((pos_converter.u32 >> 16) & 0xFF);
	data[7] = (uint8_t)((pos_converter.u32 >> 24) & 0xFF);

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_SINGLE_POS_CONTROL(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, SpinDirection direction, uint16_t speed_limit, uint16_t position){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = SINGLE_POSITION_CONTROL;
	data[1] = direction;

	union {
		uint16_t u16;
	} converter;

	converter.u16 = speed_limit;
	data[2] = (uint8_t)(converter.u16 & 0xFF);
	data[3] = (uint8_t)((converter.u16 >> 8) & 0xFF);

	converter.u16 = position;
	data[4] = (uint8_t)(converter.u16 & 0xFF);
	data[5] = (uint8_t)((converter.u16 >> 8) & 0xFF);

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_INC_POS_CL_CONTROL(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, uint16_t speed_limit, int32_t pos){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = INC_POS_CL_CONTROL;

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

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_READ_SYS_OP_MODE(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = SYS_OP_MODE;

	sendCANPacket(hfdcan, motor_id, data);

}

void MYACTUATOR_RESET_MOTOR(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = SYS_RESET;

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_READ_SYS_RUNTIME(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = READ_SYS_RUNTIME;

	sendCANPacket(hfdcan, motor_id, data);
}

void MYACTUATOR_SET_BAUD_RATE(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, BAUD_RATE_INDEX baud){
	uint8_t data[8];
	memset(data, 0, 8);

	data[0] = SET_COMM_BAUD_RATE;
	data[7] = baud;

	sendCANPacket(hfdcan, motor_id, data);
}
