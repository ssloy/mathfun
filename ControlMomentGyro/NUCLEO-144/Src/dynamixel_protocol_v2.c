#include <dynamixel_protocol_v2.h>

//legthOfTxPacket = 0;
uint8_t answerPacketSize = 0;
uint8_t servoCount = 0;


volatile uint16_t commRxBufferIdx = 0;
volatile uint16_t commTxBufferIdx = 0;

uint8_t int16_hl[2];
uint8_t int32_separate[4];

void bindDynamixelUART(UART_HandleTypeDef* _uart){
	dynamixelUART = _uart;
//	__HAL_UART_ENABLE_IT(_uart, UART_IT_RXNE);
}

void initServo(Servo* servo,uint8_t _ID){
	servo->ID = _ID;
	servoCount++;
}

void instructionPacketAssebly(uint8_t _ID, uint8_t _instruction, uint8_t* _parameters, uint8_t _numberOfParameters){
	    instructionPacket[0] = HEADER;
	    instructionPacket[1] = HEADER;
	    instructionPacket[2] = HEADER2;
	    instructionPacket[3] = RESERVED;
	    instructionPacket[4] = _ID;

	    uint16_t lenght = _numberOfParameters + 3;
		memcpy(&int16_hl, &lenght, 2);
		uint8_t Lenght_H = int16_hl[1];
		uint8_t Lenght_L = int16_hl[0];

	    instructionPacket[5] = Lenght_L; // length
	    instructionPacket[6] = Lenght_H; // length

	    instructionPacket[7] = _instruction;

	    for (int i = 1; i < _numberOfParameters+1; i++){
	    	instructionPacket[7 + i] = _parameters[i-1];
	    }

	    legthOfTxPacket = lenght + 7;//final packet size for transmitting

	    unsigned short crc = update_crc(0, instructionPacket, legthOfTxPacket-2);
		memcpy(&int16_hl, &crc, 2);
		uint8_t CRC_H = int16_hl[1];
		uint8_t CRC_L = int16_hl[0];
//		uint8_t CRC_L = (crc & 0x00FF); //Little-endian
//		uint8_t CRC_H = (crc>>8) & 0x00FF;

	    instructionPacket[legthOfTxPacket-2] = CRC_L; //CRC field
	    instructionPacket[legthOfTxPacket-1] = CRC_H; //CRC field

	    if (_instruction == COMMAND_WRITE_DATA)
	    {
	    	answerPacketSize = 11;
	    } else if(_instruction == COMMAND_SYNC_WRITE) {
	    	answerPacketSize = 11;
	    } else if (_instruction == COMMAND_READ_DATA){
	    	uint16_t assembledSizeVar = 0;
	    	memcpy(&assembledSizeVar, &instructionPacket[10], 2);
	    	answerPacketSize = assembledSizeVar + 11;
	    } else if (_instruction == COMMAND_SYNC_READ){
	    	uint16_t assembledSizeVar = 0;
	    	memcpy(&assembledSizeVar, &instructionPacket[10], 2);
	    	answerPacketSize = (assembledSizeVar + 11) * NUMBER_OF_SERVOS;
	    }
}

void transmitInstructionPacket(void){
//?	HAL_GPIO_WritePin(RSE_485_GPIO_Port, RSE_485_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(dynamixelUART,instructionPacket,legthOfTxPacket,4);
//?	HAL_GPIO_WritePin(RSE_485_GPIO_Port, RSE_485_Pin, GPIO_PIN_RESET);
	HAL_UART_Receive(dynamixelUART,commRxBuffer,answerPacketSize+1,4);
//?	HAL_GPIO_WritePin(RSE_485_GPIO_Port, RSE_485_Pin, GPIO_PIN_SET);
}

void setPosition(uint32_t position, Servo* servo){
	memcpy(&int32_separate, &position, 4);

	uint16_t reg_addr = 116;
	memcpy(&int16_hl, &reg_addr, 2);
	uint8_t REG_H = int16_hl[1];
	uint8_t REG_L = int16_hl[0];

  	parameters[0] = REG_L;
  	parameters[1] = REG_H;
  	parameters[2] = int32_separate[0];
  	parameters[3] = int32_separate[1];
  	parameters[4] = int32_separate[2];
  	parameters[5] = int32_separate[3];


  	instructionPacketAssebly(servo->ID, COMMAND_WRITE_DATA, parameters, 6);
  	transmitInstructionPacket();
}

void setProfileVelocity(int32_t velocity, Servo* servo){

	memcpy(&int32_separate, &velocity, 4);

	uint16_t reg_addr = 112;
	memcpy(&int16_hl, &reg_addr, 2);
	uint8_t REG_H = int16_hl[1];
	uint8_t REG_L = int16_hl[0];

  	parameters[0] = REG_L;
  	parameters[1] = REG_H;
  	parameters[2] = int32_separate[0];
  	parameters[3] = int32_separate[1];
  	parameters[4] = int32_separate[2];
  	parameters[5] = int32_separate[3];

  	instructionPacketAssebly(servo->ID, COMMAND_WRITE_DATA, parameters, 6);
  	transmitInstructionPacket();
}

void setOperatingMode(uint8_t mode, Servo* servo){
	uint16_t reg_addr = 11;
	memcpy(&int16_hl, &reg_addr, 2);
	uint8_t REG_H = int16_hl[1];
	uint8_t REG_L = int16_hl[0];

  	parameters[0] = REG_L;
  	parameters[1] = REG_H;
  	parameters[2] = mode;
  	instructionPacketAssebly(servo->ID, COMMAND_WRITE_DATA, parameters, 3);

  	transmitInstructionPacket();
//  	HAL_Delay(100);
}

void setVelocity(int32_t velocity, Servo* servo) {
	memcpy(&int32_separate, &velocity, 4);

	uint16_t reg_addr = 104;
	memcpy(&int16_hl, &reg_addr, 2);
	uint8_t REG_H = int16_hl[1];
	uint8_t REG_L = int16_hl[0];

  	parameters[0] = REG_L;
  	parameters[1] = REG_H;
  	parameters[2] = int32_separate[0];
  	parameters[3] = int32_separate[1];
  	parameters[4] = int32_separate[2];
  	parameters[5] = int32_separate[3];

  	instructionPacketAssebly(servo->ID, COMMAND_WRITE_DATA, parameters, 6);
  	transmitInstructionPacket();
}

void enableToque(uint8_t isEnable, Servo* servo){
	uint16_t reg_addr = 64;
	memcpy(&int16_hl, &reg_addr, 2);
	uint8_t REG_H = int16_hl[1];
	uint8_t REG_L = int16_hl[0];

  	parameters[0] = REG_L;
  	parameters[1] = REG_H;
  	parameters[2] = isEnable;
  	instructionPacketAssebly(servo->ID, COMMAND_WRITE_DATA, parameters, 3);

  	transmitInstructionPacket();
//  	HAL_Delay(100);
}

void readData(Servo* servo, uint16_t startAdress, uint16_t sizeOfData){

	memcpy(&int16_hl, &startAdress, 2);
	uint8_t START_ADRESS_H = int16_hl[1];
	uint8_t START_ADRESS_L = int16_hl[0];

	parameters[0] = START_ADRESS_L;
	parameters[1] = START_ADRESS_H;

	memcpy(&int16_hl, &sizeOfData, 2);
	uint8_t SIZE_OF_DATA_H = int16_hl[1];
	uint8_t SIZE_OF_DATA_L = int16_hl[0];

	parameters[2] = SIZE_OF_DATA_L;
	parameters[3] = SIZE_OF_DATA_H;


	instructionPacketAssebly(servo->ID, COMMAND_READ_DATA, parameters, 4);
	transmitInstructionPacket();
}

//void writeData(uint8_t _ID,  ){
//
//}

void syncWrite(uint8_t _servoCount, dataToSyncWrite* _dataToWrite, uint16_t _dataLenght, uint16_t _startAdress){
//	while (rxCompleteFlag != 1){
//	}
	dataToSyncWrite as = _dataToWrite[1];
	memcpy(&int16_hl, &_startAdress, 2);
	uint8_t START_ADRESS_H = int16_hl[1];
	uint8_t START_ADRESS_L = int16_hl[0];

	parameters[0] = START_ADRESS_L;
	parameters[1] = START_ADRESS_H;

	memcpy(&int16_hl, &_dataLenght, 2);
	uint8_t SIZE_OF_DATA_H = int16_hl[1];
	uint8_t SIZE_OF_DATA_L = int16_hl[0];

	parameters[2] = SIZE_OF_DATA_L;
	parameters[3] = SIZE_OF_DATA_H;

	int offset = 0;
	for (int i = 0; i <_servoCount; i++){
		parameters[i+4+offset] = i; //ID of servo
		for (int a = 0; a < _dataLenght; a++){
			parameters[a+5+offset+i] = _dataToWrite[i].data[a];
		}
		offset += _dataLenght;
	}

  	instructionPacketAssebly(BROADCAST_ID, COMMAND_SYNC_WRITE, parameters, _dataLenght*_servoCount+2 + 4);

  	transmitInstructionPacket();

}

void syncRead(Servo* _servoArray, uint8_t _servoCount, uint16_t _startAdress, uint16_t _dataLenght){
	memcpy(&int16_hl, &_startAdress, 2);
	uint8_t START_ADRESS_H = int16_hl[1];
	uint8_t START_ADRESS_L = int16_hl[0];

	parameters[0] = START_ADRESS_L;
	parameters[1] = START_ADRESS_H;

	memcpy(&int16_hl, &_dataLenght, 2);
	uint8_t SIZE_OF_DATA_H = int16_hl[1];
	uint8_t SIZE_OF_DATA_L = int16_hl[0];

	parameters[2] = SIZE_OF_DATA_L;
	parameters[3] = SIZE_OF_DATA_H;

	for (int i = 0; i <_servoCount; i++){
		parameters[i+4] = _servoArray[i].ID;
	}

	instructionPacketAssebly(BROADCAST_ID, COMMAND_SYNC_READ, parameters, 4 + _servoCount);
	transmitInstructionPacket();
}

void setID(uint8_t _ID, Servo* servo){
	parameters[0] = EEPROM_ID;
  	parameters[1] = _ID;

  	instructionPacketAssebly(BROADCAST_ID, COMMAND_WRITE_DATA, parameters, 2);

  	transmitInstructionPacket();
    HAL_Delay(100);
}

void setBaudrate(uint8_t _baudrate){
	parameters[0] = EEPROM_BAUD_RATE;
  	parameters[1] = _baudrate;

  	instructionPacketAssebly(BROADCAST_ID, COMMAND_WRITE_DATA, parameters, 2);
  	transmitInstructionPacket();
}

uint8_t dynamixelIRQHandler(UART_HandleTypeDef* huart) {
	if (dynamixelUART->Instance == huart->Instance) {
	if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE) != RESET)) {

		rxCompleteFlag = 0;
		commRxBuffer[commRxBufferIdx++] = huart->Instance->RDR;

		if (commRxBufferIdx == answerPacketSize)
		{
			commRxBufferIdx = 0;
			rxCompleteFlag = 1;
			rxCompleteCalback();


		}
		__HAL_UART_FLUSH_DRREGISTER(huart);
		return 1;
	}

	if((__HAL_UART_GET_FLAG(huart, UART_FLAG_TXE) != RESET)	&& (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE) != RESET)){

		if(commTxBufferIdx < legthOfTxPacket+1){
			huart->Instance->TDR = instructionPacket[commTxBufferIdx++];
		} else{
			commTxBufferIdx = 0;

//?			HAL_GPIO_WritePin(RSE_485_GPIO_Port,RSE_485_Pin,RESET);
			__HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
			//CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE);
			//__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
		}
		return 1;
	}

}
__HAL_UART_FLUSH_DRREGISTER(huart);
return 0;
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){

	if (huart == dynamixelUART){
//		dx_tx_complete = 1;

//?		HAL_GPIO_WritePin(RSE_485_GPIO_Port, RSE_485_Pin, GPIO_PIN_RESET);
		HAL_UART_Receive(dynamixelUART,commRxBuffer,answerPacketSize,HAL_MAX_DELAY);
		//HAL_UART_Receive_IT(dynamixelUART,commRxBuffer,answerPacketSize);

		return;
	}

}


unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size) {
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++) {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}
