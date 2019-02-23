#include "gl_svg.h"

#include "string.h"
#include "math.h"

extern int megacounter;

#ifdef GLVG_485
#define GLVG_PACKET_SHIFT 	4
#else
#define GLVG_PACKET_SHIFT 	0
#endif

#define TX_BUFFER_SIZE		30

#define GL_RESET 			1
#define GL_REINIT			2
#define GL_REINIT_WAITING	3
#define GL_NONE				0

static uint8_t GLVG_frame[GYRO_FRAME_SIZE];
static uint8_t rxBuffer[GYRO_FRAME_SIZE];
static uint8_t txBuffer[TX_BUFFER_SIZE];

orientationPacket1Struct svg_data1;
orientationPacket2Struct svg_data2;
static uint32_t GLVG_statusWord;

static bool packetStarted = false;

static uint8_t packetType = 0;
static uint8_t packetLength = 0;

uint8_t type = 0;
uint8_t size = 0;

static uint8_t rxBufferIdx = 0;
static uint8_t txBufferIdx = 0;

static uint8_t commandSize = 0;
static bool commandSent = true;
static uint32_t commandSentTime;

static uint16_t validPacks, wholePacks;

static uint32_t lastPacketTime;
static uint8_t GLVG_command = GL_NONE;
static uint8_t GLVG_nextCommand = GL_NONE;
static bool GLVG_firstPacketReceived = false;
static bool GLVG_coordinatesInitialized = false;

static uint32_t GLVG_latitude = 0;
static uint32_t GLVG_longitude = 0;
static uint32_t GLVG_height = 0;
static uint32_t GLVG_installCommandId = 4359;

UART_HandleTypeDef * GLVG_uart;

void GLVG_init(UART_HandleTypeDef * uart, float * coordinates){
	GLVG_uart = uart;
	GLVG_latitude = (uint32_t)(coordinates[0] * M_PI / 180 * 100000000);
	GLVG_longitude = (uint32_t)(coordinates[1] * M_PI / 180 * 100000000);
	GLVG_height = (uint32_t)(coordinates[2] * 100000);

	__HAL_UART_ENABLE_IT(GLVG_uart, UART_IT_RXNE);
  	
}

static float GLVG_calculateDistance(float lat1, float lon1, float lat2, float lon2){
	float R = 6371e3;
	float f1 = lat1;
	float f2 = lat2;
	float df = (lat2 - lat1);
	float dy = (lon2 - lon1);

	float a = sin(df / 2.0) * sin(df / 2.0) +
			  cos(f1) * cos(f2) * sin(dy / 2.0)*
			  sin(dy / 2.0);
	float c = 2 * atan2(sqrt(a), sqrt(1-a));

	return R * c;

}

static int GLVG_findPacketIndex(uint8_t * buffer, uint8_t buffer_length){
	uint8_t i = 0;

	for(; i < buffer_length - 2 && ((buffer[i] != 0xAA) | (buffer[i+1] != 0xAA)
									| (buffer[i + 2] != 0x37)); ++i);

	if(i + BINS_FRAME_LENGTH < buffer_length) return i;

	return -1;
}

static uint16_t GLVG_calcCRC(uint8_t * buffer, uint8_t size){
	uint16_t wCrc = 0x00;
	uint8_t i, j;

	for (i = 0; i < size; i++) {
		wCrc ^= buffer[i] << 8;
		for (j = 0; j < 8; j++)
			wCrc = wCrc & 0x8000 ? (wCrc << 1) ^ 0x1021 : wCrc << 1;
	}

	wCrc = wCrc & 0xffff;
	return wCrc;
}

void GLVG_processCommands(){
	uint8_t status = (svg_data1.status >> 28) & 0b11;

	float distance = 0;
	if(GLVG_isFirstPacketReceived() && !GLVG_coordinatesInitialized){
		distance = GLVG_calculateDistance(svg_data1.latitude / 100000000.0f, svg_data1.longtitude / 100000000.0f,
						 GLVG_latitude / 100000000.0f, GLVG_longitude / 100000000.0f);
	}

	switch(GLVG_command){
		case GL_REINIT:
			if((status == 0b01 || status == 0b11)
						&& commandSent){
				txBuffer[0] = 0xAA;
				txBuffer[1] = 0xAA;
				txBuffer[2] = 23;
				txBuffer[3] = 0x45;
				memcpy(&txBuffer[4], &GLVG_latitude, 4);
				memcpy(&txBuffer[8], &GLVG_longitude, 4);
				memcpy(&txBuffer[12], &GLVG_height, 4);
				memcpy(&txBuffer[20], &GLVG_installCommandId, 4);

				uint16_t crc = GLVG_calcCRC(txBuffer + 3, 21);
				txBuffer[24] = crc & 0xFF;
				txBuffer[25] = (crc >> 8) & 0xFF;

				commandSize	= 26;
				commandSent	= false;
				GLVG_command = GL_REINIT_WAITING;

				__HAL_UART_DISABLE_IT(GLVG_uart, UART_IT_RXNE);
				__HAL_UART_ENABLE_IT(GLVG_uart, UART_IT_TXE);

			}else if(status == 0b10){
				GLVG_command = GL_RESET;
			}
			break;
		case GL_RESET:
			GLVG_reset();
			GLVG_command = GL_REINIT;
			break;
		case GL_REINIT_WAITING:
			if(commandSent){
				if(distance > 5000 && commandSentTime > 2000){
					GLVG_command = GL_REINIT;
				}else{
					GLVG_command = GL_NONE;
				}
			}
			break;
		case GL_NONE:
			if(GLVG_isFirstPacketReceived()){
				if(distance > 5000){
					GLVG_command = GL_REINIT;
				}else{
					GLVG_coordinatesInitialized = true;
				}
			}
			break;
	}
}

void GLVG_reinitCoordinates(float lat, float lon, float h){
	GLVG_latitude = (uint32_t)(lat * M_PI / 180 * 100000000);
	GLVG_longitude = (uint32_t)(lon * M_PI / 180 * 100000000);
	GLVG_height = (uint32_t)(h * 100000);

	GLVG_command = GL_REINIT;
	GLVG_coordinatesInitialized = false;
}

void GLVG_reset(){
	txBuffer[0] = 0xAA;
	txBuffer[1] = 0xAA;
	txBuffer[2] = 0x03;
	txBuffer[3] = 0x88;
	txBuffer[4] = 0x80;
	txBuffer[5] = 0x10;

	commandSize = 6;
	commandSent = false;

	__HAL_UART_DISABLE_IT(GLVG_uart, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(GLVG_uart, UART_IT_TXE);

}

static void GLVG_parsePacket(){
	if(HAL_GetTick() - lastPacketTime > 1000){
		wholePacks = validPacks;
		validPacks = 0;
		lastPacketTime = HAL_GetTick();
	}

	uint16_t wCRC = GLVG_frame[size - 2] | (GLVG_frame[size - 1] << 8);
	if (GLVG_calcCRC(GLVG_frame + 3, size - 5) == wCRC) {
		// svgStruct svgData;
		GLVG_firstPacketReceived = true;

		validPacks++;
		if (type == ORIENTATION_PACKET1) {
			memcpy(&svg_data1, &GLVG_frame[4 + GLVG_PACKET_SHIFT], size - 6);
		}
		if (type == STATUS_PACKET) {
			memcpy(&GLVG_statusWord, &GLVG_frame[4 + GLVG_PACKET_SHIFT], 4);
		}

	}

}

bool GLVG_isFirstPacketReceived(){
	return GLVG_firstPacketReceived;
}

bool GLVG_isINSValid(){
	return (svg_data1.status & 0x01) == 0 ? true : false;
}

bool GLVG_isMoving(){
	return ((svg_data1.status >> 6) & 0x01) == 0 ? true : false;
}

bool GLVG_isReady(){
	return ((svg_data1.status >> 15) & 0x01);
}

uint16_t GLVG_getFPS(){
	return wholePacks;
}

float GLVG_getRoll(){
	return svg_data1.roll;
}

float GLVG_getYaw(){
	return svg_data1.yaw;
}

float GLVG_getPitch(){
	return svg_data1.pitch;
}

float GLVG_getGx(){
	return svg_data1.gx;
}

float GLVG_getGy(){
	return svg_data1.gy;
}

float GLVG_getGz(){
	return svg_data1.gz;
}

void GLVG_UART_IRQ_Handler(UART_HandleTypeDef * huart) {
    if (huart->Instance == GLVG_uart->Instance) {
		if((__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
			  && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE) != RESET)) {
			if(rxBufferIdx >= GYRO_FRAME_SIZE) rxBufferIdx = 0;


			rxBuffer[rxBufferIdx++] =  huart->Instance->RDR;
			megacounter++;

			if(rxBufferIdx > 3 && rxBuffer[rxBufferIdx - 4] == 0xAA && rxBuffer[rxBufferIdx - 3] == 0xAA){
				rxBuffer[0] = rxBuffer[rxBufferIdx - 4];
				rxBuffer[1] = rxBuffer[rxBufferIdx - 3];
				packetLength = rxBuffer[2] = rxBuffer[rxBufferIdx - 2];
				packetType = rxBuffer[3] = rxBuffer[rxBufferIdx - 1];
				rxBufferIdx = 4;
				packetStarted = true;

			}

			if(packetStarted && rxBufferIdx == packetLength + 3){
				rxBufferIdx = 0;
				packetStarted = 0;

				size = packetLength + 3;
				type = packetType;
				memcpy(&GLVG_frame, &rxBuffer, size);
				GLVG_parsePacket();
			}

		}

		if((__HAL_UART_GET_FLAG(huart, UART_FLAG_TXE) != RESET)
						&& (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE) != RESET)){

			if(txBufferIdx < commandSize){
				huart->Instance->TDR = txBuffer[txBufferIdx++];
			} else{
				txBufferIdx = 0;

				while(__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET);
				__HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
				__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);

				memset(&txBuffer,0,TX_BUFFER_SIZE);
				commandSent = true;
				commandSentTime = HAL_GetTick();

			}
		}
		__HAL_UART_FLUSH_DRREGISTER(huart);

    }
}
