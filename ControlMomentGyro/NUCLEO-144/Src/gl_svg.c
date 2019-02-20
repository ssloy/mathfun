#include "gl_svg.h"

#define FRAME_LENGTH 200
#define BINS_FRAME_LENGTH 58
#define EXTRA_FRAME_LENGTH 54
#define PRINT_BUFFER_SIZE 100
#define COMMANDS_BUFFER_LENGTH 15
#define GYRO_FRAME_SIZE 500

#define BINS_PACKET 0x70
#define EXTRA_PACKET 0x72
#define NUMBER_OF_AV_PACKS 2

#define GRAVITY_ACCELERATION 9.8065

uint8_t availablePackets[NUMBER_OF_AV_PACKS] = {BINS_PACKET, EXTRA_PACKET};

char PrintBuffer[PRINT_BUFFER_SIZE];
uint8_t frame[FRAME_LENGTH], length, buffer = 0;
uint8_t commands_buffer[COMMANDS_BUFFER_LENGTH];
uint8_t gyro_rx_buffer[GYRO_FRAME_SIZE];

uint8_t data_ready = 0;

float ax, ay, az;
float gx, gy, gz;
float yaw, pitch, roll;
float latitude1, latitude2, longtitude1, longtitude2;

uint32_t status;
uint8_t is_new_frame = 0, is_ready = 0, is_processing = 0;
uint8_t packet_started;
uint8_t fault_state = 0;

uint8_t gyroRxIdx = 0, gyroTxIdx = 0;
uint8_t packet_counter = 0;
uint8_t packet_type;
uint16_t packet_length;
uint16_t whole_n_packs = 0;
uint16_t valid_packs = 0;

uint32_t last_time;
UART_HandleTypeDef* gyro_uart;

uint16_t ccitt_crc16_table[256] = {
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac,
0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695,
0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d,
0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802,
0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a,
0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33,
0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60,
0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68,
0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51,
0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59,
0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e,
0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046,
0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f,
0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277,
0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424,
0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d,
0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615,
0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a,
0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882,
0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb,
0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3,
0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8,
0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9,
0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1,
0x1ef0
};

void add_CRC(uint16_t * fcs, uint8_t c)
{
	*fcs = ccitt_crc16_table[(*fcs >> 8 ^ c) & 0xff] ^ (*fcs << 8);
}

void setSVGUART(UART_HandleTypeDef* handler){
	gyro_uart = handler;
	__HAL_UART_ENABLE_IT(gyro_uart, UART_IT_RXNE);
}

void processSVG() {
	whole_n_packs++;
	uint16_t wData;
	uint16_t wCrc = 0x00;
	uint8_t i, j;
	for (i = 3; i < packet_length - 2; i++) {
		wCrc ^= frame[i] << 8;
		for (j = 0; j < 8; j++)
			wCrc = wCrc & 0x8000 ? (wCrc << 1) ^ 0x1021 : wCrc << 1;
	}
	wCrc = wCrc & 0xffff;
	if (wCrc
			!= (frame[packet_length - 2]) + ((frame[packet_length - 1]) << 8)) {

	} else {
		valid_packs++;
		if (packet_type == BINS_PACKET) {
			memcpy(&(svgData), &frame[8], 52);

			if (svgData.status & 0x01) {
				// gyroscope fault state handler
				fault_state = 1;
			} else {
				fault_state = 0;
			}

		}

		if (packet_type == EXTRA_PACKET) {
			float dt = (float) (HAL_GetTick() - last_time) / 1000;
			last_time = HAL_GetTick();

		}

	}

	is_ready = 0;
}



uint8_t svgIRQHandler(UART_HandleTypeDef* huart) {
	if (gyro_uart->Instance == huart->Instance) {
		if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
				&& (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE) != RESET)) {
			if (gyroRxIdx < GYRO_FRAME_SIZE) {
				gyro_rx_buffer[gyroRxIdx] = huart->Instance->RDR;
				if (gyroRxIdx > 2
					 	&&	gyro_rx_buffer[gyroRxIdx-3] == 0xAA
						&& gyro_rx_buffer[gyroRxIdx-2] == 0xAA

						&& !packet_started){
					packet_type = gyro_rx_buffer[gyroRxIdx ];
					packet_length = gyro_rx_buffer[gyroRxIdx-1]+3 ;
					packet_started = 1;
					packet_counter = 3;
				}
				gyroRxIdx++;

				if(packet_started){
					packet_counter++;
				}

				if((packet_counter == packet_length) && packet_started ){
					//if(!is_ready){
						memset(&frame,0,FRAME_LENGTH);
						memcpy(&frame, &gyro_rx_buffer[gyroRxIdx - packet_length], packet_length);
						memset(&gyro_rx_buffer,0,GYRO_FRAME_SIZE);
						processSVG();
						is_ready = 1;

					//}
					packet_started = 0;
					packet_counter = 0;
					gyroRxIdx = 0;
				}


			}else{
				gyroRxIdx = 0;
			}
			__HAL_UART_FLUSH_DRREGISTER(huart);
			return 1;
		}

		if((__HAL_UART_GET_FLAG(huart, UART_FLAG_TXE) != RESET) &&(__HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE) != RESET)) {
			huart->Instance->RDR = commands_buffer[gyroTxIdx++];

			if(gyroTxIdx == strlen(commands_buffer)){
				gyroTxIdx = 0;
				__HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
			}


			__HAL_UART_FLUSH_DRREGISTER(huart);
			return 1;
		}

	}
	__HAL_UART_FLUSH_DRREGISTER(huart);
	return 0;
}

uint8_t isValidPacket(uint8_t pack_id){
	uint8_t i;
	for(i = 0; i < NUMBER_OF_AV_PACKS; i++)
		if(availablePackets[i] == pack_id) return 1;

	return 0;
}

float getDistanceFromLatLon(float lat1,float lon1,float lat2,float lon2) {
  int R = 6371;
  float dLat = (lat2-lat1);
  float dLon = (lon2-lon1);
  float a =
    sin(dLat/2) * sin(dLat/2) +
    cos((lat1)) * cos((lat2)) *
    sin(dLon/2) * sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  float d = R * c;
  return d*1000;
}


void resetError(){
	memset(commands_buffer, 0, COMMANDS_BUFFER_LENGTH);
	commands_buffer[0] = 0xAA;
	commands_buffer[1] = 0xAA;
	commands_buffer[2] = 0x07;
	commands_buffer[3] = 0x88;
	commands_buffer[4] = 0xE1;
	commands_buffer[5] = 0xCD;
	commands_buffer[6] = 0x28;
	commands_buffer[7] = 0x64;
	commands_buffer[8] = 0x68;
	commands_buffer[9] = 0x15;

	gyroTxIdx = 0;
	__HAL_UART_ENABLE_IT(gyro_uart, UART_IT_TXE);
}

float getSVGGx(){
	return svgData.gx;
}

float getSVGGy(){
	return svgData.gy;
}

float getSVGGz(){
	return svgData.gz;
}

float getSVGYaw(){
	return svgData.yaw;
}

float getSVGRoll(){
	return svgData.roll;
}

float getSVGPitch(){
	return svgData.pitch;
}

uint8_t isInFaultState(){
	return fault_state;
}

uint8_t isReady(){
	return is_ready;
}

void dropDataFlag(){
	data_ready = 0;
}

uint32_t getSVGStatus(){
	return status;
}
