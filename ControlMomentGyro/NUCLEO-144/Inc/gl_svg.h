#ifndef GL_SVG_H
#define GL_SVG_H

#include "stm32f7xx_hal.h"
#include "stdbool.h"

#define GLVG_485 1

#define FRAME_LENGTH 200
#define BINS_FRAME_LENGTH 58
#define GYRO_FRAME_SIZE 120

#define ORIENTATION_PACKET1 0x70
#define ORIENTATION_PACKET2 0xAF
#define STATUS_PACKET 0xE6

#define BINS_PACKET 0x70
#define EXTRA_PACKET 0x72
#define NUMBER_OF_AV_PACKS 2

typedef struct orientationPacket1Struct{
  uint32_t status;

  float ax;
  float ay;
  float az;

  float gx;
  float gy;
  float gz;

  float roll;
  float yaw;
  float pitch;

  int32_t latitude;
  int32_t longtitude;
  float height;

}__attribute__((packed)) orientationPacket1Struct;

typedef struct orientationPacket2Struct{
  float gx;
  float gy;
  float gz;
  
  float north_course;
  float roll;
  float pitch;
  float stab_course;
}__attribute__((packed)) orientationPacket2Struct;

extern orientationPacket1Struct svg_data1;
extern orientationPacket2Struct svg_data2;

void GLVG_init(UART_HandleTypeDef * uart);
void GLVG_reinitCoordinates(float lat, float lon, float h);

bool GLVG_isINSValid();
bool GLVG_isMoving();
bool GLVG_isReady();
bool GLVG_isFirstPacketReceived();

uint16_t GLVG_getFPS();
float GLVG_getRoll();
float GLVG_getYaw();
float GLVG_getPitch();
float GLVG_getGx();
float GLVG_getGy();
float GLVG_getGz();

#endif
