#ifndef GL_SVG_H
#define GL_SVG_H

#include "stm32f7xx_hal.h"

typedef struct svgStruct{
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

}__attribute__((packed)) svgStruct;

svgStruct svgData;

void setSVGUART(UART_HandleTypeDef * handler);
void add_CRC(uint16_t *fcs, uint8_t c);
void processSVG();
float getSVGGx();
float getSVGGy();
float getSVGGz();
float getSVGYaw();
float getSVGRoll();
float getSVGPitch();
void dropDataFlag();

uint32_t getSVGStatus();
uint8_t isReady();
uint8_t isNewFrame();
uint8_t isValidPacket(uint8_t pack_id);

void resetNewFrame();
void resetError();
void initSVG();
float getDistanceFromLatLon(float lat1,float lon1,float lat2,float lon2);
#endif
