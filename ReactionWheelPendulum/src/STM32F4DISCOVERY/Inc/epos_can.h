#ifndef EPOS_CAN_H
#define EPOS_CAN_H

#include "stm32f4xx_hal.h"

#define RPDO1 0x1400
#define RPDO2 0x1401
#define RPDO3 0x1402
#define RPDO4 0x1403
#define RPDOMAP1 0x1600
#define RPDOMAP2 0x1601
#define RPDOMAP3 0x1602
#define RPDOMAP4 0x1603
#define TPDO1 0x1800
#define TPDO2 0x1801
#define TPDO3 0x1802
#define TPDO4 0x1803
#define TPDOMAP1 0x1A00
#define TPDOMAP2 0x1A01
#define TPDOMAP3 0x1A02
#define TPDOMAP4 0x1A03
#define CONTROLWORD 0x6040
#define CONTROLWORD 0x6040
#define POSITION_ACTUAL 0x6064
#define VELOCITY_ACTUAL 0x2028
#define VELOCITY_ACTUAL2 0x606C
#define CURRENT_ACTUAL 0x2027
#define CURRENT 0x2030
#define VELOCITY 0x206B
#define POSITION 0x2062
#define ERROR_HISTORY 0x1003

#define CURRENT_LENGTH 16
#define VELOCITY_LENGTH 32

typedef struct {
  uint32_t n_pdo;
  uint8_t n_mapped;
  uint32_t mapped[8];
} pdoMapping;

typedef struct {
  uint8_t nodeID;
  uint16_t rxCobID;
  uint16_t txCobID;
  pdoMapping rPm;
  pdoMapping tPm;
} eposStatus;


void mapObject(pdoMapping *pm, uint16_t reg, uint8_t subIndex, uint8_t length);
void enableEpos();
void eposReset();

void enterPreOperational();
void enterOperational();
void setCurrent(int current);
void setSDOCurrent(int current);
void enableEposTxPDO();
void enableEposRxPDO();
#endif

