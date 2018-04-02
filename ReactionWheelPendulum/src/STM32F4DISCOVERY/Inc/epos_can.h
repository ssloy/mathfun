#ifndef EPOS_CAN_H
#define EPOS_CAN_H

#include "stm32f4xx_hal.h"

void enableEpos(uint8_t nodeID);
void eposReset();
void enterOperational();
void setCurrent(uint16_t rxCobID, int current);

#endif
