#include "epos_can.h"
#include "stm32f4xx_hal.h"

CAN_HandleTypeDef *p_hcan1;

void EposCanInit(CAN_HandleTypeDef* can_handle_ptr) {
  p_hcan1 = can_handle_ptr;


}

