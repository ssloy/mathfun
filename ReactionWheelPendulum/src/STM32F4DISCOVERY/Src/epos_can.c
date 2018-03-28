#include "epos_can.h"
#include "stm32f4xx_hal.h"

CAN_HandleTypeDef *p_hcan1;
CAN_FilterTypeDef sFilterConfig;

void EposCanInit(CAN_HandleTypeDef* can_handle_ptr) {
  p_hcan1 = can_handle_ptr;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(p_hcan1, &sFilterConfig) != HAL_OK)
    while (1);

  if (HAL_CAN_Start(p_hcan1) != HAL_OK)
    while (1);

  if (HAL_CAN_ActivateNotification(p_hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    while (1);
}

