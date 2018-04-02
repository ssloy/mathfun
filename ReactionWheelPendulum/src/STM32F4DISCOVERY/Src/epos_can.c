#include "epos_can.h"
#include "stm32f4xx_hal.h"

extern CAN_HandleTypeDef hcan1;

void canWrite(uint8_t nodeID, uint16_t reg, uint8_t subIndex, int value) {
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8] = {0x22, reg&0xFF, reg>>8, subIndex, value&0xFF, (value>>8)&0xFF, (value>>16)&0xFF,  (value>>24)&0xFF};
  uint32_t TxMailbox;

  TxHeader.StdId = 0x600 + nodeID;
  TxHeader.ExtId = 0;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
  while (3!=HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));
}

void enableEpos(uint8_t nodeID) {
  canWrite(nodeID, 0x6040, 0x00, 0x0006);
  HAL_Delay(100);
  canWrite(nodeID, 0x6040, 0x00, 0x000F);
}

void eposReset() {
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[2] = { 0x81, 0x00 };
  uint32_t TxMailbox;

  TxHeader.StdId = 0x00;
  TxHeader.ExtId = 0;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 2;
  TxHeader.TransmitGlobalTime = DISABLE;
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
  while (3!=HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));
}

void enterOperational() {
  uint8_t TxData[2] = { 0x01, 0x00 };
  uint32_t TxMailbox;
  CAN_TxHeaderTypeDef TxHeader;

  TxHeader.StdId = 0x00;
  TxHeader.ExtId = 0;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 2;
  TxHeader.TransmitGlobalTime = DISABLE;
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
  while (3 != HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)) ;
}

void setCurrent(uint16_t rxCobID, int current) {
  uint8_t TxData[4] = { 0 };
  uint32_t TxMailbox;
  CAN_TxHeaderTypeDef TxHeader;

  TxHeader.StdId = rxCobID;
  TxHeader.ExtId = 0;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 4;
  TxHeader.TransmitGlobalTime = DISABLE;

  if (current>=0) {
    TxData[0] = current & 0xFF;
    TxData[1] = current >> 8;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
  } else {
    int c = 0xFFFF + current + 1;
    TxData[0] = c & 0xFF;
    TxData[1] = c >> 8;
    TxData[2] = 0xFF;
    TxData[3] = 0xFF;
  }

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
  while (3!=HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));
}
