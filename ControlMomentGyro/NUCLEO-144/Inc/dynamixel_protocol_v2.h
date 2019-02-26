#ifndef DYNAMIXEL_PROTOCOL_V2_H_
#define DYNAMIXEL_PROTOCOL_V2_H_

#include "stm32f7xx_hal.h"
#include <string.h>

void dynamixel_bind_uart(const UART_HandleTypeDef* _huart, uint32_t baudrate);
void dynamixel_torque_on_off(uint8_t id, uint8_t isenabled);
void dynamixel_set_velocity(uint8_t id, int32_t velocity);
void dynamixel_set_current(uint8_t id, int16_t current);
void dynamixel_set_operating_mode(uint8_t id, uint8_t mode);

bool dynamixel_read_current_velocity_position(uint8_t id, int16_t *current, int32_t *velocity, int32_t *position);

//void dynamixel_read(uint8_t id, uint16_t address, uint16_t length);


#endif /* DYNAMIXEL_PROTOCOL_V2_H_ */
