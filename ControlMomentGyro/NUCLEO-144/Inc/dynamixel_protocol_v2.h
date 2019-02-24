#ifndef DYNAMIXEL_PROTOCOL_V2_H_
#define DYNAMIXEL_PROTOCOL_V2_H_

#include "stm32f7xx_hal.h"
#include <string.h>

#define COMMAND_PING                    0x01
#define COMMAND_READ_DATA               0x02
#define COMMAND_WRITE_DATA              0x03
#define COMMAND_REG_WRITE_DATA          0x04
#define COMMAND_ACTION                  0x05
#define COMMAND_RESET                   0x06
#define COMMAND_SYNC_WRITE              0x83
#define COMMAND_SYNC_READ               0x82

void torque_on_off(UART_HandleTypeDef* huart, uint8_t id, uint8_t isenabled);
void set_velocity(UART_HandleTypeDef* huart, uint8_t id, int32_t velocity);
void set_operating_mode(UART_HandleTypeDef* huart, uint8_t id, uint8_t mode);

/*
//#########################################################################
//################ define - Dynamixel Hex code table ######################
// EEPROM AREA
#define EEPROM_MODEL_NUMBER_L           0x00
#define EEPROM_MODEL_NUMBER_H           0x01
#define EEPROM_VERSION                  0x02
#define EEPROM_ID                       0x03
#define EEPROM_BAUD_RATE                0x04
#define EEPROM_RETURN_DELAY_TIME        0x05
#define EEPROM_CW_ANGLE_LIMIT_L         0x06
#define EEPROM_CW_ANGLE_LIMIT_H         0x07
#define EEPROM_CCW_ANGLE_LIMIT_L        0x08
#define EEPROM_CCW_ANGLE_LIMIT_H        0x09
#define EEPROM_LIMIT_TEMPERATURE        0x0B
#define EEPROM_LOW_LIMIT_VOLTAGE        0x0C
#define EEPROM_HIGN_LIMIT_VOLTAGE       0x0D
#define EEPROM_MAX_TORQUE_L             0x0E
#define EEPROM_MAX_TORQUE_H             0x0F
#define EEPROM_RETURN_LEVEL             0x10
#define EEPROM_ALARM_LED                0x11
#define EEPROM_ALARM_SHUTDOWN           0x12
// RAM AREA
#define RAM_TORQUE_ENABLE               0x18
#define RAM_LED                         0x19
#define RAM_PROPORTIONAL_GAIN           0x1C
#define RAM_INTERGRAL_GAIN              0x1B
#define RAM_DERIVATIVE_GAIN             0x1A
#define RAM_GOAL_POSITION_L             0x1E
#define RAM_GOAL_POSITION_H             0x1F
#define RAM_GOAL_SPEED_L                0x20
#define RAM_GOAL_SPEED_H                0x21
#define RAM_TORQUE_LIMIT_L              0x22
#define RAM_TORQUE_LIMIT_H              0x23
#define RAM_PRESENT_POSITION_L          0x24
#define RAM_PRESENT_POSITION_H          0x25
#define RAM_PRESENT_SPEED_L             0x26
#define RAM_PRESENT_SPEED_H             0x27
#define RAM_PRESENT_LOAD_L              0x28
#define RAM_PRESENT_LOAD_H              0x29
#define RAM_PRESENT_VOLTAGE             0x2A
#define RAM_PRESENT_TEMPERATURE         0x2B
#define RAM_REGISTER                    0x2C
#define RAM_MOVING                      0x2E
#define RAM_LOCK                        0x2F
#define RAM_PUNCH_L                     0x30
#define RAM_PUNCH_H                     0x31

//#########################################################################
//################ Instruction commands Set ###############################

//#########################################################################
//################ Instruction packet lengths #############################
// Packet length is number of parameters (N) + 2
#define READ_ONE_BYTE_LENGTH            0x01
#define READ_TWO_BYTE_LENGTH            0x02
#define RESET_LENGTH                    0x02
#define PING_LENGTH                     0x02
#define ACTION_LENGTH                   0x02
#define SET_ID_LENGTH                   0x04
#define SET_BD_LENGTH                   0x04
#define SET_RETURN_LEVEL_LENGTH         0x04
#define READ_TEMP_LENGTH                0x04
#define READ_POS_LENGTH                 0x04
#define READ_LOAD_LENGTH                0x04
#define READ_SPEED_LENGTH               0x04
#define READ_VOLT_LENGTH                0x04
#define READ_REGISTER_LENGTH            0x04
#define READ_MOVING_LENGTH              0x04
#define READ_LOCK_LENGTH                0x04
#define LED_LENGTH                      0x04
#define SET_HOLDING_TORQUE_LENGTH       0x04
#define SET_MAX_TORQUE_LENGTH           0x05
#define SET_ALARM_LENGTH                0x04
#define READ_LOAD_LENGTH                0x04
#define SET_RETURN_LENGTH               0x04
#define WHEEL_LENGTH                    0x05
#define SERVO_GOAL_LENGTH               0x07
#define SET_MODE_LENGTH                 0x07
#define SET_PUNCH_LENGTH                0x04
#define SET_PID_LENGTH                  0x06
#define SET_TEMP_LENGTH                 0x04
#define SET_VOLT_LENGTH                 0x05
#define SYNC_LOAD_LENGTH                0x0D
#define SYNC_DATA_LENGTH                0x02


//#########################################################################
//############################ Specials ###################################
#define PORT0                           0x00
#define PORT1                           0x01
#define PORT2                           0x02
#define PORT3                           0x03

#define OFF                             0x00
#define ON                              0x01

#define SERVO                           0x01
#define WHEEL                           0x00

#define LEFT                            0x00
#define RIGHT                           0x01

#define NONE                            0x00
#define READ                            0x01
#define ALL                             0x02

#define BROADCAST_ID                    0xFE

#define HEADER                          0xFF
#define HEADER2                         0xFD
#define RESERVED                        0x00

#define STATUS_PACKET_TIMEOUT           50      // in millis()
#define STATUS_FRAME_BUFFER             5
//##########################################################################

#define NUMBER_OF_SERVOS 2

uint8_t txCompleteFlag, rxCompleteFlag;

typedef struct {
	uint8_t
		ID,
		FirmwareVersion,
		BaudRate,
		ReturnDelayTime,
		TemperatureLimit,
		MinVoltageLimit,
		MaxVoltageLimit,
		StatusReturnLevel,
		AlarmLED,
		Shutdown,
		ResolutionDivider,
		TorqueEnable,
		LED,
		D_Gain,
		I_Gain,
		P_Gain,
		PresentVoltage,
		PresentTemperature,
		Registered,
		Moving,
		Lock,
		TorqueCtrlModeEnable,
		GoalAcceleration;

	uint16_t
		ModelNumber,
		CWAngleLimit,
		CCWAngleLimit,
		MaxTorque,
		MultiTurnOffset,
		GoalPosition,
		MovingSpeed,
		TorqueLimit,
		PresentPosition,
		PresentSpeed,
		PresentLoad,
		Punch,
		RealtimeTick,
		Current,
		GoalTorque;
}Servo;

typedef struct{
	uint8_t data[100];
} dataToSyncWrite;

Servo servoArray[NUMBER_OF_SERVOS];
dataToSyncWrite dataForSyncWrite[NUMBER_OF_SERVOS];

uint8_t instructionPacket[300];
uint8_t receiveBuffer[300];
uint8_t commRxBuffer[300];
uint8_t commTxBuffer[300];
uint8_t parameters[300];
uint8_t numberOfParameters;
uint8_t legthOfTxPacket;
UART_HandleTypeDef* dynamixelUART;

void bindDynamixelUART(UART_HandleTypeDef*);
void initServo(Servo*,uint8_t);
void transmitInstructionPacket(void);
void instructionPacketAssebly(uint8_t,uint8_t,uint8_t*,uint8_t);
void syncWrite(uint8_t, dataToSyncWrite* , uint16_t, uint16_t);
void syncRead(Servo*, uint8_t , uint16_t , uint16_t );
void readData(Servo*,uint16_t, uint16_t);
void setProfileVelocity(int32_t, Servo*);
void writeData();
unsigned short update_crc(uint16_t, uint8_t*, uint16_t);
void rxCompleteCalback();
void setProfileVelocity(int32_t, Servo*);

void setPosition(uint32_t,Servo*);
void setID(uint8_t,Servo*);
void setBaudrate(uint8_t);
void enableToque(uint8_t, Servo*);
void setOperatingMode(uint8_t, Servo*);
void setVelocity(int32_t, Servo*);
*/

#endif /* DYNAMIXEL_PROTOCOL_V2_H_ */
