#include <dynamixel_protocol_v2.h>

// instruction for DXL Protocol
#define INST_PING               1
#define INST_READ               2
#define INST_WRITE              3
#define INST_REG_WRITE          4
#define INST_ACTION             5
#define INST_FACTORY_RESET      6
#define INST_SYNC_WRITE         131     // 0x83
#define INST_BULK_READ          146     // 0x92
// only for 2.0
#define INST_REBOOT             8
#define INST_CLEAR              16      // 0x10
#define INST_STATUS             85      // 0x55
#define INST_SYNC_READ          130     // 0x82
#define INST_BULK_WRITE         147     // 0x93

#define DXL_MAKEWORD(a, b)  ((uint16_t)( ((uint8_t )(((uint32_t)(a)) & 0xff  )) | ((uint16_t)((uint8_t )(((uint32_t)(b)) & 0xff  ))) << 8 ) )
#define DXL_MAKEDWORD(a, b) ((uint32_t)( ((uint16_t)(((uint32_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint32_t)(b)) & 0xffff))) << 16) )
#define DXL_LOBYTE(w)       ((uint8_t )( ((uint32_t)(w)) & 0xff  ))
#define DXL_LOWORD(l)       ((uint16_t)( ((uint32_t)(l)) & 0xffff))
#define DXL_HIBYTE(w)       ((uint8_t )( (((uint32_t)(w)) >> 8 ) & 0xff  ))
#define DXL_HIWORD(l)       ((uint16_t)( (((uint32_t)(l)) >> 16) & 0xffff))

#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_HEADER2             2
#define PKT_RESERVED            3
#define PKT_ID                  4
#define PKT_LENGTH_L            5
#define PKT_LENGTH_H            6
#define PKT_INSTRUCTION         7
#define PKT_ERROR               8
#define PKT_PARAMETER0          8

#define TX_PACKET_MAX_LEN  (64)
#define RX_PACKET_MAX_LEN  (64)

volatile uint8_t tx_packet[TX_PACKET_MAX_LEN];
volatile uint8_t rx_packet[RX_PACKET_MAX_LEN];
volatile static UART_HandleTypeDef* huart = NULL;
volatile static uint32_t uart_baudrate = 115200;
volatile uint32_t dynamixel_comm_err_count = 0;

void dynamixel_bind_uart(const UART_HandleTypeDef* _huart, uint32_t baudrate) {
	huart = (UART_HandleTypeDef*)_huart;
	uart_baudrate = baudrate;
}

// CRC-16-IBM (aka CRC-16-ANSI) Polynomial: x^16 + x^15 + x^2 + 1 ; check robotis manual for a look-up table implementation
uint16_t crc16(uint8_t *data, uint16_t length) {
	uint16_t crc16 = 0;
	uint16_t i, j;
	for (i=0; i<length; i++) {
		crc16 ^= ( ((uint16_t)data[i]) << 8 );
		for (j=0; j<8; j++) {
			if (crc16 & 0x8000) {
				crc16 = (crc16 << 1) ^ 0x8005;
			} else {
				crc16 <<= 1;
			}
		}
	}
	return crc16;
}

void add_stuffing(uint8_t *packet) {
	uint8_t *packet_ptr;
	uint16_t i;
	uint16_t packet_length_before_crc;
	uint16_t out_index, in_index;

	int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
	int packet_length_out = packet_length_in;

	if (packet_length_in < 8) // INSTRUCTION, ADDR_L, ADDR_H, CRC16_L, CRC16_H + FF FF FD
		return;

	packet_length_before_crc = packet_length_in - 2;
	for (i = 3; i < packet_length_before_crc; i++) {
		packet_ptr = &packet[i+PKT_INSTRUCTION-2];
		if (packet_ptr[0] == 0xFF && packet_ptr[1] == 0xFF && packet_ptr[2] == 0xFD)
			packet_length_out++;
	}

	if (packet_length_in == packet_length_out)  // no stuffing required
		return;

	out_index  = packet_length_out + 6 - 2;  // last index before crc
	in_index   = packet_length_in  + 6 - 2;  // last index before crc
	while (out_index != in_index) {
		if (packet[in_index] == 0xFD && packet[in_index-1] == 0xFF && packet[in_index-2] == 0xFF) {
			packet[out_index--] = 0xFD; // byte stuffing
			if (out_index != in_index) {
				packet[out_index--] = packet[in_index--]; // FD
				packet[out_index--] = packet[in_index--]; // FF
				packet[out_index--] = packet[in_index--]; // FF
			}
		} else {
			packet[out_index--] = packet[in_index--];
		}
	}

	packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
	packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);

	return;
}

void remove_stuffing(uint8_t *packet) {
	uint16_t i = 0;
	int index = 0;
	int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
	int packet_length_out = packet_length_in;

	index = PKT_INSTRUCTION;
	for (i = 0; i < packet_length_in - 2; i++) { // except CRC
		if (packet[i + PKT_INSTRUCTION] == 0xFD && packet[i + PKT_INSTRUCTION + 1] == 0xFD && packet[i + PKT_INSTRUCTION - 1] == 0xFF && packet[i + PKT_INSTRUCTION - 2] == 0xFF) {   // FF FF FD FD
			packet_length_out--;
			i++;
		}
		packet[index++] = packet[i + PKT_INSTRUCTION];
	}
	packet[index++] = packet[PKT_INSTRUCTION + packet_length_in - 2];
	packet[index++] = packet[PKT_INSTRUCTION + packet_length_in - 1];

	packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
	packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
}

#if 0

#define COMMAND_PING                    0x01
#define COMMAND_READ_DATA               0x02
#define COMMAND_WRITE_DATA              0x03
#define COMMAND_REG_WRITE_DATA          0x04
#define COMMAND_ACTION                  0x05
#define COMMAND_RESET                   0x06
#define COMMAND_SYNC_WRITE              0x83
#define COMMAND_SYNC_READ               0x82


void transmit_instruction_packet(const uint8_t *packet, const uint8_t size, const uint8_t status_packet_size) {
	if (!huart) {
		dynamixel_comm_err_count++;
		return;
	}
	HAL_UART_Transmit(huart, packet, size, 4);
	uint8_t rxbuffer[256] = {0};
	if (HAL_OK != HAL_UART_Receive(huart, rxbuffer, status_packet_size + 1, 4)) { // TODO check why I had +1 here
		dynamixel_comm_err_count++;
		return;
	}

	uint16_t crc1 = crc16(rxbuffer+1, status_packet_size-2);
	uint16_t crc2 = (uint16_t)(rxbuffer[1+status_packet_size - 2]) | ((uint16_t)(rxbuffer[1+status_packet_size - 1]) << 8);
	if (crc1 != crc2) {
		dynamixel_comm_err_count++;
		return;
	}
}

// this function supposes that the packet should not exceed 255 bytes
void instruction_packet_assembly(uint8_t *packet, uint8_t *instruction_packet_size, uint8_t *status_packet_size, const uint8_t id, const uint8_t instruction, const uint8_t* params, const uint8_t nparams) {
	packet[255] = 0;
	packet[0] = 0xFF;
	packet[1] = 0xFF;
	packet[2] = 0xFD;
	packet[3] = 0x00;
	packet[4] = id;

	uint16_t length = nparams + 3;
	if (length>=200) // TODO do better error handling
		while (1);
	packet[5] = length;
	packet[6] = 0x00;
	packet[7] = instruction;

	for (uint8_t i=0; i<nparams; i++) {
		packet[8 + i] = params[i];
	}
	*instruction_packet_size = length + 7; // final packet size for transmitting

	uint16_t crc = crc16(packet, *instruction_packet_size-2);
	packet[*instruction_packet_size -2] = ( crc     & 0x00FF); // little-endian
	packet[*instruction_packet_size -1] = ((crc>>8) & 0x00FF);

	if (instruction == COMMAND_WRITE_DATA)	{
		*status_packet_size = 11;
	} else if (instruction == COMMAND_READ_DATA){
		*status_packet_size = 11 + packet[10];
		if (packet[11]) { // TODO do better error handling
			while (1);
		}
	} else {
		while (1); // TODO do better error handling
	}
}

void dynamixel_set_operating_mode(uint8_t id, uint8_t mode) {
	uint8_t params[255] = {0};
  	params[0] = 11; //  address     & 0x00FF;
  	params[1] = 0;  // (address>>8) & 0x00FF;
  	params[2] = mode;

  	uint8_t packet[255] = {0};
  	uint8_t status_packet_size = 0, instruction_packet_size = 0;
  	instruction_packet_assembly(packet, &instruction_packet_size, &status_packet_size, id, COMMAND_WRITE_DATA, params, 3);
  	transmit_instruction_packet(packet, instruction_packet_size, status_packet_size);
}

void dynamixel_set_velocity(uint8_t id, int32_t velocity) {
	uint8_t params[255] = {0};
  	params[0] = 104; //  address     & 0x00FF;
  	params[1] = 0;   // (address>>8) & 0x00FF;
  	params[2] = ((uint8_t *)(&velocity))[0];
  	params[3] = ((uint8_t *)(&velocity))[1];
  	params[4] = ((uint8_t *)(&velocity))[2];
  	params[5] = ((uint8_t *)(&velocity))[3];

  	uint8_t packet[255] = {0};
  	uint8_t status_packet_size = 0, instruction_packet_size = 0;
  	instruction_packet_assembly(packet, &instruction_packet_size, &status_packet_size, id, COMMAND_WRITE_DATA, params, 6);
  	transmit_instruction_packet(packet, instruction_packet_size, status_packet_size);
}

void dynamixel_torque_on_off(uint8_t id, uint8_t isenabled) {
	uint8_t params[255] = {0};
  	params[0] = 64;
  	params[1] = 0;
  	params[2] = isenabled;
  	uint8_t packet[255] = {0};
  	uint8_t status_packet_size = 0, instruction_packet_size = 0;
  	instruction_packet_assembly(packet, &instruction_packet_size, &status_packet_size, id, COMMAND_WRITE_DATA, params, 3);
  	transmit_instruction_packet(packet, instruction_packet_size, status_packet_size);
}
#endif


void transmit_packet() {
  uint16_t total_packet_length = 0;
  uint16_t written_packet_length = 0;
  uint16_t crc;

  /*
  if (g_is_using[port_num])
  {
    packetData[port_num].communication_result = COMM_PORT_BUSY;
    return;
  }
  g_is_using[port_num] = True;
  */

  // byte stuffing for header
  add_stuffing(tx_packet);

  // check max packet length
  total_packet_length = DXL_MAKEWORD(tx_packet[PKT_LENGTH_L], tx_packet[PKT_LENGTH_H]) + 7;   // 7: HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H
  if (total_packet_length > TX_PACKET_MAX_LEN) {
//    g_is_using[port_num] = False;
//    packetData[port_num].communication_result = COMM_TX_ERROR;
    return;
  }

  // add packet header
  tx_packet[PKT_HEADER0]  = 0xFF;
  tx_packet[PKT_HEADER1]  = 0xFF;
  tx_packet[PKT_HEADER2]  = 0xFD;
  tx_packet[PKT_RESERVED] = 0x00;

  // add CRC16
  crc = crc16(tx_packet, total_packet_length - 2);  // -2 for the CRC itself
  tx_packet[total_packet_length - 2] = DXL_LOBYTE(crc);
  tx_packet[total_packet_length - 1] = DXL_HIBYTE(crc);

  memset(rx_packet, 0, RX_PACKET_MAX_LEN);
  uint32_t tx_timeout = (1000L*9L*(uint32_t)(total_packet_length))/uart_baudrate + 1;
  HAL_StatusTypeDef txres = HAL_UART_Transmit(huart, tx_packet, total_packet_length, tx_timeout);
  HAL_StatusTypeDef rxres = HAL_UART_Receive(huart, rx_packet, 12 + 1, 4);

  tx_packet[63] = txres;
  rx_packet[63] = rxres;

  /*
  // tx packet
  clearPort(port_num);
  written_packet_length = writePort(port_num, packetData[port_num].tx_packet, total_packet_length);
  if (total_packet_length != written_packet_length)
  {
    g_is_using[port_num] = False;
    packetData[port_num].communication_result = COMM_TX_FAIL;
    return;
  }

  packetData[port_num].communication_result = COMM_SUCCESS;
  */
}

void dynamixel_write(uint8_t id, uint16_t address, uint8_t *data, uint16_t length) {
	tx_packet[PKT_ID] = id;
	tx_packet[PKT_LENGTH_L] = DXL_LOBYTE(length + 5);
	tx_packet[PKT_LENGTH_H] = DXL_HIBYTE(length + 5);
	tx_packet[PKT_INSTRUCTION] = INST_WRITE;
	tx_packet[PKT_PARAMETER0 + 0] = DXL_LOBYTE(address);
	tx_packet[PKT_PARAMETER0 + 1] = DXL_HIBYTE(address);

	for (uint16_t i=0; i<length; i++)
		tx_packet[PKT_PARAMETER0 + 2 + i] = data[i];
	transmit_packet();
}

void dynamixel_read(uint8_t id, uint16_t address, uint16_t length) {
	tx_packet[PKT_ID] = id;
	tx_packet[PKT_LENGTH_L] = 7;
	tx_packet[PKT_LENGTH_H] = 0;
	tx_packet[PKT_INSTRUCTION] = INST_READ;
	tx_packet[PKT_PARAMETER0 + 0] = DXL_LOBYTE(address);
	tx_packet[PKT_PARAMETER0 + 1] = DXL_HIBYTE(address);
	tx_packet[PKT_PARAMETER0 + 2] = DXL_LOBYTE(length);
	tx_packet[PKT_PARAMETER0 + 3] = DXL_HIBYTE(length);
	transmit_packet();
}


void dynamixel_set_operating_mode(uint8_t id, uint8_t mode) {
	dynamixel_write(id, 11, &mode, 1);
}

void dynamixel_torque_on_off(uint8_t id, uint8_t isenabled) {
	dynamixel_write(id, 64, &isenabled, 1);
}

void dynamixel_set_velocity(uint8_t id, int32_t velocity) {
	dynamixel_write(id, 104, (uint8_t *)&velocity, 4);
}

void dynamixel_set_current(uint8_t id, int16_t current) {
	dynamixel_write(id, 102, (uint8_t *)&current, 2);
}
