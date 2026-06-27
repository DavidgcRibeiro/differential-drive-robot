/*
 * comms.h
 *
 *  Created on: Apr 4, 2026
 *      Author: dylan
 */

#ifndef INC_COMMS_H_
#define INC_COMMS_H_

#include "Arduino.h"

#define PACKET_DATA_SIZE 4

#define START_BYTE 0xFE

typedef struct _comms_packet_t {
	uint16_t start;
	uint8_t packet_type;
	uint8_t address;
	uint8_t data[PACKET_DATA_SIZE];
	uint8_t checksum;
} comms_packet_t;

typedef enum _comms_packet_type {
	COMMS_TYPE_ECHO = 0,
	COMMS_TYPE_WRITE,
	COMMS_TYPE_READ,
	COMMS_TYPE_ACK = 254,
	COMMS_TYPE_ERR = 255,
} comms_packet_type;

class PacketHandler
{
private:
	uint8_t checksum();

public:
	comms_packet_t packet;
	bool isPacketValid();
	void parseBytes(uint8_t *buff);
	void buildPacket(comms_packet_type packet_type, uint8_t address, uint8_t *data);
	void print();
};

#endif /* INC_COMMS_H_ */
