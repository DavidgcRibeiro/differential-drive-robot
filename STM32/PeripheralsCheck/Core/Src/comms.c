/*
 * comms.c
 *
 *  Created on: Apr 4, 2026
 *      Author: dylan
 */


#include "comms.h"
#include <stdlib.h>
#include <string.h>

comms_packet_t comms_packet;

static uint8_t checksum(comms_packet_t * _packet)
{
	uint8_t sum = 0;
	for (uint8_t i = 0; i < PACKET_DATA_SIZE; i++)
	{
		sum += _packet->data[i];
	}
	return sum;
}

bool isPacketValid(comms_packet_t * _packet)
{
	return (_packet->checksum == checksum(_packet));
}

void packetFromBytes(comms_packet_t * _packet, uint8_t *buff)
{
	memcpy(_packet, buff, sizeof(comms_packet_t));
}

void buildPacket(comms_packet_t * _packet, comms_packet_type packet_type, uint8_t address, uint8_t *data)
{
	_packet->start = START_BYTE;
	_packet->packet_type = packet_type;
	_packet->address = address;
	memcpy(_packet->data, data, PACKET_DATA_SIZE);
	_packet->checksum = checksum(_packet);
}
