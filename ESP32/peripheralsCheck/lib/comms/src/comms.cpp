/*
 * comms.c
 *
 *  Created on: Apr 4, 2026
 *      Author: dylan
 */

#include "comms.h"
#include <stdlib.h>
#include <string.h>

uint8_t PacketHandler::checksum()
{
	uint8_t sum = 0;
	for (uint8_t i = 0; i < PACKET_DATA_SIZE; i++)
	{
		sum += packet.data[i];
	}
	return sum;
}

bool PacketHandler::isPacketValid()
{
	return (packet.checksum == checksum() && packet.start == START_BYTE);
}

void PacketHandler::parseBytes(uint8_t *buff)
{
	memcpy((uint8_t *)&packet, buff, sizeof(packet));
}

void PacketHandler::buildPacket(comms_packet_type packet_type, uint8_t address, uint8_t *data)
{
	packet.start = START_BYTE;
	packet.packet_type = packet_type;
	packet.address = address;
	memcpy(packet.data, data, PACKET_DATA_SIZE);
	packet.checksum = checksum();
}

void PacketHandler::print()
{
	uint32_t value;
	memcpy(&value, packet.data, PACKET_DATA_SIZE);
	Serial.println("PACKET:");
	Serial.printf("Valid: %d\n", isPacketValid());
	Serial.printf("SYNC1: 0x%02X\n", packet.start);
	Serial.printf("TYPE: 0x%02X\n", packet.packet_type);
	Serial.printf("ADDR: 0x%02X\n", packet.address);
	Serial.printf("CHKSUM: 0x%02X\n", packet.checksum);
	Serial.printf("Value: %d\r\n", value);
	Serial.println();
}