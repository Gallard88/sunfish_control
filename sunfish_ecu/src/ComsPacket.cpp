#include <cstdio>
#include "ros/ros.h"
#include "ComsPacket.h"

ComsPacket::ComsPacket()
{
}

int ComsPacket::HEADER_SIZE = 8;
int ComsPacket::PAYLOAD_OFFSET = ComsPacket::HEADER_SIZE;

int ComsPacket::DATA_SIZE = 512;
int ComsPacket::PAYLOAD_SIZE = ComsPacket::DATA_SIZE-ComsPacket::HEADER_SIZE;

void ComsPacket::setModuleId(uint16_t id)
{
  buffer[0] = id >> 8;
  buffer[1] = id;
}

uint16_t ComsPacket::getModuleId()
{
  uint16_t id;

  id = (buffer[0] * 256) | buffer[1];
  return id;
}

void ComsPacket::setPacketId(uint16_t id)
{
  buffer[2] = id >> 8;
  buffer[3] = id;
}

uint16_t ComsPacket::getPacketId()
{
  uint16_t id;

  id = (buffer[2] * 256) | buffer[3];
  return id;
}

void ComsPacket::setSeqNumber(uint32_t seq)
{
  buffer[4] = seq >> 24;
  buffer[5] = seq >> 16;
  buffer[6] = seq >> 8;
  buffer[7] = seq ;
}

uint32_t ComsPacket::getSeqNumber()
{
  uint32_t id = 0;

  id = buffer[4];
  id = (id * 256) | buffer[5];
  id = (id * 256) | buffer[6];
  id = (id * 256) | buffer[7];
  return id;
}

void ComsPacket::setPayload(const uint8_t * payload)
{
  ROS_ASSERT(payload != NULL);
  memcpy(buffer, payload, PAYLOAD_SIZE);
}

const uint8_t * ComsPacket::getPayload()
{
  return buffer;
}

const uint8_t * ComsPacket::getData()
{
  return buffer + ComsPacket::PAYLOAD_OFFSET;
}

void ComsPacket::setData(const uint8_t * data)
{
  ROS_ASSERT(data != NULL);
  memcpy(buffer+ComsPacket::PAYLOAD_OFFSET, data, DATA_SIZE);
}

