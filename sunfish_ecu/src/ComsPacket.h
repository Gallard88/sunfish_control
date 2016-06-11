#ifndef COMS_PACKET_H
#define COMS_PACKET_H

#include <stdint.h>

class ComsPacket
{
public:
  ComsPacket();

  static int DATA_SIZE;
  static int PAYLOAD_SIZE;

  uint16_t getModuleId()  const;
  uint16_t getPacketId()  const;
  uint32_t getSeqNumber() const;

  void setModuleId(uint16_t id);
  void setPacketId(uint16_t id);
  void setSeqNumber(uint32_t seq);

  const uint8_t * getPayload() const;
  const uint8_t * getData()    const;

  void setPayload(const uint8_t * payload);
  void setData(const uint8_t * data);

  static uint16_t readValue_16b(const uint8_t *ptr);
  static uint32_t readValue_32b(const uint8_t *ptr);

private:
  uint8_t buffer[512];
  static int HEADER_SIZE;
  static int PAYLOAD_OFFSET;

/*
 * uint16_t ModuleId
 * uint16_t PktId
 * uint32_t seqNumber
 * uint8_t payload[]
 */

};



#endif

