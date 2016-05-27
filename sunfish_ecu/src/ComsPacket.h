#ifndef COMS_PACKET_H
#define COMS_PACKET_H

#include <stdint.h>

class ComsPacket
{
public:
  ComsPacket();
  
  static int DATA_SIZE;
  static int PAYLOAD_SIZE;

  uint16_t getModuleId();
  uint16_t getPacketId();
  uint32_t getSeqNumber();

  void setModuleId(uint16_t id);
  void setPacketId(uint16_t id);
  void setSeqNumber(uint32_t seq);
 
  const uint8_t * getPayload();
  const uint8_t * getData();
  
  void setPayload(const uint8_t * payload);
  void setData(const uint8_t * data);
  
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

