
#include <gtest/gtest.h>
#include "../src/ComsPacket.h"

/* ------------------------------------------------------------ */
TEST(t_ComsPacket, contructor_default)
{
  ComsPacket cp;

  for ( int i = 0; i < ComsPacket::BUFF_SIZE; i ++ ) {
    ASSERT_EQ(0, cp.buffer[i]);
  }
}

TEST(t_ComsPacket, contructor_default_moduleId)
{
  ComsPacket cp;
  ASSERT_EQ(0x0000, cp.getModuleId());
}

TEST(t_ComsPacket, contructor_default_packetId)
{
  ComsPacket cp;
  ASSERT_EQ(0x0000, cp.getPacketId());
}

TEST(t_ComsPacket, contructor_default_seqNumber)
{
  ComsPacket cp;
  ASSERT_EQ(0x0000, cp.getSeqNumber());
}

/* ------------------------------------------------------------ */
TEST(t_ComsPacket, getModuleID_ByteOrder)
{ 
  ComsPacket cp;
  uint8_t buffer[512];
  
  buffer[0] = 0x55;
  buffer[1] = 0xAA;  
  cp.setData(buffer);
  
  ASSERT_EQ(0x55AA, cp.getModuleId());
}

TEST(t_ComsPacket, getModuleID_setModuleId)
{ 
  ComsPacket cp;

  cp.setModuleId(0xAA55);  
  ASSERT_EQ(0xAA55, cp.getModuleId());
}

/* ------------------------------------------------------------ */
TEST(t_ComsPacket, getPacketId_ByteOrder)
{ 
  ComsPacket cp;
  uint8_t buffer[512];
  
  buffer[2] = 0x55;
  buffer[3] = 0xAA;  
  cp.setData(buffer);
  
  ASSERT_EQ(0x55AA, cp.getPacketId());
}

TEST(t_ComsPacket, getPacketId_setPacketId)
{ 
  ComsPacket cp;

  cp.setPacketId(0xABCD);  
  ASSERT_EQ(0xABCD, cp.getPacketId());
}

/* ------------------------------------------------------------ */
TEST(t_ComsPacket, getSeqNumber_ByteOrder)
{ 
  ComsPacket cp;
  uint8_t buffer[512];
  
  buffer[4] = 0x12;
  buffer[5] = 0x34;  
  buffer[6] = 0x56;
  buffer[7] = 0x78;  
  cp.setData(buffer);
  
  ASSERT_EQ(0x12345678, cp.getSeqNumber());
}

TEST(t_ComsPacket, getSeqNumber_setSeqNumber)
{ 
  ComsPacket cp;

  cp.setSeqNumber(0x87654321);  
  ASSERT_EQ(0x87654321, cp.getSeqNumber());
}

/* ------------------------------------------------------------ */
const uint8_t * getPayload();
const uint8_t * getData();
void setPayload(const uint8_t * payload);
void setData(const uint8_t * data);

TEST(t_ComsPacket, data_set_get )
{ 
  ComsPacket cp;
  
  uint8_t buffer[512];
  memset(buffer, 0x55, 512);
  cp.setData(buffer);
  
  ASSERT_EQ(buffer, cp.getData());
}

TEST(t_ComsPacket, data_set_headerinfo )
{ 
  ComsPacket cp;
  uint8_t buffer[512];
  memset(buffer, 0x55, 512);
  cp.setData(buffer);
  cp.setModuleId(0xAA55);  
  cp.setPacketId(0xABCD);  
  cp.setSeqNumber(0x87654321);  
  
  buffer[0] = 0xAA;
  buffer[1] = 0x55;

  buffer[2] = 0xAB;
  buffer[3] = 0xCD;

  buffer[4] = 0x87;
  buffer[5] = 0x65;
  buffer[6] = 0x43;
  buffer[7] = 0x21;
  
  ASSERT_EQ(buffer, cp.getData());
}

/* ------------------------------------------------------------ */
TEST(t_ComsPacket, payload_checkheader )
{ 
  ComsPacket cp;
  uint8_t buffer[512];
  memset(buffer, 0x55, 512);
  cp.setPayload(buffer);
  
  buffer[0] = 0;
  buffer[1] = 0;

  buffer[2] = 0;
  buffer[3] = 0;

  buffer[4] = 0;
  buffer[5] = 0;
  buffer[6] = 0;
  buffer[7] = 0;
  
  ASSERT_EQ(buffer, cp.getData());
}

TEST(t_ComsPacket, payload_checkpayload )
{ 
  ComsPacket cp;
  uint8_t buffer[512];
  memset(buffer, 0x55, 512);
  cp.setPayload(buffer);
  
   ASSERT_EQ(buffer, cp.getPayload());
}


