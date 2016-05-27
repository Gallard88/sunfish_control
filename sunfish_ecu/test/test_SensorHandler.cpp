#include <gtest/gtest.h>

#include "../src/SensorHandler.h"

const ComsPacket TestMsg;

/* ----------------------------------------------------- */
TEST(t_SensorHandler, construct_deconstruct_no_errors_1)
{
  SensorHandler sh;
}

/* ----------------------------------------------------- */
TEST(t_SensorHandler, construct_default_bool_IntTemp)
{
  SensorHandler sh;
  ASSERT_EQ( false, sh.newIntTemp());
}

TEST(t_SensorHandler, construct_check_default_IntTemp)
{
  SensorHandler sh;
  sensor_msgs::Temperature t = sh.getIntTemp();
  ASSERT_EQ( 0.0, t.temperature);
}

TEST(t_SensorHandler, IntTemp_DoubleBoolCheck)
{
  SensorHandler sh;

  sh.receiveMsg(TestMsg);
  sh.newIntTemp();
  ASSERT_EQ( true, sh.newIntTemp());
}

TEST(t_SensorHandler, IntTemp_Read_BoolCheck)
{
  SensorHandler sh;

  sh.receiveMsg(TestMsg);
  sh.getIntTemp();
  ASSERT_EQ( false, sh.newIntTemp());
}


/* ----------------------------------------------------- */
TEST(t_SensorHandler, construct_default_bool_ExtTemp)
{
  SensorHandler sh;
  ASSERT_EQ( false, sh.newExtTemp());
}

TEST(t_SensorHandler, construct_check_default_ExtTemp)
{
  SensorHandler sh;
  sensor_msgs::Temperature t = sh.getExtTemp();
  ASSERT_EQ( 0.0, t.temperature);
}

TEST(t_SensorHandler, ExtTemp_DoubleBoolCheck)
{
  SensorHandler sh;

  sh.receiveMsg(TestMsg);
  sh.newExtTemp();
  ASSERT_EQ( true, sh.newExtTemp());
}

TEST(t_SensorHandler, ExtTemp_Read_BoolCheck)
{
  SensorHandler sh;

  sh.receiveMsg(TestMsg);
  sh.getExtTemp();
  ASSERT_EQ( false, sh.newExtTemp());
}

/* ----------------------------------------------------- */
TEST(t_SensorHandler, construct_default_bool_Power)
{
  SensorHandler sh;
  ASSERT_EQ( false, sh.newPower());
}

TEST(t_SensorHandler, Power_DoubleBoolCheck)
{
  SensorHandler sh;

  sh.receiveMsg(TestMsg);
  sh.newPower();
  ASSERT_EQ( true, sh.newPower());
}

TEST(t_SensorHandler, Power_Read_BoolCheck)
{
  SensorHandler sh;

  sh.receiveMsg(TestMsg);
  sh.getPower();
  ASSERT_EQ( false, sh.newPower());
}

/* ----------------------------------------------------- */
TEST(t_SensorHandler, construct_default_bool_Status)
{
  SensorHandler sh;
  ASSERT_EQ( false, sh.newStatus());
}

TEST(t_SensorHandler, Status_DoubleBoolCheck)
{
  SensorHandler sh;

  sh.receiveMsg(TestMsg);
  sh.newStatus();
  ASSERT_EQ( true, sh.newStatus());
}

TEST(t_SensorHandler, Statusp_Read_BoolCheck)
{
  SensorHandler sh;

  sh.receiveMsg(TestMsg);
  sh.getStatus();
  ASSERT_EQ( false, sh.newStatus());
}

/* ----------------------------------------------------- */
TEST(t_SensorHandler, construct_default_bool_INS)
{
  SensorHandler sh;
  ASSERT_EQ( false, sh.newINS());
}

TEST(t_SensorHandler, INS_DoubleBoolCheck)
{
  SensorHandler sh;

  sh.receiveMsg(TestMsg);
  sh.newINS();
  ASSERT_EQ( true, sh.newINS());
}

TEST(t_SensorHandler, INS_Read_BoolCheck)
{
  SensorHandler sh;

  sh.receiveMsg(TestMsg);
  sh.getINS();
  ASSERT_EQ( false, sh.newINS());
}

/* ----------------------------------------------------- */
TEST(t_SensorHandler, construct_default_bool_Depth)
{
  SensorHandler sh;
  ASSERT_EQ( false, sh.newDepth());
}

TEST(t_SensorHandler, Depth_DoubleBoolCheck)
{
  SensorHandler sh;

  sh.receiveMsg(TestMsg);
  sh.newDepth();
  ASSERT_EQ( true, sh.newDepth());
}

TEST(t_SensorHandler, Depth_Read_BoolCheck)
{
  SensorHandler sh;

  sh.receiveMsg(TestMsg);
  sh.getDepth();
  ASSERT_EQ( false, sh.newDepth());
}

/* ----------------------------------------------------- */
TEST(t_SensorHandler, construct_default_bool_Mag)
{
  SensorHandler sh;
  ASSERT_EQ( false, sh.newMagField());
}

TEST(t_SensorHandler, Mag_DoubleBoolCheck)
{
  SensorHandler sh;

  sh.receiveMsg(TestMsg);
  sh.newMagField();
  ASSERT_EQ( true, sh.newMagField());
}

TEST(t_SensorHandler, Mag_Read_BoolCheck)
{
  SensorHandler sh;

  sh.receiveMsg(TestMsg);
  sh.getMagField();
  ASSERT_EQ( false, sh.newMagField());
}


