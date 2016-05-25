#include <gtest/gtest.h>

#include "../src/outputChannel.h"

TEST(t_outputChannel, contructor_default_channel){
  
  OutputChannel out;

  ASSERT_EQ(-1, out.getChannel());
}

TEST(t_outputChannel, contructor_default_duty){
  
  OutputChannel out;

  ASSERT_EQ(0.0, out.getDuty());
}

TEST(t_outputChannel, contructor_default_dutyUpdated){
  
  OutputChannel out;

  ASSERT_EQ(false, out.isDutyUpdated());
}

/* ------------------------------------------------------------ */
// Set Channel
TEST(t_outputChannel, setChannel_Good){
  
  OutputChannel out;
  
  out.setChannel(0);
  ASSERT_EQ(0, out.getChannel());
}

/* ------------------------------------------------------------ */
// set Map
TEST(t_outputChannel, setMap_Default){
  
  OutputChannel out;
  
  for ( int i = 0; i < 6; i ++ ) {
    ASSERT_EQ(0.0, out.getMap(i));
  }  
}

TEST(t_outputChannel, setMap_SetGood){
  
  OutputChannel out;
  
  out.setMap(0, 1.0);
  ASSERT_EQ(1.0, out.getMap(0));
}

TEST(t_outputChannel, setMap_SetTooHigh){
  
  OutputChannel out;
  
  out.setMap(0, 11.1);
  ASSERT_EQ(1.0, out.getMap(0));
}

TEST(t_outputChannel, setMap_SetTooLow){
  
  OutputChannel out;
  
  out.setMap(0, -11.1);
  ASSERT_EQ(-1.0, out.getMap(0));
}

/* ------------------------------------------------------------ */
//  Run
TEST(t_outputChannel, run_normal){
  
  OutputChannel out;
  float vector[6] = { 0 };
  
  out.setChannel(1);
  out.setMap(0, 1.0);
  vector[0] = 1.0;
  out.run(vector);
  
  ASSERT_EQ(1.0, out.getDuty());
}

TEST(t_outputChannel, run_normal_checkduty_changed){
  
  OutputChannel out;
  float vector[6];
  
  out.setChannel(1);
  out.setMap(0, 1.0);
  vector[0] = 1.0;
  vector[1] = 0.0;
  vector[2] = 0.0;
  vector[3] = 0.0;
  vector[4] = 0.0;
  vector[5] = 0.0;
  out.run(vector);
  vector[0] = 0.1;
  out.run(vector);
  
  ASSERT_EQ(true, out.isDutyUpdated());
}

TEST(t_outputChannel, run_normal_checkduty_hasnt_changed){
  
  OutputChannel out;
  float vector[6] = { 0 };
  
  out.setChannel(1);
  out.setMap(0, 1.0);
  vector[0] = 1.0;
  out.run(vector);
  out.run(vector);
  
  ASSERT_EQ(false, out.isDutyUpdated());
}

TEST(t_outputChannel, run_check_scale){
  
  OutputChannel out;
  float vector[6] = { 0 };
  
  out.setChannel(1);
  out.setMap(0, 0.5);
  vector[0] = 1.0;
  out.run(vector);
  
  ASSERT_EQ(0.5, out.getDuty());
}

TEST(t_outputChannel, run_check_multiVector_singleChannel){
  
  OutputChannel out;
  float vector[6] = { 0 };
  
  out.setChannel(1);
  out.setMap(0, 0.5);
  vector[0] = 1.0;
  vector[1] = 1.0;
  out.run(vector);
  
  ASSERT_EQ(0.5, out.getDuty());
}

TEST(t_outputChannel, run_check_multiVector_multiChannel){
  
  OutputChannel out;
  float vector[6] = { 0 };
  
  out.setChannel(1);
  out.setMap(0, 1.0);
  out.setMap(1, 1.0);
  vector[0] = 1.0;
  vector[1] = 1.0;
  out.run(vector);
  
  ASSERT_EQ(2.0, out.getDuty());
}

/* ------------------------------------------------------------ */
