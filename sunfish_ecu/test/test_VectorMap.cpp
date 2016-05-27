#include <gtest/gtest.h>

#include "../src/VectorMap.h"

TEST(t_VectorMap, contructor_default_channel)
{
  VectorMap vm;

  for ( int i = 0; i < NUM_PWM_CHANNELS; i ++ ) {
    ASSERT_EQ(0.0, vm.getChannelDuty_Float(i));
    ASSERT_EQ(  0, vm.getChannelDuty_Int(i));
  }
}

TEST(t_VectorMap, contructor_default_outputsActive)
{
  VectorMap vm;

  ASSERT_EQ(false, vm.areOutputsActive());
}


/* ------------------------------------------------------------ */
TEST(t_VectorMap, range_check_normal)
{ 
  ASSERT_EQ(0.0, VectorMap::rangeCheck(0.0));
}

TEST(t_VectorMap, range_check_upperlimit)
{
  ASSERT_EQ(1.0, VectorMap::rangeCheck(1.0));
}

TEST(t_VectorMap, range_check_lowerlimit)
{
  ASSERT_EQ(-1.0, VectorMap::rangeCheck(-1.0));
}

TEST(t_VectorMap, range_check_beyond_upperlimit)
{
  ASSERT_EQ(1.0, VectorMap::rangeCheck(1.1));
}

TEST(t_VectorMap, range_check_beyond_lowerlimit)
{
  ASSERT_EQ(-1.0, VectorMap::rangeCheck(-1.1));
}

/* ------------------------------------------------------------ */
/* ------------------------------------------------------------ */
/*
TEST(t_VectorMap, update_)
{
  VectorMap vm;
  

  ASSERT_EQ(0.0, vm.getChannelDuty_Float());
}
*/
/* ------------------------------------------------------------ */
TEST(t_VectorMap, getDuty_Float_BadChannel_Under)
{
  VectorMap vm;

  ASSERT_EQ(0.0, vm.getChannelDuty_Float(NUM_PWM_CHANNELS));
}

TEST(t_VectorMap, getDuty_Float_BadChannel_Over)
{
  VectorMap vm;
  ASSERT_EQ(0.0, vm.getChannelDuty_Float(-1));
}

/* ------------------------------------------------------------ */
TEST(t_VectorMap, getDuty_Int_BadChannel_Under)
{
  VectorMap vm;

  ASSERT_EQ(0.0, vm.getChannelDuty_Int(NUM_PWM_CHANNELS+1));
}

TEST(t_VectorMap, getDuty_Int_BadChannel_Over)
{
  VectorMap vm;
  ASSERT_EQ(0.0, vm.getChannelDuty_Int(-1));
}

/* ------------------------------------------------------------ */
TEST(t_VectorMap, setOutputDuty_OutputEnabled)
{
  VectorMap vm;

  vm.setOutputDuty(0, 1.0);
  ASSERT_EQ(100, vm.getChannelDuty_Int(0));
}

TEST(t_VectorMap, setOutputDuty_OutputEnabled_NegativePower)
{
  VectorMap vm;

  vm.setOutputDuty(0, -1.0);
  ASSERT_EQ(-100, vm.getChannelDuty_Int(0));
}

TEST(t_VectorMap, setOutputDuty_OutputHalfPower)
{
  VectorMap vm;

  vm.setOutputDuty(0, 0.5);
  ASSERT_EQ(50, vm.getChannelDuty_Int(0));
}

TEST(t_VectorMap, setOutputDuty_OutputEnabled_CheckNeighbour)
{
  VectorMap vm;

  vm.setOutputDuty(0, 1.0);
  ASSERT_EQ(0, vm.getChannelDuty_Int(1));
}

/* ------------------------------------------------------------ */
TEST(t_VectorMap, update_all_0)
{
  VectorMap vm;

  geometry_msgs::Twist t;

  t.linear.x = 0.0;
  t.linear.y = 0.0;
  t.linear.z = 0.0;
  t.angular.x = 0.0;
  t.angular.y = 0.0;
  t.angular.z = 0.0;

  vm.update(t);
  ASSERT_EQ(0, vm.getChannelDuty_Int(0));
}

/* ------------------------------------------------------------ */
TEST(t_VectorMap, motorlock_active)
{
  VectorMap vm;

  geometry_msgs::Twist t;

  t.linear.x = 1.0;
  t.linear.y = 1.0;
  t.linear.z = 1.0;
  t.angular.x = 1.0;
  t.angular.y = 1.0;
  t.angular.z = 1.0;

  vm.lockMotors(true);
  vm.update(t);
  ASSERT_EQ(0, vm.getChannelDuty_Int(0));
}
