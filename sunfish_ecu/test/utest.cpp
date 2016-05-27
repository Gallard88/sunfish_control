#include <gtest/gtest.h>
#include <climits>

#include "ros/ros.h"

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){

  ros::Time::init();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
