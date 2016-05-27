#ifndef VECTORMAP_H
#define VECTORMAP_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "outputChannel.h"

class VectorMap
{
public:
  VectorMap();

#define NUM_PWM_CHANNELS  12
  static float rangeCheck(float f);

  bool areOutputsActive();
  void update(const geometry_msgs::Twist & update);
  void watchdog(const ros::TimerEvent & e);

  float getChannelDuty_Float(int channel);
  int   getChannelDuty_Int  (int channel);

  void setOutputDuty(int channel, float duty);
  void lockMotors(bool l);

private:
  bool outputsRunning_;
  bool motorsLocked_;
  OutputChannel channels_[NUM_PWM_CHANNELS];
  ros::Time lastUpdate_;
  ros::Time watchdog_;

};

#endif
