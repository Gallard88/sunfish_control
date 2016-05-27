
#include "VectorMap.h"

VectorMap::VectorMap():
  outputsRunning_(false),
  motorsLocked_(false)
{
  //------------------------------
  channels_[0].setChannel(0);
  channels_[0].setMap(0, 0.0);
}

float VectorMap::rangeCheck(float f)
{
  if ( f > 1.0 ) {
    return 1.0;
  } else if ( f < -1.0 ) {
    return -1.0;
  } else {
    return f;
  }
}

bool VectorMap::areOutputsActive()
{
  return outputsRunning_;
}

void VectorMap::update(const geometry_msgs::Twist & update)
{
  outputsRunning_ = false;
  float vect[OutputChannel::vecSize];

  if ( motorsLocked_ == true ) {
    vect[OutputChannel::vecFwd]    = 0;
    vect[OutputChannel::vecStrafe] = 0;
    vect[OutputChannel::vecDive]   = 0;
    vect[OutputChannel::vecYaw]    = 0;
    vect[OutputChannel::vecRoll]   = 0.0;
    vect[OutputChannel::vecPitch]  = 0.0;

  } else {
    vect[OutputChannel::vecFwd]    = rangeCheck(update.linear.x);
    vect[OutputChannel::vecStrafe] = rangeCheck(update.linear.y);
    vect[OutputChannel::vecDive]   = rangeCheck(update.linear.z);
    vect[OutputChannel::vecYaw]    = rangeCheck(update.angular.z);

    // At the moment roll & pitch control are not supported.
    vect[OutputChannel::vecRoll]   = 0.0;
    vect[OutputChannel::vecPitch]  = 0.0;
    ROS_INFO("Update - %f, %f, %f, %f", vect[0], vect[1], vect[2], vect[3]);
  }

  for ( int i = 0; i < NUM_PWM_CHANNELS; i ++ ) {

    float d = channels_[i].run(vect);
    if ( d != 0.0 ) {
      outputsRunning_ = true;
    }
  }
  lastUpdate_ = ros::Time::now();
}

void VectorMap::watchdog(const ros::TimerEvent & e)
{
  if ( outputsRunning_ == true ) {
    if ( watchdog_ > lastUpdate_ ) {
      ROS_ERROR("Watchdog tripped");
      ROS_ERROR("Killing all thrusters");
      outputsRunning_ = false;
    }
    watchdog_ = ros::Time::now();
  }
}

float VectorMap::getChannelDuty_Float(int channel)
{
  if ( channel >= 0 && channel < NUM_PWM_CHANNELS ) {
    return channels_[channel].getDuty();
  } else {
    return 0.0;
  }
}

int VectorMap::getChannelDuty_Int  (int channel)
{
  return getChannelDuty_Float(channel) * 100.0;
}

void VectorMap::setOutputDuty(int channel, float duty)
{
  if ( channel >= 0 && channel < NUM_PWM_CHANNELS ) {
    channels_[channel].setDuty(rangeCheck(duty));
  }
}

void VectorMap::lockMotors(bool l)
{
  motorsLocked_ = l;
}







