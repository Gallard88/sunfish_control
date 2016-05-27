
#include "ros/ros.h"
#include "outputChannel.h"

OutputChannel::OutputChannel():
  channel_(-1),  newDuty_(0.0),
  oldDuty_(0.0)
{
  for ( int i = 0; i < 6; i ++ ) {
    map_[i] = 0.0;
  }
}

void OutputChannel::setChannel(int channel)
{
  channel_ = channel;
}

int OutputChannel::getChannel(void)
{
  return channel_;
}

void OutputChannel::setMap(int vect, float value)
{
  if ( vect >= 0 && vect < vecSize ) {
    if ( value > 1.0 ) {
      map_[vect] = 1.0;
    } else if ( value < -1.0 ) {
      map_[vect] = -1.0;
    } else {
      map_[vect] = value;
    }
  }
}

float OutputChannel::getMap(int vect)
{
  if ( vect >= 0 && vect < vecSize ) {
    return map_[vect];
  }
  return 0;
}

float OutputChannel::run(const float * power)
{
  if ( channel_ < 0 ) {
    return 0.0;
  }
  ROS_ASSERT(power != NULL);
  oldDuty_ = newDuty_;

  newDuty_ = 0.0;
  for ( int i = 0; i < vecSize; i ++ ) {
      newDuty_ += map_[i] * power[i];
  }
  return newDuty_;
}

void OutputChannel::setDuty(float duty)
{
  oldDuty_ = newDuty_;
  newDuty_ = duty;
}

float OutputChannel::getDuty(void)
{
  return newDuty_;
}

bool OutputChannel::isDutyUpdated(void)
{
  return ( newDuty_ != oldDuty_)? true: false;
}
