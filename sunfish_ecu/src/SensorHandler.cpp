#include "ros/ros.h"
#include "SensorHandler.h"
#include "ComsPacket.h"


SensorHandler::SensorHandler():
  newIntTemp_(false),
  newExtTemp_(false),
  newPower_(false),
  newStatus_(false),
  newINS_(false),
  newDepth_(false),
  newMag_(false)
{

}

SensorHandler::~SensorHandler()
{

}

void SensorHandler::receiveMsg(const ComsPacket & msg)
{

  newIntTemp_ = true;
  newExtTemp_ = true;
  newPower_ = true;
  newStatus_ = true;
  newINS_ = true;
  newDepth_ = true;
  newMag_ = true;
}

sensor_msgs::Temperature SensorHandler::getIntTemp()
{
  newIntTemp_ = false;
  return intTemp_;
}

sensor_msgs::Temperature SensorHandler::getExtTemp()
{
  newExtTemp_ = false;
  return extTemp_;
}

sunfish_ecu::Power SensorHandler::getPower()
{
  newPower_ = false;
  return power_;
}

sunfish_ecu::Status SensorHandler::getStatus()
{
  newStatus_ = false;
  return status_;
}

sunfish_ecu::INS SensorHandler::getINS()
{
  newINS_ = false;
  return ins_;
}

sunfish_ecu::Depth SensorHandler::getDepth()
{
  newDepth_ = false;
  return depth_;
}

sunfish_ecu::MagField SensorHandler::getMagField()
{
  newMag_ = false;
  return mag_;
}

bool SensorHandler::newIntTemp()
{
  return newIntTemp_;
}

bool SensorHandler::newExtTemp()
{
  return newExtTemp_;
}

bool SensorHandler::newPower()
{
  return newPower_;
}

bool SensorHandler::newStatus()
{
  return newStatus_;
}

bool SensorHandler::newINS()
{
  return newINS_;
}

bool SensorHandler::newDepth()
{
  return newDepth_;
}

bool SensorHandler::newMagField()
{
  return newMag_;
}



