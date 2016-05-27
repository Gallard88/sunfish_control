#ifndef SENSORHANDER_H
#define SENSORHANDER_H

#include "sunfish_ecu/Depth.h"
#include "sunfish_ecu/INS.h"
#include "sunfish_ecu/MagField.h"
#include "sunfish_ecu/Power.h"
#include "sunfish_ecu/Status.h"
#include "sensor_msgs/Temperature.h"
#include "sunfish_ecu/setPWM.h"

#include "ComsPacket.h"

class SensorHandler
{
public:
  SensorHandler();
  ~SensorHandler();

  void receiveMsg(const ComsPacket & msg);
  sensor_msgs::Temperature getIntTemp();
  sensor_msgs::Temperature getExtTemp();
  sunfish_ecu::Power       getPower();
  sunfish_ecu::Status      getStatus();
  sunfish_ecu::INS         getINS();
  sunfish_ecu::Depth       getDepth();
  sunfish_ecu::MagField    getMagField();

  bool newIntTemp();
  bool newExtTemp();
  bool newPower();
  bool newStatus();
  bool newINS();
  bool newDepth();
  bool newMagField();

private:
  sensor_msgs::Temperature intTemp_;
  sensor_msgs::Temperature extTemp_;
  sunfish_ecu::Power power_;
  sunfish_ecu::Status status_;
  sunfish_ecu::INS ins_;
  sunfish_ecu::Depth depth_;
  sunfish_ecu::MagField mag_;

  bool newIntTemp_;
  bool newExtTemp_;
  bool newPower_;
  bool newStatus_;
  bool newINS_;
  bool newDepth_;
  bool newMag_;

};


#endif

