#include "ros/ros.h"
#include "SensorHandler.h"
#include "ComsPacket.h"

#include <cstring>
/*

// Power
float Voltage_Avg
float Voltage_Current
uint32_t SampleRate
float32 voltage_[100];
float32 current_[100];

// INS Data
float32 AbsOrientation_Euler[3] # (Euler Vector, 100Hz)
                               # Three axis orientation data based on a 360° sphere
geometry_msgs::Quaternion AbsoluteOrientation_Quart[3] # (Quaterion, 100Hz)
                               # Four point quaternion output for more accurate data manipulation
float32 AngularVel[3]     # (100Hz)
                               # Three axis of 'rotation speed' in rad/s
float32 AccelVector  # (100Hz)
                               # Three axis of acceleration (gravity + linear motion) in m/s^2
float32 MagField       # (20Hz)
                               # Three axis of magnetic field sensing in micro Tesla (uT)
float32 LinAccVector[3] # (100Hz)
                               # Three axis of linear acceleration data (acceleration minus gravity) in m/s^2
float32 GravityVector[3]      # (100Hz)
                               # Three axis of gravitational acceleration (minus any movement) in m/s^2
float32 Temperature         # (1Hz)
                               # Ambient temperature in degrees celsius

So how do we plan this.

One thought is a HID driver moving large (512 byte) chunks as needed.
Nearly all of the data above could be packed into one massive struct.
Add a bit mask to indicate whats changed...
Most data updates at 100 Hz. The flag field is useful for other fields that update less quickly.

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t w;
} Quart_Int;


typedef struct {
    char Firmware[16];      //  0
    uint16_t HwdType;       // 16
    uint16_t Flags;         // 18
    uint16_t depth_pressure;// 20
    int16_t  depth_temperature; // 22

    // INS
    int16_t   AbsOrientation_Euler[3];
    Quart_Int AbsoluteOrientation_Quart[3];
    int16_t   AngularVel[3];
    int16_t   AccelVector[3];
    int16_t   MagField[3];
    int16_t   LinAccVector[3];
    int16_t   GravityVector[3];
    int16_t   Temp;

    // power
    uint32_t SampleRate;
    uint16_t volt_[100];
    int16_t current_[100];


}  ECU_DataModel;
*/


SensorHandler::SensorHandler():
  lastSeqNum_(0),
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

const uint16_t SensorHandler::MODULE_ID = 0x0001;
const uint16_t SensorHandler::DATA_PACKET_ID = 0x0001;
const uint8_t  SensorHandler::UPDATE_INS_MAG = 0x01;
const uint8_t  SensorHandler::UPDATE_INS_TEMP = 0x02;

void SensorHandler::receiveMsg(const ComsPacket & msg)
{
  if ( msg.getModuleId() != MODULE_ID) {
    return;
  }
  if ( msg.getPacketId() != DATA_PACKET_ID ) {
    return;
  }
  uint32_t s = msg.getSeqNumber();
  if ( s == lastSeqNum_ ) {
    return;
  }
  if ( (s+1) != lastSeqNum_ ) {
    ROS_INFO("We appear to have missed USB packets");
    ROS_INFO("S: %d", s);
    ROS_INFO("LastSeqNum_: %d", lastSeqNum_);
  }
  lastSeqNum_ = s;

  const uint8_t *data = msg.getPayload();
  processStatus(data);
  processDepth(data);
  processINS(data);
  processPower(data);
}

void SensorHandler::processStatus(const uint8_t *ptr)
{
  char f[20];

  strncpy(f, (const char *)ptr, 16);
  f[15] = 0;
  status_.Stamp = ros::Time::now();
  status_.HardwareOK = true;
  status_.Firmware = std::string(f);
  status_.Hardware = "Sunfish_ECU";
  status_.Voltage = 13.8; // Volt
  status_.Current =  1.0; // Amp
  status_.InternalTemp = 25.0;
  status_.ExternalTemp = 35.0;
  status_.Fault_PWM_1 = false;
  status_.Fault_PWM_2 = false;
  status_.Fault_PWM_3 = false;

  newStatus_ = true;
}

void SensorHandler::processDepth(const uint8_t *ptr)
{

  depth_.Stamp = ros::Time::now();
  depth_.Pressure = 0.0;
  depth_.Temperature = 0.0;
  newDepth_ = true;

  extTemp_.temperature = depth_.Temperature;
  newExtTemp_ = true;
}

void SensorHandler::processINS(const uint8_t *ptr)
{
  uint8_t data_update = 0;

  ins_.Stamp = ros::Time::now();

  ins_.AbsOrientation[0] = 0.0;
  ins_.AbsOrientation[1] = 0.0;
  ins_.AbsOrientation[2] = 0.0;

  ins_.AngularVelocity[0] = 0.0;
  ins_.AngularVelocity[1] = 0.0;
  ins_.AngularVelocity[2] = 0.0;

  ins_.AccelerationVector[0] = 0.0;
  ins_.AccelerationVector[1] = 0.0;
  ins_.AccelerationVector[2] = 0.0;

  if ( data_update & UPDATE_INS_MAG ) {
    ins_.MagneticField[0] = 0.0;
    ins_.MagneticField[1] = 0.0;
    ins_.MagneticField[2] = 0.0;

    mag_.Stamp = ros::Time::now();
    mag_.MagneticField[0] = ins_.MagneticField[0];
    mag_.MagneticField[1] = ins_.MagneticField[1];
    mag_.MagneticField[2] = ins_.MagneticField[2];
    newMag_ = true;
  }

  ins_.LinearAcceleration[0] = 0.0;
  ins_.LinearAcceleration[1] = 0.0;
  ins_.LinearAcceleration[2] = 0.0;

  ins_.GravityVector[0] = 0.0;
  ins_.GravityVector[1] = 0.0;
  ins_.GravityVector[2] = 0.0;


  if ( data_update & UPDATE_INS_TEMP ) {
    ins_.Temperature = 25.0;
    intTemp_.temperature = ins_.Temperature;
    newIntTemp_ = true;
  }

  newINS_ = true;

}

void SensorHandler::processPower(const uint8_t *ptr)
{
  power_.Stamp = ros::Time::now();
  power_.SampleRate = 100;
  for ( int i = 0; i < 100; i++ ) {
    power_.Voltage[i] = 13.8;
    power_.Current[i] = 1.1;
  }
  newPower_ = true;
}

/* ------------------------------------------------- */
std::string SensorHandler::getFirmwareVer()
{
  return firmwareVer_;
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

