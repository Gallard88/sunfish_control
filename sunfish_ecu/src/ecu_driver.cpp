#include "ros/ros.h"

#include "sunfish_ecu/Depth.h"
#include "sunfish_ecu/INS.h"
#include "sunfish_ecu/MagField.h"
#include "sunfish_ecu/Power.h"
#include "sunfish_ecu/Status.h"
#include "sensor_msgs/Temperature.h"  // Twist
#include "sunfish_ecu/setPWM.h"

#include "VectorMap.h"

static ros::Publisher  intTemp_Pub_;
static ros::Publisher  extTemp_Pub_;
static ros::Publisher  power_Pub_;
static ros::Publisher  status_Pub_;
static ros::Publisher  INS_Pub_;
static ros::Publisher  depth_Pub_;
static ros::Publisher  MST_Pub_;

static ros::ServiceServer  PWM_Srv_;
static ros::Timer  heartbeatTimer_;

static VectorMap vecMap_;

/* ================================================================== */
/*

Hardware Data Model
uint8_t PWM_Channels[12];

// Temperature
float internalTemp_;
float externalTemp_;

// Depth
float Depth_Pressure;
float Depth_Temperature;

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


*/
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t w;
} Quart_Int;

typedef struct {
    uint16_t PacketHeader;
    char Firmware[16];
    uint16_t HwdType;
    uint16_t Flags;
    uint16_t depth_pressure;
    int16_t  depth_temperature;

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

typedef union {
  ECU_DataModel s;
  uint8_t       u[512];
} UnionPacket;

/* ================================================================== */
bool serviceHandler_PWM_Single(sunfish_ecu::setPWM::Request  &req,
                               sunfish_ecu::setPWM::Response &res)
{
  if ( req.Channel < NUM_PWM_CHANNELS ) {

    if ( req.Duty <= 100 ) {
      float d = (float) req.Duty / 100.0;
      vecMap_.setOutputDuty(req.Channel, d);
      ROS_INFO("setPWM() = %d, %d", req.Channel, req.Duty);
    }
  }
  return true;
}


/* ================================================================== */
static void TimerCallback(const ros::TimerEvent & e)
{// here every 5 ms, maybe

}

/* ================================================================== */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ecu_driver");
  ros::NodeHandle n;

  intTemp_Pub_ = n.advertise<sensor_msgs::Temperature>("/sunfish/ecu/int_temp", 5);
  extTemp_Pub_ = n.advertise<sensor_msgs::Temperature>("/sunfish/ecu/ext_temp", 5);
  power_Pub_   = n.advertise<sunfish_ecu::Power>("/sunfish/ecu/power", 5);
  status_Pub_  = n.advertise<sunfish_ecu::Status>("/sunfish/ecu/status", 5);
  INS_Pub_     = n.advertise<sunfish_ecu::INS>("/sunfish/ecu/INS", 10);
  depth_Pub_   = n.advertise<sunfish_ecu::Depth>("/sunfish/ecu/depth", 5);
  MST_Pub_     = n.advertise<sunfish_ecu::MagField>("/sunfish/ecu/MST", 5);

  // ------------------------------------------------
  // Set up file descriptor that allows us to talk to the hardware.
  heartbeatTimer_ = n.createTimer(ros::Duration(0.005), &TimerCallback);

  // ------------------------------------------------
  // Set up service to manually control PWM outputs.
  PWM_Srv_ = n.advertiseService("/sunfish/ecu/PWM", serviceHandler_PWM_Single);

  // ------------------------------------------------
  // Set up Control Vector sub-system.
  ros::Subscriber joySub_ = n.subscribe("/sunfish/ecu/vector", 2, &VectorMap::update, &vecMap_);
  ros::Timer      updateTimer = n.createTimer(ros::Duration(1.0), &VectorMap::watchdog, &vecMap_);

  // ------------------------------------------------
  // Now we let ros::spin() do all the hard work!

  ROS_INFO("Sunfish ECU Online");
  ros::spin();
  return 0;
}
