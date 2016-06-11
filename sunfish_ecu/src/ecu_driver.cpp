
#include <string>
#include <vector>

#include "ros/ros.h"

#include "sunfish_ecu/Depth.h"
#include "sunfish_ecu/INS.h"
#include "sunfish_ecu/MagField.h"
#include "sunfish_ecu/Power.h"
#include "sunfish_ecu/Status.h"
#include "sensor_msgs/Temperature.h"  // Twist
#include "sunfish_ecu/setPWM.h"


#include "VectorMap.h"
#include "SensorHandler.h"
#include "HardwareComs.h"

static ros::Publisher  intTemp_Pub_;
static ros::Publisher  extTemp_Pub_;
static ros::Publisher  power_Pub_;
static ros::Publisher  status_Pub_;
static ros::Publisher  INS_Pub_;
static ros::Publisher  depth_Pub_;
static ros::Publisher  MST_Pub_;

static ros::ServiceServer  PWM_Srv_;
static ros::Timer  heartbeatTimer_;

static ros::Subscriber joySub_;
static ros::Timer      updateTimer;

static VectorMap vecMap_;
static SensorHandler sensorHandler_;
static HardwareComs hwdComs_;

/* ================================================================== */
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

  // Runs USB Coms
  // Process and received packets
  ComsPacket p;

  bool packetReceived = hwdComs_.run(20, NULL, &p);

  if (packetReceived == false ) {
    return;
  }

  // Check if we need to publish any new messages.
  sensorHandler_.receiveMsg(p);

  if ( sensorHandler_.newIntTemp() ) {
    intTemp_Pub_.publish(sensorHandler_.getIntTemp());
    ROS_INFO("pub - IntTemp");
  }
  if ( sensorHandler_.newExtTemp() ) {
    extTemp_Pub_.publish(sensorHandler_.getExtTemp());
    ROS_INFO("pub - ExtTemp");
  }
  if ( sensorHandler_.newPower() ) {
    power_Pub_.publish(sensorHandler_.getPower());
    ROS_INFO("pub - Power");
  }
  if ( sensorHandler_.newStatus() ) {
    status_Pub_.publish(sensorHandler_.getStatus());
    ROS_INFO("pub - Status");
  }
  if ( sensorHandler_.newINS() ) {
    INS_Pub_.publish(sensorHandler_.getINS());
    ROS_INFO("pub - INS");
  }
  if ( sensorHandler_.newDepth() ) {
    depth_Pub_.publish(sensorHandler_.getDepth());
    ROS_INFO("pub - Depth");
  }
  if ( sensorHandler_.newMagField() ) {
    MST_Pub_.publish(sensorHandler_.getMagField());
    ROS_INFO("pub - Mag");
  }

}

/* ================================================================== */
bool isDebug()
{
  std::string val;
  ros::param::param<std::string>("/sunfish/ecu/mode", val, "debug-");
  if ( val == "debug") {
    ROS_INFO("Starting in debug mode");
    return true;

  } else if ( val == "active" ) {
    ROS_INFO("Starting in active mode");
    return false;

  } else {
    ROS_INFO("Mode = unknown, defaulting to debug");
    return true;
  }
}

static void loadOutputMapping(int ch, const char *name)
{
  ros::NodeHandle n;
  std::vector<double> val;
  float array[6];

  if ( !n.hasParam(name)) {
    return;
  }

  n.getParam(name, val);
  for(unsigned i=0; i < val.size(); i++) {
    array[i] = val[i];
  }
  vecMap_.configureOutputChannel(ch, array);
}

/* ================================================================== */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ecu_driver");
  ros::NodeHandle n;

  ROS_INFO("Sunfish ECU Online");

  if ( isDebug() == true ) {
    ROS_INFO("All outputs are IN-ACTIVE");
    vecMap_.lockMotors(true);
  } else {
    ROS_INFO("All outputs are ACTIVE");
    vecMap_.lockMotors(false);
  }

  for ( int i = 0; i < NUM_PWM_CHANNELS; i ++ ) {
    char name[124];
    sprintf(name, "/sunfish/ecu/output%d", i);
    loadOutputMapping(i, name);
  }

  hwdComs_.setDevName("/dev/hid0");
  hwdComs_.connect();

  // ------------------------------------------------
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
  joySub_ = n.subscribe("/sunfish/ecu/vector", 2, &VectorMap::update, &vecMap_);
  updateTimer = n.createTimer(ros::Duration(1.0), &VectorMap::watchdog, &vecMap_);

  // ------------------------------------------------
  // Now we let ros::spin() do all the hard work!

  ROS_INFO("ECU driver active");
  ros::spin();
  return 0;
}

