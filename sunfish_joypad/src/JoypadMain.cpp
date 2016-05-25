#include "ros/ros.h"

#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"  // Twist

/* ------------------------------------------------------- */
static ros::Subscriber joySub_;
static ros::Publisher  vecPub_;
static ros::Timer      updateTimer;
static geometry_msgs::Twist cmdVector_;
static bool diveInc_, diveDec_;
static double depth_;
static const double depthInc_ = 0.1;

/* ------------------------------------------------------- */
static void joystickUpdate(const sensor_msgs::Joy & update)
{
  if (( diveInc_ == false ) && ( update.buttons[5] != 0 )) {
    depth_ += depthInc_;
    if ( depth_ > 0.0 )
       depth_ = 0.0;
  }

  if (( diveDec_ == false ) && ( update.buttons[4] != 0 )) {
      depth_ -= depthInc_;
      if ( depth_ < -1.0 )
       depth_ = -1.0;
  }
  diveInc_ = update.buttons[5];
  diveDec_ = update.buttons[4];

  cmdVector_.linear.x = update.axes[1];
  cmdVector_.linear.y = update.axes[3] * -1.0;
  cmdVector_.linear.z = depth_;
  cmdVector_.angular.x = 0.0;
  cmdVector_.angular.y = 0.0;
  cmdVector_.angular.z = update.axes[0] * -1.0;

}

static void callbackTimer(const ros::TimerEvent & e)
{
  vecPub_.publish(cmdVector_);
}

/* ------------------------------------------------------- */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "sunfish_joypad");
  ros::NodeHandle n;

  diveInc_ = diveDec_ = false;
  depth_ = 0.0;


  ROS_INFO("Sunfish Joypad online");
  joySub_  = n.subscribe("/joy", 1, joystickUpdate);
  vecPub_  = n.advertise<geometry_msgs::Twist>("/sunfish/ecu/vector", 2);

  double update;
  n.param("/sunfish/joy_update", update, 10.0);
  if ( update < 1.0 ) {
    update = 1.0;
  } else if ( update > 50.0 ) {
    update = 50.0;
  }
  updateTimer = n.createTimer(ros::Duration(1.0/update), &callbackTimer);

  ros::spin();
  return 0;
}
