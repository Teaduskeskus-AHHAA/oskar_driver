#include <oskar_driver/driver.h>
#include <ros/ros.h>

using namespace ahhaa_oskar;

int main(int argc, char** argv)
{
  // Register a ROS node
  ros::init(argc, argv, "ahhaa_oskar_driver_node");
  ros::NodeHandle nh;

  // Create a driver instance
  Driver drv;

  // Spin forever
  ros::spin();
}