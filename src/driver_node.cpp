#include <oskar_driver/driver.h>
#include <ros/ros.h>
#include <oskar_driver/oskar_trajectory_follower.h>
using namespace ahhaa_oskar;

int main(int argc, char** argv)
{
  // Register a ROS node
  ros::init(argc, argv, "ahhaa_oskar_driver_node");
  ros::NodeHandle nh;

  // Create a driver instance
  Driver drv;

  OskarTrajectoryFollower df("oskar3_left_hand_controller/follow_joint_trajectory");

  // Spin forever
  ros::spin();
}