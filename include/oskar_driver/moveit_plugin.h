#ifndef MOVEITPLUGIN_H
#define MOVEITPLUGIN_H
#include <moveit_msgs/RobotTrajectory.h>
#include <oskar_driver/plugin.h>
#include <ros/ros.h>

namespace ahhaa_oskar
{
class MoveitPlugin : public Plugin
{
public:
  MoveitPlugin(BoardComms* comms, std::string name);
  ~MoveitPlugin();

private:
  ros::NodeHandle nh_;
  ros::Subscriber execute_trajectory_sub_;
  std::vector<uint8_t> data;

  void execute_trajectory_callback(const moveit_msgs::RobotTrajectory& trajectory_msg);
};
}  // namespace ahhaa_oskar

#endif
