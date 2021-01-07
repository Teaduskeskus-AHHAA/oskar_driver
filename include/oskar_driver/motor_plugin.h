#ifndef MOTORPLUGIN_H
#define MOTORPLUGIN_H
#include <geometry_msgs/Twist.h>
#include <oskar_driver/plugin.h>
#include <ros/ros.h>

namespace ahhaa_oskar
{
class MotorPlugin : public Plugin
{
public:
  MotorPlugin(BoardComms* comms, std::string name);
  ~MotorPlugin();

private:
  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
  void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg);
};
}  // namespace ahhaa_oskar

#endif