#include <oskar_driver/motor_plugin.h>

namespace ahhaa_oskar
{
MotorPlugin::MotorPlugin(BoardComms* comms, std::string name) : Plugin(comms, name)
{
  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &MotorPlugin::cmd_vel_callback, this);
}

MotorPlugin::~MotorPlugin()
{
}

void MotorPlugin::cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg)
{
  ROS_INFO_STREAM("Heard:" << cmd_vel_msg);
}

}  // namespace ahhaa_oskar