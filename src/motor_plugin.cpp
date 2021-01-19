#include <oskar_driver/motor_plugin.h>

namespace ahhaa_oskar
{
MotorPlugin::MotorPlugin(BoardComms* comms, std::string name) : Plugin(comms, name)
{
  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &MotorPlugin::cmd_vel_callback, this);
  this->data_["type"] = "COMMAND";
}

MotorPlugin::~MotorPlugin()
{
}

void MotorPlugin::cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg)
{
  ROS_INFO_STREAM("Heard:" << cmd_vel_msg);
  this->data_["data"][0]["target"] = "WHEEL_LEFT";
  this->data_["data"][0]["value"] = 0;

  this->data_["data"][1]["target"] = "WHEEL_RIGHT";
  this->data_["data"][1]["value"] = 0;

  ROS_INFO_STREAM(this->data_.dump());
}

}  // namespace ahhaa_oskar