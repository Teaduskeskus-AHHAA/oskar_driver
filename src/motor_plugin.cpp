#include <oskar_driver/motor_plugin.h>

namespace ahhaa_oskar
{
MotorPlugin::MotorPlugin(BoardComms* comms, std::string name) : Plugin(comms, name)
{
  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &MotorPlugin::cmd_vel_callback, this);
  this->base_width_ = 0.600;
  this->data_["type"] = "COMMAND";
}

MotorPlugin::~MotorPlugin()
{
}

int32_t MotorPlugin::calc_speed(const geometry_msgs::Twist& cmd_vel_msg, bool left = false)
{
  int divider = left ? -2 : 2;
  float mps = (cmd_vel_msg.linear.x + ((this->base_width_ / divider) * cmd_vel_msg.angular.z));
  float radps = mps / 0.084;
  float dps = radps * 57.29578;
  int32_t speed = dps * 6;
  return speed;
}

void MotorPlugin::cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg)
{
  this->data_["data"][0]["target"] = "WHEEL_LEFT";
  this->data_["data"][0]["value"] = this->calc_speed(cmd_vel_msg, true);
  this->data_["data"][1]["target"] = "WHEEL_RIGHT";
  this->data_["data"][1]["value"] = this->calc_speed(cmd_vel_msg);

  ROS_INFO_STREAM("Writing out " << this->data_.dump());
  this->comms_->writeObject(this->data_);
}

}  // namespace ahhaa_oskar