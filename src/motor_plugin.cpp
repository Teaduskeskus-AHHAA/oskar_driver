#include <oskar_driver/motor_plugin.h>

namespace ahhaa_oskar
{
MotorPlugin::MotorPlugin(BoardComms* comms, std::string name) : Plugin(comms, name)
{
  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &MotorPlugin::cmd_vel_callback, this);
  this->base_width_ = 0.600;
  this->packet.setCommand(0x01);
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
  int32_t left_speed = this->calc_speed(cmd_vel_msg, true);
  int32_t right_speed = this->calc_speed(cmd_vel_msg);

  this->data.push_back(0x4c);
  this->data.push_back(left_speed & 0xFF);
  this->data.push_back((left_speed >> 8) & 0xFF);
  this->data.push_back((left_speed >> 16) & 0xFF);
  this->data.push_back((left_speed >> 24) & 0xFF);
  this->data.push_back(0x52);
  this->data.push_back(right_speed & 0xFF);
  this->data.push_back((right_speed >> 8) & 0xFF);
  this->data.push_back((right_speed >> 16) & 0xFF);
  this->data.push_back((right_speed >> 24) & 0xFF);

  this->packet.setData(this->data);
  this->packet.encapsulate();
  this->comms_->send(this->packet);
}

}  // namespace ahhaa_oskar