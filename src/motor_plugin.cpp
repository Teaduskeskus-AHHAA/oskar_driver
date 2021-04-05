#include <oskar_driver/motor_plugin.h>
#include <oskar_driver/oskar_commands.h>
namespace ahhaa_oskar
{
MotorPlugin::MotorPlugin(BoardComms* comms, std::string name) : Plugin(comms, name)
{
  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &MotorPlugin::cmd_vel_callback, this);
  this->nh_.getParam("phys/wheel_diam_m", wheel_diam_m_);
  this->nh_.getParam("phys/wheel_dist_m", wheel_dist_m_);
  this->nh_.getParam("phys/wheel_gear_ratio", wheel_gear_ratio_);
}

MotorPlugin::~MotorPlugin()
{
}

int32_t MotorPlugin::calc_speed(const geometry_msgs::Twist& cmd_vel_msg, bool left = false)
{
  int32_t speed = 0;

  return speed;
}

void MotorPlugin::cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg)
{
  OskarPacket packet;
  packet.setCommand(DRIVESPEEDS_COMMAND);

  double vel_left_desired = (cmd_vel_msg.linear.x - cmd_vel_msg.angular.z * wheel_dist_m_ / 2.0) / (wheel_diam_m_ / 2);
  double vel_right_desired = (cmd_vel_msg.linear.x + cmd_vel_msg.angular.z * wheel_dist_m_ / 2.0) / (wheel_diam_m_ / 2);

  double rad_per_s_left = vel_left_desired / (wheel_diam_m_ / 2);
  double rad_per_s_right = vel_right_desired / (wheel_diam_m_ / 2);

  double degrees_per_s_left = (rad_per_s_left * 180) / M_PI;
  double degrees_per_s_right = (rad_per_s_right * 180) / M_PI;

  int32_t left_speed = -(degrees_per_s_left * wheel_gear_ratio_);
  int32_t right_speed = degrees_per_s_right * wheel_gear_ratio_;

  this->data.clear();
  this->data.push_back(left_speed & 0xFF);
  this->data.push_back((left_speed >> 8) & 0xFF);
  this->data.push_back((left_speed >> 16) & 0xFF);
  this->data.push_back((left_speed >> 24) & 0xFF);
  this->data.push_back(right_speed & 0xFF);
  this->data.push_back((right_speed >> 8) & 0xFF);
  this->data.push_back((right_speed >> 16) & 0xFF);
  this->data.push_back((right_speed >> 24) & 0xFF);

  packet.setData(this->data);
  packet.encapsulate();
  this->comms_->send(packet);
}

void MotorPlugin::processPacket(OskarPacket packet)
{
}

}  // namespace ahhaa_oskar
