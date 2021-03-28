#include <oskar_driver/odom_plugin.h>
#include <oskar_driver/oskar_commands.h>

namespace ahhaa_oskar
{
OdomPlugin::OdomPlugin(BoardComms* comms, std::string name) : Plugin(comms, name)
{
  this->base_width_ = 0.600;
}

OdomPlugin::~OdomPlugin()
{
}

int32_t OdomPlugin::calc_speed(const geometry_msgs::Twist& cmd_vel_msg, bool left = false)
{
  int divider = left ? -2 : 2;
  float mps = (cmd_vel_msg.linear.x + ((this->base_width_ / divider) * cmd_vel_msg.angular.z));
  float radps = mps / 0.084;
  float dps = radps * 57.29578;
  int32_t speed = dps * 6;
  return speed;
}

/*void OdomPlugin::cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg)
{
  OskarPacket packet;
  packet.setCommand(DRIVESPEEDS_COMMAND);

  int32_t left_speed = this->calc_speed(cmd_vel_msg, true);
  int32_t right_speed = this->calc_speed(cmd_vel_msg);
  this->data.clear();
  this->data.push_back(LEFT_Odom);
  this->data.push_back(left_speed & 0xFF);
  this->data.push_back((left_speed >> 8) & 0xFF);
  this->data.push_back((left_speed >> 16) & 0xFF);
  this->data.push_back((left_speed >> 24) & 0xFF);
  this->data.push_back(RIGHT_Odom);
  this->data.push_back(right_speed & 0xFF);
  this->data.push_back((right_speed >> 8) & 0xFF);
  this->data.push_back((right_speed >> 16) & 0xFF);
  this->data.push_back((right_speed >> 24) & 0xFF);

  packet.setData(this->data);
  packet.encapsulate();
  this->comms_->send(packet);
}
*/
void OdomPlugin::processPacket(OskarPacket packet)
{
  // ROS_INFO("Odom has at least a packet w command %x", packet.getCommand());

  if (packet.getCommand() == ODOM_COMMAND)
  {
    ROS_INFO_STREAM("ODOM PLUGIN GOT ODOM PACKET");
  }
}

}  // namespace ahhaa_oskar
