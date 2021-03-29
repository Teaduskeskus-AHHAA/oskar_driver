#include <oskar_driver/odom_plugin.h>
#include <oskar_driver/oskar_commands.h>

namespace ahhaa_oskar
{
OdomPlugin::OdomPlugin(BoardComms* comms, std::string name) : Plugin(comms, name)
{
  setFrameId("odom");
  setChildFrameId("base_footprint");

  this->base_width_ = 0.600;

  reset();

  // Initialize odom publisher
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 2);
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

void OdomPlugin::setFrameId(const std::string& frame_id)
{
  odom_msg_.header.frame_id = frame_id;
  odom_transform_.header.frame_id = frame_id;
}

void OdomPlugin::setChildFrameId(const std::string& child_frame_id)
{
  odom_msg_.child_frame_id = child_frame_id;
  odom_transform_.child_frame_id = child_frame_id;
}

void OdomPlugin::reset()
{
  odom_msg_.header.stamp = ros::Time::now();
  odom_msg_.pose.pose.position.x = 0;
  odom_msg_.pose.pose.position.y = 0;
  odom_msg_.pose.pose.position.z = 0;
  odom_msg_.pose.pose.orientation.x = 0;
  odom_msg_.pose.pose.orientation.y = 0;
  odom_msg_.pose.pose.orientation.z = 0;
  odom_msg_.pose.pose.orientation.w = 1;

  odom_transform_.header.stamp = odom_msg_.header.stamp;
  odom_transform_.transform.translation.x = 0;
  odom_transform_.transform.translation.y = 0;
  odom_transform_.transform.translation.z = 0;
  odom_transform_.transform.rotation.x = 0;
  odom_transform_.transform.rotation.y = 0;
  odom_transform_.transform.rotation.z = 0;
  odom_transform_.transform.rotation.w = 0;
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
    publish();
  }
}

void OdomPlugin::publish()
{
  odom_pub_.publish(odom_msg_);
  odom_broadcaster_.sendTransform(odom_transform_);
}

}  // namespace ahhaa_oskar