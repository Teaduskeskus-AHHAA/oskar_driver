#include <oskar_driver/odom_plugin.h>
#include <oskar_driver/oskar_commands.h>

namespace ahhaa_oskar
{
OdomPlugin::OdomPlugin(BoardComms* comms, std::string name) : Plugin(comms, name)
{
  setFrameId("odom");
  setChildFrameId("base_link");

  this->base_width_ = 0.600;

  x = 0;
  th = 0;

  reset();

  last_time_ = ros::Time::now();

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

float OdomPlugin::calc_speed_inverse(int32_t speed, float theta, bool left = false)
{
  int divider = left ? -2 : 2;
  float mps = (((speed / 6) / 57.29578) * 0.084);
  float vel = mps - ((this->base_width_ * theta) / divider);
  return vel;
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
  x = 0;
  th = 0;

  odom_msg_.header.stamp = ros::Time::now();
  odom_msg_.pose.pose.position.x = 0;
  odom_msg_.pose.pose.position.y = 0;
  odom_msg_.pose.pose.position.z = 0;
  odom_msg_.pose.pose.orientation.x = 0;
  odom_msg_.pose.pose.orientation.y = 0;
  odom_msg_.pose.pose.orientation.z = 0;
  odom_msg_.pose.pose.orientation.w = 1;

  odom_transform_.header.stamp = odom_msg_.header.stamp;
  odom_transform_.transform.translation.x = 0.0;
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
    ros::Time current_time = ros::Time::now();

    double dt = (current_time - last_time_).toSec();

    int32_t speed_left = (int32_t)((int32_t)packet.data[3] << 24) | ((int32_t)packet.data[2] << 16) |
                         ((int32_t)packet.data[1] << 8) | (packet.data[0]);
    int32_t speed_right = (int32_t)((int32_t)packet.data[7] << 24) | ((int32_t)packet.data[6] << 16) |
                          ((int32_t)packet.data[5] << 8) | (packet.data[4]);

    float theta = (speed_right / 6 - speed_left / 6) / base_width_;

    float v_left = calc_speed_inverse(speed_left, theta, true) * dt;
    float v_right = calc_speed_inverse(speed_right, theta) * dt;

    float v_wx = ((v_right + v_left) / 2) * cos(theta) - 0 * sin(theta);

    x += v_wx;
    th += theta * dt;

    // ROS_INFO("%f %f %f %f", v_left, v_right, theta, dt);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    odom_msg_.header.stamp = current_time;
    odom_msg_.pose.pose.position.x = x;
    odom_msg_.pose.pose.position.y = 0;
    odom_msg_.pose.pose.position.z = 0;
    odom_msg_.pose.pose.orientation = odom_quat;

    odom_msg_.twist.twist.linear.x = v_wx;
    odom_msg_.twist.twist.angular.z = theta;

    odom_transform_.header.stamp = odom_msg_.header.stamp;
    odom_transform_.transform.translation.x = x;
    odom_transform_.transform.translation.y = 0;
    odom_transform_.transform.translation.z = 0;
    odom_transform_.transform.rotation = odom_quat;

    if (odom_pub_.getNumSubscribers() > 0)
    {
      publish();
      last_time_ = current_time;
    }
  }
}

void OdomPlugin::publish()
{
  odom_pub_.publish(odom_msg_);
  odom_broadcaster_.sendTransform(odom_transform_);
}

}  // namespace ahhaa_oskar
