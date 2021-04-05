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
  y = 0;
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
  // speed = left ? speed * -1 : speed;
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

void OdomPlugin::processPacket(OskarPacket packet)
{
  // ROS_INFO("Odom has at least a packet w command %x", packet.getCommand());

  if (packet.getCommand() == ODOM_COMMAND)
  {
    /*

    double vel_left_desired = (cmd_vel_msg.linear.x - cmd_vel_msg.angular.z * wheel_dist_m_ / 2.0) / (wheel_diam_m_ /
  2); double vel_right_desired = (cmd_vel_msg.linear.x + cmd_vel_msg.angular.z * wheel_dist_m_ / 2.0) / (wheel_diam_m_ /
  2);

  double rad_per_s_left = vel_left_desired / (wheel_diam_m_ / 2);
  double rad_per_s_right = vel_right_desired / (wheel_diam_m_ / 2);

  double degrees_per_s_left = (rad_per_s_left * 180) / M_PI;
  double degrees_per_s_right = (rad_per_s_right * 180) / M_PI;

  int32_t left_speed = degrees_per_s_left * wheel_gear_ratio_;
  int32_t right_speed = degrees_per_s_right * wheel_gear_ratio_;

    */

    ros::Time current_time = ros::Time::now();
    double delta_time = (current_time - last_time_).toSec();
    double dt = (current_time - last_time_).toSec();

    int32_t in_motorspeed_left = (int32_t)((int32_t)packet.data[3] << 24) | ((int32_t)packet.data[2] << 16) |
                                 ((int32_t)packet.data[1] << 8) | (packet.data[0]);
    int32_t in_motorspeed_right = (int32_t)((int32_t)packet.data[7] << 24) | ((int32_t)packet.data[6] << 16) |
                                  ((int32_t)packet.data[5] << 8) | (packet.data[4]);

    double degrees_per_s_left = -in_motorspeed_left / wheel_gear_ratio_;
    double degrees_per_s_right = in_motorspeed_right / wheel_gear_ratio_;

    double radians_per_s_left = (M_PI * degrees_per_s_left) / 180;
    double radians_per_s_right = (M_PI * degrees_per_s_right) / 180;

    double vel_left = (radians_per_s_left * wheel_diam_m_) / 2;
    double vel_right = (radians_per_s_right * wheel_diam_m_) / 2;

    double v_robot_x = (vel_right + vel_left) / 2;
    double v_robot_y = 0;
    double theta = (vel_right - vel_left) / wheel_dist_m_;

    double v_world_x = v_robot_x * cos(theta) - v_robot_y * sin(theta);
    double v_world_y = v_robot_x * sin(theta) + v_robot_y * cos(theta);

    x += v_world_x * dt;
    y += v_world_y * dt;
    th += theta * dt;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    odom_msg_.header.stamp = current_time;
    odom_msg_.pose.pose.position.x = x;
    odom_msg_.pose.pose.position.y = y;
    odom_msg_.pose.pose.position.z = 0;
    odom_msg_.pose.pose.orientation = odom_quat;

    odom_msg_.twist.twist.linear.x = v_robot_x;
    odom_msg_.twist.twist.angular.z = theta;

    odom_transform_.header.stamp = odom_msg_.header.stamp;
    odom_transform_.transform.translation.x = x;
    odom_transform_.transform.translation.y = y;
    odom_transform_.transform.translation.z = 0;
    odom_transform_.transform.rotation = odom_quat;

    ROS_INFO(" %f %f ", vel_left, vel_right);

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
