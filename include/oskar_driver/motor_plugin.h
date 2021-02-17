#ifndef MOTORPLUGIN_H
#define MOTORPLUGIN_H
#include <geometry_msgs/Twist.h>
#include <oskar_driver/plugin.h>
#include <ros/ros.h>
#include <vector>

#define LEFT_MOTOR 0x4c
#define RIGHT_MOTOR 0x52
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
  float base_width_;
  OskarPacket packet;
  std::vector<uint8_t> data;
  int32_t calc_speed(const geometry_msgs::Twist& cmd_vel_msg, bool left);
  void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg);
};
}  // namespace ahhaa_oskar

#endif
