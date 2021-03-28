#ifndef ODOMPLUGIN_H
#define ODOMPLUGIN_H
#include <geometry_msgs/Twist.h>
#include <oskar_driver/plugin.h>
#include <ros/ros.h>
#include <vector>

namespace ahhaa_oskar
{
class OdomPlugin : public Plugin
{
public:
  OdomPlugin(BoardComms* comms, std::string name);
  ~OdomPlugin();
  void processPacket(OskarPacket packet) override;

private:
  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
  float base_width_;
  std::vector<uint8_t> data;
  int32_t calc_speed(const geometry_msgs::Twist& cmd_vel_msg, bool left);
  void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg);
};
}  // namespace ahhaa_oskar

#endif
