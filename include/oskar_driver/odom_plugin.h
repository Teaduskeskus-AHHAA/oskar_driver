#ifndef ODOMPLUGIN_H
#define ODOMPLUGIN_H
#include <nav_msgs/Odometry.h>
#include <oskar_driver/plugin.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
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
  void reset();
  void setFrameId(const std::string& frame_id);

  void setChildFrameId(const std::string& child_frame_id);
  void publish();

  void fake_odom(const ros::TimerEvent& event);
  ros::Timer timer_;

  nav_msgs::Odometry odom_msg_;
  geometry_msgs::TransformStamped odom_transform_;

  ros::Publisher odom_pub_;
  tf::TransformBroadcaster odom_broadcaster_;

  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
  float base_width_;
  double x;
  double th;
  double y;

  double wheel_diam_m_;
  double wheel_gear_ratio_;
  double wheel_dist_m_;

  ros::Time last_time_;
};
}  // namespace ahhaa_oskar

#endif
