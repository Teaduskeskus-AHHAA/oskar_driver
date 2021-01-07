#ifndef DRIVER_H
#define DRIVER_H
#include <oskar_driver/board_comms.h>
#include <oskar_driver/motor_plugin.h>
#include <oskar_driver/plugin.h>
#include <ros/ros.h>
#include <vector>

namespace ahhaa_oskar
{
class Driver
{
public:
  Driver();
  ~Driver();

private:
  void update(const ros::TimerEvent& event);

  std::vector<PluginPtr> plugins_;

  ros::Timer timer_;
  ros::NodeHandle nh_;

  BoardComms* bc_;
};

}  // namespace ahhaa_oskar

#endif