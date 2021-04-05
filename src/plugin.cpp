#include <oskar_driver/plugin.h>

namespace ahhaa_oskar
{
Plugin::Plugin(BoardComms *comms, std::string name) : name_(name), comms_(comms)
{
  ROS_INFO("Created plugin %s\n", name.c_str());
  this->nh_.getParam("phys/wheel_diam_m", wheel_diam_m_);
  this->nh_.getParam("phys/wheel_dist_m", wheel_dist_m_);
  this->nh_.getParam("phys/wheel_gear_ratio", wheel_gear_ratio_);
}
Plugin::~Plugin()
{
}

std::string Plugin::getName()
{
  return this->name_;
}

void Plugin::processPacket(OskarPacket packet)
{
}

}  // namespace ahhaa_oskar