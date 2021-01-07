#ifndef MOTORPLUGIN_H
#define MOTORPLUGIN_H
#include <oskar_driver/plugin.h>
#include <ros/ros.h>

namespace ahhaa_oskar
{
class MotorPlugin : public Plugin
{
public:
  MotorPlugin(BoardComms* comms, std::string name);
  ~MotorPlugin();
};
}  // namespace ahhaa_oskar

#endif