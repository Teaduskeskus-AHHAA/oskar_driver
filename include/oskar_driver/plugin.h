#ifndef PLUGIN_H
#define PLUGIN_H
#include <oskar_driver/board_comms.h>
#include <ros/ros.h>
#include <string.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace ahhaa_oskar
{
class Plugin
{
public:
  Plugin(BoardComms* comms, std::string name);
  ~Plugin();

  std::string getName();

private:
  std::string name_;
  ros::NodeHandle nh_;

protected:
  BoardComms* comms_;
  json data_;
};
typedef std::shared_ptr<Plugin> PluginPtr;
}  // namespace ahhaa_oskar

#endif