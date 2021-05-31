#include <oskar_driver/moveit_plugin.h>
#include <oskar_driver/oskar_commands.h>
namespace ahhaa_oskar
{
MoveitPlugin::MoveitPlugin(BoardComms* comms, std::string name) : Plugin(comms, name)
{
  execute_trajectory_sub_ =
      nh_.subscribe("move_group/execute_trajectory", 1, &MoveitPlugin::execute_trajectory_callback, this);
}

MoveitPlugin::~MoveitPlugin()
{
}

void MoveitPlugin::execute_trajectory_callback(const moveit_msgs::RobotTrajectory& trajectory_msg)
{
  ROS_INFO_STREAM("GOT trajectory_msg");
  ROS_INFO_STREAM(trajectory_msg);
}

}  // namespace ahhaa_oskar
