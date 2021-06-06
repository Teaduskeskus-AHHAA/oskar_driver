#ifndef OSKAR_TRAJECTORY_FOLLOWER_H
#define OSKAR_TRAJECTORY_FOLLOWER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace ahhaa_oskar
{
class OskarTrajectoryFollower
{
public:
  OskarTrajectoryFollower(std::string name);
  ~OskarTrajectoryFollower();
  void goalCB();
  void preemptCB();

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  std::string action_name_;
};
}  // namespace ahhaa_oskar

#endif