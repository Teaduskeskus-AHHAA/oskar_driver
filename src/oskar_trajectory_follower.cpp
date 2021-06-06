#include <oskar_driver/oskar_trajectory_follower.h>

namespace ahhaa_oskar
{
OskarTrajectoryFollower::OskarTrajectoryFollower(std::string name) : as_(nh_, name, false), action_name_(name)
{
  // Register callback functions:
  as_.registerGoalCallback(boost::bind(&OskarTrajectoryFollower::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&OskarTrajectoryFollower::preemptCB, this));

  as_.start();
}

OskarTrajectoryFollower::~OskarTrajectoryFollower()
{
}

void OskarTrajectoryFollower::goalCB()
{
  trajectory_ = as_.acceptNewGoal()->trajectory;
  ROS_INFO_STREAM("Accepted Goal");
}

void OskarTrajectoryFollower::preemptCB()
{
  ROS_INFO("%s: Preempted", action_name_.c_str());
  // set the action state to preempted
  as_.setPreempted();
}

}  // namespace ahhaa_oskar
