#include <oskar_driver/board_comms.h>
#include <oskar_driver/driver.h>
#include <oskar_driver/motor_plugin.h>

namespace ahhaa_oskar
{
Driver::Driver()
{
  ROS_INFO("Initializing Oskar-III driver");

  timer_ = nh_.createTimer(ros::Duration(0.01), &Driver::update, this);

  this->bc_ = new BoardComms("/dev/ttyUSB0", 9600);
  this->plugins_.emplace_back(std::make_shared<MotorPlugin>(this->bc_, "MotorPlugin"));
}

Driver::~Driver()
{
  ROS_INFO("Killing Oskar-III driver");
}

void Driver::update(const ros::TimerEvent &event)
{
  // ROS_INFO("Oskar III Update");
}
}  // namespace ahhaa_oskar
