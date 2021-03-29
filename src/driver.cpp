#include <oskar_driver/board_comms.h>
#include <oskar_driver/driver.h>
#include <oskar_driver/motor_plugin.h>
#include <oskar_driver/odom_plugin.h>

namespace ahhaa_oskar
{
Driver::Driver()
{
  ROS_INFO("Initializing Oskar-III driver");

  timer_ = nh_.createTimer(ros::Duration(0.01), &Driver::update, this);

  this->bc_ = new BoardComms("/dev/ttyUSB0", 9600);
  this->plugins_.emplace_back(std::make_shared<MotorPlugin>(this->bc_, "MotorPlugin"));
  this->plugins_.emplace_back(std::make_shared<OdomPlugin>(this->bc_, "OdomPlugin"));
}

Driver::~Driver()
{
  ROS_INFO("Killing Oskar-III driver");
}

void Driver::update(const ros::TimerEvent& event)
{
  OskarPacket dummy;
  std::vector<OskarPacket> incoming_packets;
  std::string dbg;

  incoming_packets = this->bc_->readPackets(dummy, dbg);
  for (int i = 0; i < incoming_packets.size(); i++)
  {
    for (auto& plugin : plugins_)
    {
      plugin->processPacket(incoming_packets.at(i));
    }
  }

  /* if (this->bc_->readPacket(packet, dbg))
   {
     for (auto& plugin : plugins_)
     {
       plugin->processPacket(packet);
     }
   }*/
  // ROS_INFO("Oskar III Update");
}
}  // namespace ahhaa_oskar
