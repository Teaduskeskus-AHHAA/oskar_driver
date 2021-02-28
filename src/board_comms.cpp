#include <oskar_driver/board_comms.h>

namespace ahhaa_oskar
{
BoardComms::BoardComms(std::string port_name, int baudrate)
{
  this->reconnect_requested_ = false;
  this->serial_.setPort(port_name);
  this->serial_.setBaudrate(baudrate);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  serial_.setTimeout(timeout);

  this->tryConnect();

  this->timer_ = nh_.createTimer(ros::Duration(.5), &BoardComms::reconnect, this);
}

BoardComms::~BoardComms()
{
  this->serial_.close();
  this->serial_.~Serial();
}

void BoardComms::tryConnect()
{
  if (this->serial_.isOpen())
  {
    this->serial_.close();
  }
  try
  {
    this->serial_.open();
    ROS_INFO("Serial port to board opened successfully");
    this->reconnect_requested_ = false;
  }
  catch (serial::IOException e)
  {
    ROS_ERROR_STREAM_THROTTLE(1, "Unable to open serial port '" << this->serial_.getPort() << "': " << e.what());
  }
  catch (serial::SerialException e)
  {
    ROS_ERROR_STREAM_THROTTLE(1, "Unable to open serial port '" << this->serial_.getPort() << "': " << e.what());
  }
}

void BoardComms::reconnect(const ros::TimerEvent &event)
{
  if (!serial_.isOpen() || this->reconnect_requested_)
  {
    this->reconnect_requested_ = true;
    ROS_INFO("Serial port attempting reconnection");
    tryConnect();
  }
}

void BoardComms::send(OskarPacket packet)
{
  if (!this->reconnect_requested_)
  {
    ROS_INFO_STREAM(packet.getEncapsulatedFrame().size());
    if (packet.getEncapsulatedFrame().size() == 0)
    {
      return;
    }
    else
    {
      try
      {
        serial_.write(packet.getEncapsulatedFrame());
      }
      catch (serial::SerialException e)
      {
        reconnect_requested_ = true;
        ROS_ERROR_STREAM_THROTTLE(1, e.what());
      }
    }
  }
}

}  // namespace ahhaa_oskar
