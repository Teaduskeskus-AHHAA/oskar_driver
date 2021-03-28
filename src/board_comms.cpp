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
    this->data_read_buffer.clear();
    ROS_INFO("Serial port attempting reconnection");
    tryConnect();
  }
}

void BoardComms::send(OskarPacket packet)
{
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
    catch (serial::IOException e)
    {
      reconnect_requested_ = true;
    }
    catch (serial::SerialException e)
    {
      reconnect_requested_ = true;
    }
    catch (serial::PortNotOpenedException)
    {
      reconnect_requested_ = true;
    }
  }
}

bool BoardComms::readPacket(OskarPacket packet, std::string temp_DBG)
{
  std::vector<uint8_t> data_read;

  try
  {
    size_t bytes_available = serial_.available();
    if (bytes_available)
    {
      serial_.read(data_read, bytes_available);
      for (int i = 0; i < data_read.size(); i++)
      {
        ROS_INFO("%x ", data_read[i]);
      }
      ROS_INFO("-------------------------");
      data_read_buffer.insert(this->data_read_buffer.end(), data_read.begin(), data_read.end());
      // TODO: Implement a timeout for flushing data_read_buffer when it does not form a packet in X amount of time, or
      // implement method to slice out valid packages from it.
    }
  }
  catch (serial::IOException e)
  {
    reconnect_requested_ = true;
  }
  catch (serial::SerialException e)
  {
    reconnect_requested_ = true;
  }
  catch (serial::PortNotOpenedException)
  {
    reconnect_requested_ = true;
  }
  ROS_INFO_STREAM("RBSIZE: " << data_read_buffer.size());

  if ((data_read_buffer.size() > 1) && (data_read_buffer[0] == END) && (data_read[data_read.size() - 1] == END))
  {
    packet.reconstruct(data_read_buffer);
    data_read_buffer.clear();
    return true;
  }

  return false;
}

}  // namespace ahhaa_oskar
