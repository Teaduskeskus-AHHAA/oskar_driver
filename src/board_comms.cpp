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

void BoardComms::reconnect(const ros::TimerEvent& event)
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

std::vector<OskarPacket> BoardComms::readPackets(OskarPacket packet, std::string temp_DBG)
{
  std::vector<OskarPacket> results;
  try
  {
    uint8_t byte_in[1];

    // size_t bytes_available = serial_.available();

    while (serial_.available())
    {
      serial_.read(byte_in, 1);
      if ((last_byte_in_ == END) && (*byte_in == END) & (data_read.at(0) == END))
      {
        awaiting_data.push_back(data_read);
        data_read.clear();
      }
      else if ((last_byte_in_ == END) && (*byte_in == END) & (data_read.at(0) != END))
      {
        data_read.clear();
      }

      data_read.push_back(*byte_in);
      last_byte_in_ = *byte_in;
    }

    /*   if (bytes_available)
       {
         for (int i = 0; i < bytes_available; i++)
         {
           serial_.read(byte_in, 1);


         }
       }*/

    // size_t bytes_available = serial_.available();
    //  if (bytes_available)
    //  {
    // serial_.read(data_read, bytes_available);

    /* for (int i = 0; i < data_read.size(); i++)
     {
       ROS_INFO("%x ", *byte_in);
     }*/

    /*   serial_.read(data_read, bytes_available);
       for (int i = 0; i < data_read.size(); i++)
       {
         ROS_INFO("%x ", data_read[i]);
       }*/
    //   ROS_INFO("-------------------------");
    //   data_read_buffer.insert(this->data_read_buffer.end(), data_read.begin(), data_read.end());
    // TODO: Implement a timeout for flushing data_read_buffer when it does not form a packet in X amount of time, or
    // implement method to slice out valid packages from it.
    // }
  }
  catch (serial::IOException e)
  {
    reconnect_requested_ = true;
    ROS_ERROR_STREAM("" << e.what());
  }
  catch (serial::SerialException e)
  {
    reconnect_requested_ = true;
    ROS_ERROR_STREAM("" << e.what());
  }
  catch (serial::PortNotOpenedException e)
  {
    reconnect_requested_ = true;
    ROS_ERROR_STREAM("" << e.what());
  }

  OskarPacket pct;
  while (awaiting_data.size() > 0)
  {
    ROS_INFO("AWAITING DATA IS %d", awaiting_data.size());
    pct.reconstruct(awaiting_data.at(0));
    awaiting_data.erase(awaiting_data.begin());
    results.push_back(pct);
  }

  /*  if ((data_read_buffer.size() > 1) && (data_read_buffer[0] == END) && (data_read[data_read.size() - 1] == END))
    {
      packet.reconstruct(data_read_buffer);
      data_read_buffer.clear();
      return true;
    }*/

  return results;
}  // namespace ahhaa_oskar

}  // namespace ahhaa_oskar
