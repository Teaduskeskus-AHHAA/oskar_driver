#ifndef BOARDCOMMS_H
#define BOARDCOMMS_H
#include <oskar_driver/oskar_packet.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <string.h>

namespace ahhaa_oskar
{
class BoardComms
{
public:
  BoardComms(std::string port_name, int baudrate);
  ~BoardComms();
  void send(OskarPacket packet);
  std::vector<OskarPacket> readPackets(OskarPacket packet, std::string temp_DBG);
  std::vector<uint8_t> data_read;

  std::vector<uint8_t> data_read_buffer;
  std::vector<std::vector<uint8_t>> awaiting_data;

private:
  void tryConnect();
  void reconnect(const ros::TimerEvent& event);
  serial::Serial serial_;
  ros::NodeHandle nh_;
  ros::Timer timer_;
  bool reconnect_requested_;
  uint8_t last_byte_in_;
};

}  // namespace ahhaa_oskar

#endif