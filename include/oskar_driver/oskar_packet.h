#ifndef OSKAR_PACKET_H
#define OSKAR_PACKET_H

#include <stdint.h>
#include <vector>

#define END 0xC0
#define ESC 0xDB
#define ESC_END 0xDC
#define ESC_ESC 0xDD

/*  OskarPacket - a binary packet encapsulation for ROS<->Robot comms.
    Somewhat inspired and based on the encapsulation of the Serial Line Internet Protocol -SLIP in egards to signaling
   start and end of packet https://en.wikipedia.org/wiki/Serial_Line_Internet_Protocol#Description

*/

namespace ahhaa_oskar
{
class OskarPacket
{
public:
  OskarPacket();
  ~OskarPacket();
  void encapsulate();
  void setCommand(uint8_t command);
  void setData(std::vector<uint8_t> data);
  uint8_t getCommand();
  std::vector<uint8_t> getEscapedData();
  std::vector<uint8_t> getEncapsulatedFrame();
  uint16_t calcCRC(uint16_t crc, uint8_t data);

private:
  uint8_t command;
  std::vector<uint8_t> data;
  uint16_t crc16;
  std::vector<uint8_t> encapsulated_frame;
};
}  // namespace ahhaa_oskar

#endif