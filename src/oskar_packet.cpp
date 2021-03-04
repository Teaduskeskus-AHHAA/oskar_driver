#include <oskar_driver/oskar_packet.h>
#include <ros/ros.h>

namespace ahhaa_oskar
{
OskarPacket::OskarPacket()
{
}

OskarPacket::~OskarPacket()
{
}

void OskarPacket::setCommand(uint8_t command)
{
  this->command = command;
}

void OskarPacket::setData(std::vector<uint8_t> data)
{
  this->data = data;
}

uint8_t OskarPacket::getCommand()
{
  return this->command;
}

std::vector<uint8_t> OskarPacket::getEscapedData()
{
  std::vector<uint8_t> result;
  result.clear();

  for (uint8_t i = 0; i < this->data.size(); i++)
  {
    if (this->data[i] == END)
    {
      result.push_back(ESC);
      result.push_back(ESC_END);
    }
    else if (this->data[i] == 0xDB)
    {
      result.push_back(ESC);
      result.push_back(ESC_ESC);
    }
    else
    {
      result.push_back(this->data[i]);
    }
  }

  return result;
}

void OskarPacket::encapsulate()
{
  std::vector<uint8_t> esc_data = this->getEscapedData();
  this->encapsulated_frame.clear();
  this->encapsulated_frame.push_back(END);
  this->encapsulated_frame.push_back(esc_data.size() + 1);
  this->encapsulated_frame.push_back(this->command);
  this->encapsulated_frame.insert(this->encapsulated_frame.end(), esc_data.begin(), esc_data.end());
  this->encapsulated_frame.push_back(END);
}

std::vector<uint8_t> OskarPacket::getEncapsulatedFrame()
{
  return this->encapsulated_frame;
}

}  // namespace ahhaa_oskar
