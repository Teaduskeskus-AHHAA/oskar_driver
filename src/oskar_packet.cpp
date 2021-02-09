#include <oskar_driver/oskar_packet.h>

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

void OskarPacket::encapsulate()
{
  this->encapsulated_frame.clear();
  this->encapsulated_frame.push_back(END);
  this->encapsulated_frame.push_back(this->data.size() + 1);
  this->encapsulated_frame.push_back(this->command);
  for (uint8_t i = 0; i < this->data.size(); i++)
  {
    if (this->data[i] == END)
    {
      this->encapsulated_frame.push_back(ESC);
      this->encapsulated_frame.push_back(ESC_END);
    }
    else if (this->data[i] == 0xDB)
    {
      this->encapsulated_frame.push_back(ESC);
      this->encapsulated_frame.push_back(ESC_ESC);
    }
    else
    {
      this->encapsulated_frame.push_back(this->data[i]);
    }
  }
  this->encapsulated_frame.push_back(END);
}

std::vector<uint8_t> OskarPacket::getEncapsulatedFrame()
{
  return this->encapsulated_frame;
}

}  // namespace ahhaa_oskar
