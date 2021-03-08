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

uint16_t OskarPacket::calcCRC(uint16_t crc, uint8_t data)
{
  data ^= (crc)&0xff;
  data ^= data << 4;

  return ((((uint16_t)data << 8) | (crc) >> 8) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
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

  uint16_t crc = 0, i;
  for (i = 0; i < esc_data.size() / sizeof(esc_data[0]); i++)
  {
    crc = calcCRC(crc, esc_data[i]);
  }
  this->encapsulated_frame.push_back(crc & 0xFF);
  this->encapsulated_frame.push_back(crc >> 8);
  ROS_INFO("crc %x", crc);

  this->encapsulated_frame.push_back(END);

}

std::vector<uint8_t> OskarPacket::getEncapsulatedFrame()
{
  return this->encapsulated_frame;
}

void reconstruct(std::vector<uint8_t> data) {
  
}

}  // namespace ahhaa_oskar
