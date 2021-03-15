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
    else if (this->data[i] == ESC)
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

std::vector<uint8_t> OskarPacket::getUnescapedData(std::vector<uint8_t> escaped_data)
{
  std::vector<uint8_t> result;

  uint8_t i = 0;
  while (i < escaped_data.size())
  {
    if (escaped_data.at(i) != ESC)
    {
      result.push_back(escaped_data.at(i));
      i++;
    }
    else
    {
      if (escaped_data.at(i + 1) == ESC_END)
      {
        result.push_back(END);
      }
      else if (escaped_data.at(i + 1) == ESC_ESC)
      {
        result.push_back(ESC);
      }
      i += 2;
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
  for (uint8_t i = 0; i < esc_data.size(); i++)
  {
    ROS_INFO("0x%x", esc_data.at(i));
  }
  uint16_t crc = 0, i;
  for (i = 0; i < esc_data.size() / sizeof(esc_data[0]); i++)
  {
    crc = calcCRC(crc, esc_data[i]);
  }
  this->encapsulated_frame.push_back(crc & 0xFF);
  this->encapsulated_frame.push_back(crc >> 8);
  this->encapsulated_frame.push_back(END);
}

std::vector<uint8_t> OskarPacket::getEncapsulatedFrame()
{
  return this->encapsulated_frame;
}

void OskarPacket::reconstruct(std::vector<uint8_t> data)
{
  if ((data.at(0) == END) && (data.at(data.size() - 1) == END))
  {
    uint16_t crc_read = data.at(data.size() - 3) | (data.at(data.size() - 2) << 8);
    uint16_t crc = 0, i;
    for (i = 4; i < (data.size() / sizeof(data[0])) - 3; i++)
    {
      crc = calcCRC(crc, data[i]);
    }
    if (crc == crc_read)
    {
      this->setCommand(data.at(2));
      std::vector<uint8_t> encapsulated_escaped_data;
      encapsulated_escaped_data.insert(encapsulated_escaped_data.end(), &data.at(3), &data.at(3 + (data.at(1) - 1)));
      for (uint8_t i = 0; i < encapsulated_escaped_data.size(); i++)
      {
        ROS_INFO("0x%x", encapsulated_escaped_data.at(i));
      }
    }
  }
}

}  // namespace ahhaa_oskar
