#include "manipulator_hw/serial_comm.hpp"
#include <iostream>
#include <sstream>
#include <algorithm> 

SerialComm::SerialComm()
{
}

SerialComm::~SerialComm()
{
  if (serial_.IsOpen())
  {
    serial_.Close();
  }
}

bool SerialComm::connect(const std::string &port, unsigned int baud_rate)
{
  try
  {
    serial_.Open(port);
    serial_.SetBaudRate(LibSerial::BaudRate(baud_rate));
    serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
  }
  catch (const LibSerial::OpenFailed&)
  {
    std::cerr << "Failed to open serial port: " << port << std::endl;
    return false;
  }

  return true;
}

bool SerialComm::read_positions(std::array<double, 6> &positions)
{
  std::string line;

  try
  {
    serial_.ReadLine(line, '\n');
  }
  catch (const LibSerial::ReadTimeout&)
  {
    std::cerr << "Read timeout." << std::endl;
    return false;
  }

  std::replace(line.begin(), line.end(), ',', ' ');
  std::istringstream iss(line);

  for (size_t i = 0; i < positions.size(); ++i)
  {
    if (!(iss >> positions[i]))
    {
      std::cerr << "Failed to parse position " << i << " from line: " << line << std::endl;
      return false;
    }
  }

  return true;
}

bool SerialComm::send_commands(const std::array<double, 6> &commands)
{
  std::ostringstream oss;
  for (size_t i = 0; i < commands.size(); ++i)
  {
    oss << commands[i];
    if (i != commands.size() - 1)
      oss << ",";
  }
  oss << "\n";

  try
  {
    serial_.Write(oss.str());
  }
  catch (const LibSerial::NotOpen&)
  {
    std::cerr << "Serial port not open during write." << std::endl;
    return false;
  }

  return true;
}

void SerialComm::disconnect()
{
  if (serial_.IsOpen()) {
    serial_.Close();
  }
}

