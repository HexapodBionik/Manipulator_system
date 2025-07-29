#pragma once

#include <libserial/SerialPort.h>
#include <array>
#include <string>

class SerialComm
{
public:
  SerialComm();                          // brakująca deklaracja
  ~SerialComm();                         // brakująca deklaracja

  bool connect(const std::string &port, unsigned int baud_rate);
  bool read_positions(std::array<double, 6> &positions);
  bool send_commands(const std::array<double, 6> &commands);  // brakująca deklaracja
  void disconnect();


private:
  LibSerial::SerialPort serial_;
};
