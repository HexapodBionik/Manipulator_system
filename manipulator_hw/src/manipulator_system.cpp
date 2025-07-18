#include "manipulator_hw/manipulator_system.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <sstream>

namespace manipulator_hw
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn ManipulatorSystem::on_init(const hardware_interface::HardwareInfo &info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    joint_names_[i] = info.joints[i].name;
    pos_[i] = 0.0;
    cmd_[i] = 0.0;
    vel_[i] = 0.0;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ManipulatorSystem::on_activate(const rclcpp_lifecycle::State &) { return CallbackReturn::SUCCESS; }
CallbackReturn ManipulatorSystem::on_deactivate(const rclcpp_lifecycle::State &) { return CallbackReturn::SUCCESS; }

std::vector<hardware_interface::StateInterface> ManipulatorSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state;
  for (size_t i = 0; i < NUM_JOINTS; ++i)
    state.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_[i]);
  return state;
}

std::vector<hardware_interface::CommandInterface> ManipulatorSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmd;
  for (size_t i = 0; i < NUM_JOINTS; ++i)
    cmd.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &cmd_[i]);
  return cmd;
}

return_type ManipulatorSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  char buf[128] = {};
  int n = read(fd_, buf, sizeof(buf));
  if (n > 0) {
    buffer_ += std::string(buf, n);
    auto pos_nl = buffer_.find("\n");
    if (pos_nl != std::string::npos) {
      std::string line = buffer_.substr(0, pos_nl);
      buffer_.erase(0, pos_nl+1);
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream ss(line);
      double j1,j2,j3; int g;
      if (ss >> j1 >> j2 >> j3 >> g) {
        pos_[0]=j1; pos_[1]=j2; pos_[2]=j3;
        pos_[3]=cmd_[3];
        pos_[4]=pos_[5]=cmd_[4];
      }
    }
  }
  return return_type::OK;
}

return_type ManipulatorSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  char out[128] = {};
  snprintf(out, sizeof(out), "%.3f,%.3f,%.3f,%.3f,%.3f\n", cmd_[0], cmd_[1], cmd_[2], cmd_[3], cmd_[4]);
  write(fd_, out, strlen(out));
  return return_type::OK;
}

bool ManipulatorSystem::configure_serial(const std::string &port, int baud)
{
  fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) return false;
  termios tty{};
  tcgetattr(fd_, &tty);
  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tcsetattr(fd_, TCSANOW, &tty);
  return true;
}

}  // namespace manipulator_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(manipulator_hw::ManipulatorSystem, hardware_interface::SystemInterface)
