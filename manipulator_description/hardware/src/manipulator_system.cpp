// #include "manipulator_hw/manipulator_system.hpp"
// #include "manipulator_hw/serial_comm.hpp"  // Klasa do komunikacji UART
// #include "pluginlib/class_list_macros.hpp"
// #include <rclcpp/rclcpp.hpp>

// namespace manipulator_hw
// {

// using hardware_interface::CallbackReturn;
// using hardware_interface::return_type;

// CallbackReturn ManipulatorSystem::on_init(const hardware_interface::HardwareInfo &info)
// {
//   RCLCPP_INFO(rclcpp::get_logger("ManipulatorSystem"), "on_init called");

//   if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
//     return CallbackReturn::ERROR;

//   for (size_t i = 0; i < NUM_JOINTS; ++i) {
//     joint_names_[i] = info.joints[i].name;
//     pos_[i] = 0.0;
//     vel_[i] = 0.0;
//     cmd_[i] = 0.0;
//   }

//   return CallbackReturn::SUCCESS;
// }

// CallbackReturn ManipulatorSystem::on_configure(const rclcpp_lifecycle::State &)
// {
//   if (!serial_comm_.connect("/dev/ttyAMA0", 115200)) {
//     RCLCPP_ERROR(rclcpp::get_logger("ManipulatorSystem"), "Failed to open serial port");
//     return CallbackReturn::ERROR;
//   }

//   RCLCPP_INFO(rclcpp::get_logger("ManipulatorSystem"), "Serial connected successfully");
//   return CallbackReturn::SUCCESS;
// }

// CallbackReturn ManipulatorSystem::on_activate(const rclcpp_lifecycle::State &)
// {
//   RCLCPP_INFO(rclcpp::get_logger("ManipulatorSystem"), "System activated");
//   return CallbackReturn::SUCCESS;
// }

// CallbackReturn ManipulatorSystem::on_deactivate(const rclcpp_lifecycle::State &)
// {
//   RCLCPP_INFO(rclcpp::get_logger("ManipulatorSystem"), "System deactivated");
//   return CallbackReturn::SUCCESS;
// }

// CallbackReturn ManipulatorSystem::on_cleanup(const rclcpp_lifecycle::State &)
// {
//   serial_comm_.disconnect();
//   RCLCPP_INFO(rclcpp::get_logger("ManipulatorSystem"), "Serial disconnected");
//   return CallbackReturn::SUCCESS;
// }

// std::vector<hardware_interface::StateInterface> ManipulatorSystem::export_state_interfaces()
// {
//   std::vector<hardware_interface::StateInterface> state;
//   for (size_t i = 0; i < NUM_JOINTS; ++i) {
//     state.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_[i]);
//     state.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_[i]);
//   }
//   return state;
// }

// std::vector<hardware_interface::CommandInterface> ManipulatorSystem::export_command_interfaces()
// {
//   std::vector<hardware_interface::CommandInterface> cmd;
//   for (size_t i = 0; i < NUM_JOINTS; ++i) {
//     cmd.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &cmd_[i]);
//   }
//   return cmd;
// }

// return_type ManipulatorSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
// {
//   std::array<double, NUM_JOINTS> new_positions;
//   if (serial_comm_.read_positions(new_positions)) {
//     pos_ = new_positions;
//     RCLCPP_DEBUG(rclcpp::get_logger("ManipulatorSystem"), "Read positions from serial");
//   }
//   return return_type::OK;
// }

// return_type ManipulatorSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
// {
//   serial_comm_.send_commands(cmd_);
//   RCLCPP_DEBUG(rclcpp::get_logger("ManipulatorSystem"), "Sent commands to serial");
//   return return_type::OK;
// }

// }  // namespace manipulator_hw

// PLUGINLIB_EXPORT_CLASS(manipulator_hw::ManipulatorSystem, hardware_interface::SystemInterface)











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
 if (info.joints.size() != NUM_JOINTS) {
  RCLCPP_ERROR(rclcpp::get_logger("ManipulatorSystem"),
               "Wrong number of joints in URDF/hardware info. Expected: %zu, got: %zu",
               NUM_JOINTS, info.joints.size());
  return CallbackReturn::ERROR;
}

  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    RCLCPP_INFO(rclcpp::get_logger("ManipulatorSystem"), "Joint %zu: %s", i, info.joints[i].name.c_str());
    joint_names_[i] = info.joints[i].name;
    pos_[i] = 0.0;
    cmd_[i] = 0.0;
    vel_[i] = 0.0;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ManipulatorSystem::on_configure(const rclcpp_lifecycle::State &)
{
  if (!configure_serial("/dev/ttyAMA0", 115200)) {
    RCLCPP_ERROR(rclcpp::get_logger("ManipulatorSystem"), "Failed to open serial port");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}




CallbackReturn ManipulatorSystem::on_activate(const rclcpp_lifecycle::State &) { return CallbackReturn::SUCCESS; }
CallbackReturn ManipulatorSystem::on_deactivate(const rclcpp_lifecycle::State &) { return CallbackReturn::SUCCESS; }

std::vector<hardware_interface::StateInterface> ManipulatorSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    state.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_[i]);
    state.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_[i]);
  }
  return state;
}


std::vector<hardware_interface::CommandInterface> ManipulatorSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmd;
  for (size_t i = 0; i < NUM_JOINTS; ++i)
    cmd.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &cmd_[i]);
  return cmd;
}

// return_type ManipulatorSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
// {
//   char buf[512] = {};
//   int n = ::read(fd_, buf, sizeof(buf));
//   if (n > 0) {
//     buffer_ += std::string(buf, n);
//     auto pos_nl = buffer_.find("\n");
//     if (pos_nl != std::string::npos) {
//       std::string line = buffer_.substr(0, pos_nl);
//       buffer_.erase(0, pos_nl+1);
//       std::replace(line.begin(), line.end(), ',', ' ');
//       std::istringstream ss(line);
//       double j1,j2,j3; int g;
//       if (ss >> j1 >> j2 >> j3 >> g) {
//         RCLCPP_INFO(rclcpp::get_logger("ManipulatorSystem"), "Received: j1=%.3f j2=%.3f j3=%.3f g=%d", j1, j2, j3, g);

//         pos_[0]=j1; pos_[1]=j2; pos_[2]=j3;
//         // pos_[0]=cmd_[0]; pos_[1]=cmd_[1]; pos_[2]=cmd_[3];
        
//         pos_[3]=cmd_[3];
//         pos_[4]=pos_[5]=cmd_[4];
//       }
//     }
//   }
//   else {
//     RCLCPP_WARN(rclcpp::get_logger("ManipulatorSystem"), "No data from serial - injecting dummy values");
//     for (size_t i = 0; i < NUM_JOINTS; ++i) {
//       pos_[i] = cmd_[i]; // lub np. pos_[i] = 0.0;
//       vel_[i] = 0.0;
//     }
//   }
//   return return_type::OK;
// }

return_type ManipulatorSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  char buf[256] = {};
  int n = ::read(fd_, buf, sizeof(buf));
  if (n > 0) {
    buffer_ += std::string(buf, n);

    while (true) {
      size_t start = buffer_.find('<');
      size_t end = buffer_.find('>', start);

      if (start != std::string::npos && end != std::string::npos && end > start) {
        std::string frame = buffer_.substr(start + 1, end - start - 1);
        buffer_.erase(0, end + 1);  // usuń przetworzoną ramkę

        std::replace(frame.begin(), frame.end(), ',', ' ');
        std::istringstream ss(frame);
        double j1, j2, j3;
        int g;

        if (ss >> j1 >> j2 >> j3 >> g) {
          RCLCPP_INFO(rclcpp::get_logger("ManipulatorSystem"), 
                      "Received: j1=%.3f j2=%.3f j3=%.3f g=%d",
                      j1, j2, j3, g);

        pos_[0]=j1; pos_[1]=j2; pos_[2]=j3;
        pos_[3]=cmd_[3];
        pos_[4]=pos_[5]=cmd_[4];
        }
      } else {
        // brak kompletnej ramki, poczekaj na więcej danych
        break;
      }
    }
  } else {
    RCLCPP_WARN(rclcpp::get_logger("ManipulatorSystem"), "No data from serial - injecting dummy values");
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      pos_[i] = cmd_[i];
      vel_[i] = 0.0;
    }
  }

  return return_type::OK;
}


return_type ManipulatorSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  char out[128];
  int gripper_state = 0;  // dostosuj w razie potrzeby

  snprintf(out, sizeof(out), "<%.3f,%.3f,%.3f,%.3f,%d>", 
           cmd_[0], cmd_[1], cmd_[2], cmd_[3], gripper_state);

  RCLCPP_INFO(rclcpp::get_logger("ManipulatorSystem"), "Sent: %s", out);

  ::write(fd_, out, strlen(out));
  return return_type::OK;
}


bool ManipulatorSystem::configure_serial(const std::string &port, int baud)
{
  fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ManipulatorSystem"), "Failed to open serial port");
    return false;
}
RCLCPP_INFO(rclcpp::get_logger("ManipulatorSystem"), "Serial port opened: %s", port.c_str());

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
