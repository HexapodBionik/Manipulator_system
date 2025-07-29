// #pragma once

// #include "hardware_interface/system_interface.hpp"
// #include "hardware_interface/types/hardware_interface_type_values.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "manipulator_hw/serial_comm.hpp"

// #include <vector>
// #include <array>
// #include <string>

// namespace manipulator_hw
// {

// class ManipulatorSystem : public hardware_interface::SystemInterface
// {
// public:
//   CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
//   CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
//   CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
//   CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
//   CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

//   std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
//   std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

//   hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
//   hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

// private:
//   static constexpr size_t NUM_JOINTS = 6;
//   std::array<double, NUM_JOINTS> pos_{};
//   std::array<double, NUM_JOINTS> vel_{};
//   std::array<double, NUM_JOINTS> cmd_{};
//   std::array<std::string, NUM_JOINTS> joint_names_{};

//   SerialComm serial_comm_;  // obsługuje komunikację UART
// };

// }  // namespace manipulator_hw






#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <array>

namespace manipulator_hw
{

class ManipulatorSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr size_t NUM_JOINTS = 6;
  std::array<double, NUM_JOINTS> pos_{};
  std::array<double, NUM_JOINTS> vel_{};
  std::array<double, NUM_JOINTS> cmd_{};
  std::array<std::string, NUM_JOINTS> joint_names_{};
  int fd_ = -1;
  std::string buffer_;
  bool configure_serial(const std::string &port, int baud);
};

}  // namespace manipulator_hw
