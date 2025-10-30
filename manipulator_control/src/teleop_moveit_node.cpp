#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>
#include <map>
#include <thread>
#include <sstream>

using moveit::planning_interface::MoveGroupInterface;

class TeleopMoveItNode : public rclcpp::Node {
public:
  TeleopMoveItNode() : Node("teleop_moveit_node") {
    arm_group_name_      = this->declare_parameter<std::string>("arm_group_name", "arm");
    gripper_group_name_  = this->declare_parameter<std::string>("gripper_group_name", "gripper");

    arm_joint_names_     = this->declare_parameter<std::vector<std::string>>(
      "arm_joint_names", std::vector<std::string>{"joint_1","joint_2","joint_3","joint_4"});

    gripper_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "gripper_joint_names", std::vector<std::string>{"lf_joint","rf_joint"});
    gripper_joint_name_  = this->declare_parameter<std::string>("gripper_joint_name", "gripper_joint");

    vel_scale_           = this->declare_parameter<double>("velocity_scale", 0.5);
    acc_scale_           = this->declare_parameter<double>("acceleration_scale", 0.5);
    gripper_open_        = this->declare_parameter<double>("gripper_open", 0.04);
    gripper_closed_      = this->declare_parameter<double>("gripper_closed", 0.0);
  }

  void init() {
    arm_group_     = std::make_shared<MoveGroupInterface>(shared_from_this(), arm_group_name_);
    gripper_group_ = std::make_shared<MoveGroupInterface>(shared_from_this(), gripper_group_name_);

    arm_group_->setMaxVelocityScalingFactor(vel_scale_);
    arm_group_->setMaxAccelerationScalingFactor(acc_scale_);
    gripper_group_->setMaxVelocityScalingFactor(vel_scale_);
    gripper_group_->setMaxAccelerationScalingFactor(acc_scale_);

    RCLCPP_INFO(get_logger(), "TeleopMoveIt gotowy. Grupy: arm='%s', gripper='%s'",
                arm_group_name_.c_str(), gripper_group_name_.c_str());


    std::thread([this]() { this->prompt_loop(); }).detach();
  }

private:
  void prompt_loop() {
    std::string line;
    print_help();
    while (rclcpp::ok()) {
      std::cout << "Podaj 4 kąty [rad] i chwytak (0/1), np.: 0.0 1.0 -0.5 0.2 1> " << std::flush;
      if (!std::getline(std::cin, line)) {
        RCLCPP_INFO(get_logger(), "Koniec wejścia — wychodzę.");
        rclcpp::shutdown();
        return;
      }
      if (line == "q" || line == "quit" || line == "exit") {
        rclcpp::shutdown();
        return;
      }
      std::stringstream ss(line);
      double j1, j2, j3, j4; int grip;
      if (!(ss >> j1 >> j2 >> j3 >> j4 >> grip)) {
        std::cout << "Niepoprawny format. Spróbuj ponownie." << std::endl;
        continue;
      }
      if (arm_joint_names_.size() != 4) {
        RCLCPP_ERROR(get_logger(), "arm_joint_names ma %zu pozycji, oczekiwano 4.", arm_joint_names_.size());
        continue;
      }

      std::map<std::string, double> target;
      target[arm_joint_names_[0]] = j1;
      target[arm_joint_names_[1]] = j2;
      target[arm_joint_names_[2]] = j3;
      target[arm_joint_names_[3]] = j4;

      arm_group_->setJointValueTarget(target);
      MoveGroupInterface::Plan plan;
      auto ok = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (!ok) {
        RCLCPP_WARN(get_logger(), "Planowanie dla ramienia nie powiodło się.");
      } else {
        auto exec_ret = arm_group_->execute(plan);
        if (exec_ret != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_WARN(get_logger(), "Wykonanie ruchu ramienia nie powiodło się.");
        }
      }

      double gval = (grip == 0) ? gripper_closed_ : gripper_open_;
      std::map<std::string, double> gtarget;
      if (!gripper_joint_names_.empty()) {
        for (const auto &jn : gripper_joint_names_) gtarget[jn] = gval;
      } else {
        gtarget[gripper_joint_name_] = gval;
      }

      gripper_group_->setJointValueTarget(gtarget);
      MoveGroupInterface::Plan gplan;
      ok = (gripper_group_->plan(gplan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (!ok) {
        RCLCPP_WARN(get_logger(), "Planowanie dla chwytaka nie powiodło się.");
      } else {
        auto exec_ret = gripper_group_->execute(gplan);
        if (exec_ret != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_WARN(get_logger(), "Wykonanie ruchu chwytaka nie powiodło się.");
        }
      }
    }
  }

  void print_help() {
    std::cout << "=============================="
              << "TeleopMoveIt (4DOF + chwytak)"
              << "Grupy: '" << arm_group_name_ << "', '" << gripper_group_name_ << "'"
              << "Jointy ramienia: ";
    for (auto &n : arm_joint_names_) std::cout << n << " ";
    std::cout << "Chwytak joint(s): ";
    if (!gripper_joint_names_.empty()) {
      for (auto &n : gripper_joint_names_) std::cout << n << " ";
    } else {
      std::cout << gripper_joint_name_;
    }
    std::cout << "velocity_scale=" << vel_scale_ << ", acceleration_scale=" << acc_scale_ << ""
              << "Wpisz 'q' aby zakończyć." << std::endl;
  }

  std::string arm_group_name_, gripper_group_name_;
  std::vector<std::string> arm_joint_names_;
  std::vector<std::string> gripper_joint_names_;
  std::string gripper_joint_name_;
  double vel_scale_{}, acc_scale_{}, gripper_open_{}, gripper_closed_{};

  std::shared_ptr<MoveGroupInterface> arm_group_;
  std::shared_ptr<MoveGroupInterface> gripper_group_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopMoveItNode>();
  node->init();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}




