#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
    "pick_and_place",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  try {
    std::string kinematics_path =
      ament_index_cpp::get_package_share_directory("manipulator_moveit_config") + "/config/kinematics.yaml";
    YAML::Node kin_yaml = YAML::LoadFile(kinematics_path);

    for (auto it = kin_yaml.begin(); it != kin_yaml.end(); ++it) {
      std::string group_name = it->first.as<std::string>();
      auto group_params = it->second;

      std::string ns = "robot_description_kinematics." + group_name + ".";

      for (auto param_it = group_params.begin(); param_it != group_params.end(); ++param_it) {
        std::string param_name = ns + param_it->first.as<std::string>();
        auto value = param_it->second;

        if (value.IsScalar()) {
          std::string str_val = value.as<std::string>();
          try {
            double d_val = std::stod(str_val);
            node->declare_parameter(param_name, d_val);
            node->set_parameter(rclcpp::Parameter(param_name, d_val));
          } catch (...) {
            node->declare_parameter(param_name, str_val);
            node->set_parameter(rclcpp::Parameter(param_name, str_val));
          }
        }
      }
    }
    RCLCPP_INFO(node->get_logger(),
      "Loaded kinematics parameters from YAML into robot_description_kinematics namespace!");
  } catch (const std::exception &e) {
    RCLCPP_WARN(node->get_logger(), "Could not load kinematics.yaml: %s", e.what());
  }


  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  auto const logger = rclcpp::get_logger("pick_and_place");


  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface arm_group(node, "arm");
  MoveGroupInterface gripper_group(node, "gripper");

  arm_group.startStateMonitor();
  gripper_group.startStateMonitor();

  std::vector<geometry_msgs::msg::Pose> waypoints;

  geometry_msgs::msg::Pose pregrip;
  pregrip.position.x = -0.17;
  pregrip.position.y = -0.13;
  pregrip.position.z = 0.20;
  pregrip.orientation.x = 0.65;
  pregrip.orientation.y = -0.30;
  pregrip.orientation.z = -0.32;
  pregrip.orientation.w = 0.62;
  waypoints.push_back(pregrip);

  geometry_msgs::msg::Pose grip;
  grip.position.x = -0.23;
  grip.position.y = -0.17;
  grip.position.z = 0.03;
  grip.orientation.x = 0.80;
  grip.orientation.y = -0.38;
  grip.orientation.z = -0.21;
  grip.orientation.w = 0.40;
  waypoints.push_back(grip);

  geometry_msgs::msg::Pose prerelease;
  prerelease.position.x = 0.0;
  prerelease.position.y = -0.2;
  prerelease.position.z = 0.2;
  prerelease.orientation.x = 0.71;
  prerelease.orientation.y = 0.0;
  prerelease.orientation.z = 0.0;
  prerelease.orientation.w = 0.71;
  waypoints.push_back(prerelease);

  geometry_msgs::msg::Pose release;
  release.position.x = 0.01;
  release.position.y = -0.22;
  release.position.z = 0.06;
  release.orientation.x = 0.92;
  release.orientation.y = -0.08;
  release.orientation.z = 0.05;
  release.orientation.w = 0.39;
  waypoints.push_back(release);


  for (size_t i = 0; i < waypoints.size(); ++i) {
    RCLCPP_INFO(logger, "Planning to pose %zu...", i + 1);

    arm_group.setApproximateJointValueTarget(waypoints[i]);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(arm_group.plan(plan));

    if (success) {
      RCLCPP_INFO(logger, "Plan %zu succeeded, executing...", i + 1);
      arm_group.execute(plan);
      rclcpp::sleep_for(std::chrono::seconds(2));

      if (i == 0) {
        RCLCPP_INFO(logger, "Opening gripper...");
        gripper_group.setNamedTarget("open");
        gripper_group.move();
      }
      if (i == 1) {
        RCLCPP_INFO(logger, "Closing gripper...");
        gripper_group.setNamedTarget("close");
        gripper_group.move();
      }
      if (i == 3) {
        RCLCPP_INFO(logger, "Opening gripper (release)...");
        gripper_group.setNamedTarget("open");
        gripper_group.move();
      }

    } else {
      RCLCPP_WARN(logger, "Plan %zu failed, skipping...", i + 1);
    }
  }

    arm_group.setNamedTarget("ready");
    moveit::planning_interface::MoveGroupInterface::Plan plan_ready;
    if (arm_group.plan(plan_ready) == moveit::core::MoveItErrorCode::SUCCESS) {
    arm_group.execute(plan_ready);
    }

  RCLCPP_INFO(logger, " Pick and place sequence complete!");


  executor.cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return 0;
}
