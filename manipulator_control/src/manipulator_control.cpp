#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "manipulator_control",
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



  // Utworzenie executora i uruchomienie wątku spinującego
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);

std::thread executor_thread([&executor]() {
    executor.spin();
});


  // Create a ROS logger
  auto const logger = rclcpp::get_logger("manipulator_control");

  // Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
// auto move_group_interface = MoveGroupInterface(node, "arm");
moveit::planning_interface::MoveGroupInterface::Options move_group_options("arm");
auto move_group_interface =
    moveit::planning_interface::MoveGroupInterface(node, move_group_options);


move_group_interface.startStateMonitor();

move_group_interface.setGoalPositionTolerance(0.05);
move_group_interface.setGoalOrientationTolerance(0.05);  
// move_group_interface.setGoalTolerance(1.0);

auto current_pose = move_group_interface.getCurrentPose();
RCLCPP_INFO(
    logger,
    "Current pose: position=(%.2f, %.2f, %.2f), orientation=(%.2f, %.2f, %.2f, %.2f)",
    current_pose.pose.position.x,
    current_pose.pose.position.y,
    current_pose.pose.position.z,
    current_pose.pose.orientation.x,
    current_pose.pose.orientation.y,
    current_pose.pose.orientation.z,
    current_pose.pose.orientation.w
);


// Set a target Pose
auto const target_pose = []{
  geometry_msgs::msg::Pose msg;
  // msg.orientation.w = 1.0;
  // msg.position.x = 0.0;
  // msg.position.y = -0.2;
  // msg.position.z = 0.2;

  msg.position.x = 0.02;
  msg.position.y = -0.22;
  msg.position.z = 0.04;
  msg.orientation.x = 0.92;
  msg.orientation.y = -0.08;
  msg.orientation.z = 0.05;
  msg.orientation.w = 0.39;
  return msg;
}();


move_group_interface.setApproximateJointValueTarget(target_pose);

auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
  move_group_interface.execute(plan);
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}

current_pose = move_group_interface.getCurrentPose();
RCLCPP_INFO(
    logger,
    "Current pose after movement: position=(%.2f, %.2f, %.2f), orientation=(%.2f, %.2f, %.2f, %.2f)",
    current_pose.pose.position.x,
    current_pose.pose.position.y,
    current_pose.pose.position.z,
    current_pose.pose.orientation.x,
    current_pose.pose.orientation.y,
    current_pose.pose.orientation.z,
    current_pose.pose.orientation.w
);

executor.cancel();
executor_thread.join();

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}