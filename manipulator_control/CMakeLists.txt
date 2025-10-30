cmake_minimum_required(VERSION 3.8)
project(manipulator_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)


add_executable(teleop_moveit_node src/teleop_moveit_node.cpp)
ament_target_dependencies(teleop_moveit_node
  rclcpp
  moveit_ros_planning_interface
)

install(TARGETS teleop_moveit_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(manipulator_control src/manipulator_control.cpp)
ament_target_dependencies(manipulator_control
  rclcpp
  moveit_ros_planning_interface
  ament_index_cpp
)
target_link_libraries(manipulator_control yaml-cpp)
install(TARGETS manipulator_control
  RUNTIME DESTINATION lib/${PROJECT_NAME})


ament_package()
