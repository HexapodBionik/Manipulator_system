cmake_minimum_required(VERSION 3.8)
project(manipulator_hw)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED src/manipulator_system.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} rclcpp hardware_interface pluginlib builtin_interfaces)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

pluginlib_export_plugin_description_file(hardware_interface manipulator_hw_plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(FILES manipulator_hw_plugins.xml DESTINATION share/${PROJECT_NAME})

ament_package()
