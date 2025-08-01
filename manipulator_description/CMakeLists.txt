cmake_minimum_required(VERSION 3.8)
project(manipulator_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(manipulator_hw SHARED
  hardware/src/manipulator_system.cpp
)
target_include_directories(manipulator_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(manipulator_hw
  rclcpp
  pluginlib
  hardware_interface
)
target_compile_features(manipulator_hw PUBLIC cxx_std_17)

pluginlib_export_plugin_description_file(hardware_interface manipulator_hw_plugins.xml)

install(TARGETS manipulator_hw
  LIBRARY DESTINATION lib
)

install(DIRECTORY hardware/include/
  DESTINATION include
)

install(FILES manipulator_hw_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
