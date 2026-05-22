# Manipulator System

This repository contains a ROS 2–based control and simulation system for a robotic manipulator.
It includes MoveIt integration, pick-and-place functionality, and teleoperation.

## Requirements

- ROS 2 Jazzy (required)
- Linux system compatible with ROS 2 Jazzy
- MoveIt
- ros2_control
#### Install MoveIt 2 and ros2_control

```bash
sudo apt update
sudo apt install ros-jazzy-moveit
sudo apt install \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager
```

#### Make sure your ROS 2 environment is sourced:

source /opt/ros/jazzy/setup.bash

## Build the Workspace

cd manipulator_ws

colcon build

source install/setup.bash

## Running the System (MoveIt Demo)

#### Launch the MoveIt configuration and simulation:

cd manipulator_ws/ 

ros2 launch manipulator_moveit_config demo.launch.py

## Pick and Place

#### Run the pick-and-place example:

ros2 run manipulator_control pick_and_place

## Teleoperation

#### Start teleoperation using MoveIt:

ros2 run manipulator_control teleop_moveit_node

## Notes

- Make sure the workspace is built and sourced before running any commands.
- This project is developed and tested with ROS 2 Jazzy only.
