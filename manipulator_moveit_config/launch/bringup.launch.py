from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    manipulator_config = get_package_share_directory('manipulator_moveit_config')

    # robot_state_publisher + controllers
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(manipulator_config, 'launch', 'rsp.launch.py')
        )
    )

    # MoveIt move_group
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(manipulator_config, 'launch', 'move_group.launch.py')
        )
    )

    # RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(manipulator_config, 'launch', 'moveit_rviz.launch.py')
        )
    )

    # Spawn controllers
    spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(manipulator_config, 'launch', 'spawn_controllers.launch.py')
        )
    )


    return LaunchDescription([
        rsp,
        move_group,
        rviz,
        spawn_controllers,
    ])

