#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    # create and return launch description object
    return LaunchDescription(
        [   
            ExecuteProcess(
                cmd=["ros2", "launch", "turtlebot3_bringup", "robot.launch.py"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "csi_cam", "cam1"], output="screen"
            )
        ]
    )
