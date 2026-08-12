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
                cmd=["ros2", "run", "gm_autorace", "line_tracing"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "gm_autorace", "traffic_light"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "gm_autorace", "intersection"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "gm_autorace", "construction_sign"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "gm_autorace", "construction"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "gm_autorace", "parking_sign"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "gm_autorace", "parking"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "gm_autorace", "blockbar"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "gm_autorace", "tunnel_sign"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "gm_autorace", "tunnel_right_hand"], output="screen"
            )
        ]
    )
