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
                cmd=["ros2", "run", "gm_autorace", "line_tracing", "--ros-args", "-p", "sim_bool:=True"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "gm_autorace", "traffic_light", "--ros-args", "-p", "sim_bool:=True"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "gm_autorace", "intersection", "--ros-args", "-p", "sim_bool:=True"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "gm_autorace", "construction_sign", "--ros-args", "-p", "sim_bool:=True"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "gm_autorace", "construction", "--ros-args", "-p", "sim_bool:=True"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "gm_autorace", "parking_sign", "--ros-args", "-p", "sim_bool:=True"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "gm_autorace", "parking", "--ros-args", "-p", "sim_bool:=True"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "gm_autorace", "blockbar", "--ros-args", "-p", "sim_bool:=True"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "gm_autorace", "tunnel_sign", "--ros-args", "-p", "sim_bool:=True"], output="screen"
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "gm_autorace", "tunnel_right_hand", "--ros-args", "-p", "sim_bool:=True"], output="screen"
            )
        ]
    )
