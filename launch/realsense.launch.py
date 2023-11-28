import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="realsense2_camera",
                executable="rs_launch.py",
                output="screen",
                parameters=[
                    {"rgb_camera.profile": "640x480x30""}
                ],
            )
        ]
    )
