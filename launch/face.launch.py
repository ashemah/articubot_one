import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='dozer_face',
            executable='dozer_face',
            output='screen',
        )
    ])
