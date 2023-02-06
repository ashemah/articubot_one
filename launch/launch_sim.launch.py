import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, TextSubstitution, LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name = "articubot_one"  # <--- CHANGE ME

    rgb_camera_profile_arg = DeclareLaunchArgument(
        "rgb_camera_profile", default_value=TextSubstitution(text="640x480x30")
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name), "launch", "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true", "use_ros2_control": "true"}.items(),
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch",
                    "joystick.launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), "config", "twist_mux.yaml"
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {"use_sim_time": True}],
        remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
    )

    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name), "config", "gazebo_params.yaml"
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={
            "extra_gazebo_args": "--verbose --ros-args --params-file " + gazebo_params_file
        }.items(),
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_bot"],
        output="screen",
    )

    delayed_controller_manager_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=["diff_cont"],
            ),
            Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=["joint_broad"],
            ),
        ],
    )

    camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        parameters=[{"rgb_camera.profile", LaunchConfiguration("rgb_camera_profile")}],
    )

    pc2scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        arguments=[],
    )

    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["diff_cont"],
    # )

    # joint_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["joint_broad"],
    # )

    # Code for delaying a node (I haven't tested how effective it is)
    #
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner

    # Launch them all!
    return LaunchDescription(
        [
            rgb_camera_profile_arg,
            camera,
            pc2scan,
            rsp,
            joystick,
            twist_mux,
            gazebo,
            spawn_entity,
            delayed_controller_manager_spawner,
            # diff_drive_spawner,
            # joint_broad_spawner
        ]
    )
