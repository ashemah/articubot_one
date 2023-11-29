import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, TextSubstitution, LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name = "articubot_one"  # <--- CHANGE ME
    package_path = get_package_share_directory(package_name)

    # Params    
    rgb_camera_profile_arg = DeclareLaunchArgument(
        "rgb_camera_profile", default_value=TextSubstitution(text="640x480x30")
    )

    twist_mux_params = os.path.join(
        package_path, "config", "twist_mux.yaml"
    )

    controller_params = os.path.join(
        package_path, "config", "my_controllers.yaml"
    )

    # Launch files
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    package_path, 
                    "launch", 
                    "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time": "false", "use_ros2_control": "true"}.items(),
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    package_path,
                    "launch",
                    "joystick.launch.py",
                )
            ]
        )
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
    )

    robot_description = Command(
        ["ros2 param get --hide-type /robot_state_publisher robot_description"]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controller_params],
    )

    # Launch the controller manager after the robot state publisher has started
    delayed_controller_manager = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=rsp,
            on_start=[controller_manager],
        )
    )

    # delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    face = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(
                package_path,
                "launch",
                "face.launch.py",
            )]
        )
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
            rsp,
            joystick,
            twist_mux,
            # camera,
            # pc2scan,
            delayed_controller_manager,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner,
            face,
        ]
    )
