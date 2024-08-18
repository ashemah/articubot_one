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

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

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

    sensor_fusion_params = os.path.join(
        package_path, "config", "sensor_fusion.yaml"
    )

    depth2scan_params = os.path.join(
        package_path, "config", "depth2scan_params.yaml"
    )

    robot_description_template = os.path.join(package_path,'description','robot.urdf.xacro')
    robot_description = Command(['xacro ', robot_description_template, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    rsp_params = {'robot_description': robot_description, 'use_sim_time': use_sim_time}

    # Create a robot_state_publisher node
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output='screen',
        parameters=[rsp_params]
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

    lidar = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD19'},
            {'topic_name': 'scan'},
            {'frame_id': 'laser_frame'},
            {'port_name': '/dev/ttyUSB0'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0}
        ]
    )

    # micro_ros_agent = Node(
    #     package='micro_ros_agent',
    #     executable='micro_ros_agent',
    #     name="micro_ros_agent",
    #     output='screen',
    #     arguments=['serial', '--dev', '/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_DC:54:75:D8:35:68-if00']
    # )

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
        output='screen',
        parameters=[{"robot_description": robot_description}, controller_params],
    )

    # Launch the controller manager after the robot state publisher has started
    delayed_controller_manager = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=rsp,
            on_start=[controller_manager],
        )
    )

#    delayed_controller_manager = TimerAction(period=4.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
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
        executable="spawner",
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

    depth2scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[('depth', '/camera/camera/depth/image_rect_raw'),
                    ('depth_camera_info', '/camera/camera/depth/camera_info')],
        parameters=[depth2scan_params]
    )

    sensor_fusion = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        #arguments=['--ros-args', '--log-level', 'DEBUG'],
        parameters=[sensor_fusion_params,
        {'use_sim_time': False}]
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
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use sim time if true'),
            DeclareLaunchArgument(
                'use_ros2_control',
                default_value='true',
                description='Use ros2_control if true'),
            rgb_camera_profile_arg,
            rsp,
            joystick,
            lidar,
            twist_mux,
            # camera,
            # depth2scan,
            delayed_controller_manager,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner,
            # micro_ros_agent,
            sensor_fusion,
            face,
        ]
    )
