#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

# DDS and Zenoh config paths via ament package share directory
_DDS_CONFIG_DIR = os.path.join(
    get_package_share_directory('uosm_uav_bringup'), 'config', 'dds'
)
CYCLONEDDS_URI_PATH = os.path.join(_DDS_CONFIG_DIR, 'cyclonedds_device.xml')
ZENOH_BRIDGE_CONFIG_PATH = os.path.join(_DDS_CONFIG_DIR, 'zenoh_bridge_device.json5')

def generate_launch_description():    
    # LaunchConfiguration
    use_mavros = LaunchConfiguration('use_mavros')
    use_rosbag = LaunchConfiguration('use_rosbag')
    bag_output_path = LaunchConfiguration('bag_output_path')
    rosbag_delay = LaunchConfiguration('rosbag_delay')
    use_zenoh_bridge = LaunchConfiguration('use_zenoh_bridge')


    # Declare launch argument to enable MAVROS
    use_mavros_arg = DeclareLaunchArgument(
        'use_mavros',
        default_value='true',
        description='Whether to launch MAVROS',
        choices=['true', 'false']
    )

    # Declare launch argument to enable ROS2 bag recording
    use_rosbag_arg = DeclareLaunchArgument(
        'use_rosbag',
        default_value='false',
        description='Whether to record ROS2 bag',
        choices=['true', 'false']
    )

    # Declare launch argument for bag output path
    bag_output_path_arg = DeclareLaunchArgument(
        'bag_output_path',
        default_value='bags/test1', # relative
        description='Output directory for ROS2 bag files'
    )

    # Declare launch argument for rosbag startup delay
    rosbag_delay_arg = DeclareLaunchArgument(
        'rosbag_delay',
        default_value='10.0',
        description='Delay in seconds before starting rosbag recording (wait for sensors)'
    )

    # Declare launch argument to enable Zenoh bridge
    use_zenoh_bridge_arg = DeclareLaunchArgument(
        'use_zenoh_bridge',
        default_value='true',
        description='Whether to launch Zenoh bridge for cross-network ROS 2 communication',
        choices=['true', 'false']
    )

    #  DDS and Zenoh environment
    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    set_cyclone_uri = SetEnvironmentVariable(
        'CYCLONEDDS_URI', 'file://' + os.path.realpath(CYCLONEDDS_URI_PATH)
    )

    # Configuration paths
    urdf_dir = get_package_share_directory('uosm_robot_viewer')
    urdf_xacro_path = os.path.join(urdf_dir, 'urdf', 'uosm_uav_platform.urdf.xacro')
    
    planner_dir = get_package_share_directory('planner_manager')
    px4_pluginlists_path = os.path.join(planner_dir, 'config', 'mavros_pluginlists.yaml')
    px4_config_path = os.path.join(planner_dir, 'config', 'mavros_config.yaml')
    
    bringup_package = get_package_share_directory('uosm_uav_bringup')
    zed_config_common = os.path.join(bringup_package, 'config', 'sensor_config', 'common_stereo.yaml')
    zed_config_camera = os.path.join(bringup_package, 'config', 'sensor_config', 'zedm.yaml')
    mcap_options = os.path.join(bringup_package, 'config', 'mcap_writer_options.yaml')
    qos_overrides = os.path.join(bringup_package, 'config', 'qos_overrides.yaml')

    # Flight controller component
    sanity_check_component = ComposableNode(
        package='sanity_check',
        namespace='',
        plugin='uosm::path_planning::SanityCheck',
        name='sanity_check_node',
        parameters=[{
            'flight_height': 1.0,
            'request_timeout': 2.0,
            'hover_period': 10.0,
            'preflight_check_timeout': 2.0,
            'frame_id': 'map',
            'execution_rate': 50.0,
            'do_orbit': False,
            'orbit_radius': 1.0,
            'orbit_speed': 0.5,
        }],
        remappings=[
            ('/mavros/odometry/out', 'mavros/odometry/out'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    
    zed_node_parameters = [zed_config_common, zed_config_camera]

    # Sensor component
    zed_component = ComposableNode(
        package='zed_components',
        plugin='stereolabs::ZedCamera',
        name='zed_node',
        parameters=zed_node_parameters,
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # RPLIDAR component
    rplidar_component = ComposableNode(
        package='sllidar_ros2',
        plugin='sllidar_ros2::SLlidarComponent',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 256000,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Perception component
    perception_component = ComposableNode(
        package='msf_ros2',
        namespace='',
        plugin='uosm::perception::MultiSensorFusion',
        name='msf_node',
        parameters=[
            {"odom_frame": "odom"},
            {"base_frame": "base_link"},
            {"camera_frame": "zedm_camera_link"},
            {"window_size": 10},
            {"publish_rate": 30.0},
            {"buffer_max_size": 10},
            {"broadcast_tf": True},
        ],
        remappings=[
            ('vio/odom', '/zed_node/odom'),
            ('fused/odom', '/mavros/odometry/out')
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    rsp_component = ComposableNode(
        package='robot_state_publisher',
        plugin='robot_state_publisher::RobotStatePublisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_xacro_path]),
            'package_path': bringup_package
        }]
    )

    # Autonomy stack container
    autonomy_stack_container = ComposableNodeContainer(
        name='autonomy_stack_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        composable_node_descriptions=[
            # sanity_check_component,
            zed_component,
            rplidar_component,
            perception_component,
            rsp_component
        ]
    )
    
    jetson_gscam2_dir = get_package_share_directory('jetson_gscam2')
    csi_cam_config = os.path.join(jetson_gscam2_dir, 'config', 'wide_angle_preset.yaml')
    csi_cam_info_url = 'package://jetson_gscam2/config/camera_calibration.yaml'

    # CSI camera component
    csi_camera_component = ComposableNode(
        package='jetson_gscam2',
        plugin='uosm::perception::CSICameraComponent',
        name='csi_camera_node',
        namespace='',
        parameters=[
            csi_cam_config,
            {'camera_info_url': csi_cam_info_url},
        ],
        remappings=[
            ('image_raw', 'csi_cam/image_raw'),
            ('image_raw/compressed', 'csi_cam/image_raw/compressed'),
            ('camera_info', 'csi_cam/camera_info'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # CSI camera container
    csi_camera_container = ComposableNodeContainer(
        name='csi_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        composable_node_descriptions=[
            csi_camera_component
        ]
    )

    # MAVROS
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        condition=IfCondition(use_mavros),
        parameters=[
            px4_pluginlists_path,
            px4_config_path,
            {
                'fcu_url': '/dev/ttyACM0:2000000',
                #'fcu_url': '/dev/ttyTHS1:1152000',
                #'gcs_url': 'udp://@127.0.0.1:14550',
                'gcs_url': 'udp://@192.168.0.2:14550',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': "v2.0",
                'respawn_mavros': "false",
                'namespace': "mavros",
            }
        ],
        arguments=['--ros-args', '--log-level', 'error']  # 'debug', 'info', 'error', 'fatal'
    )

    # Jetson Stats Node
    jetson_stats_node = Node(
        package='ros2_jetson_stats',
        executable='ros2_jtop',
        name='ros2_jtop',
        condition=IfCondition(use_rosbag),
        parameters=[
            {'interval': 1.0} # 0.1 ~ 10Hz max
        ],
        output='screen',
    )

    # Joint State Publisher
    jsp_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen"
    )

    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=[
            '--x', '0.0',
            '--y', '0.0', 
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom'
        ],
        output='screen'
    )

    # Zenoh bridge for cross-network communication
    zenoh_bridge = ExecuteProcess(
        condition=IfCondition(use_zenoh_bridge),
        cmd=[
            'zenoh-bridge-ros2dds',
            '-c', os.path.realpath(ZENOH_BRIDGE_CONFIG_PATH),
        ],
        output='screen',
    )

    # ROS2 bag recording with delay for sensor initialization
    rosbag_record_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_output_path,
            '--storage', 'mcap', # mcap, sqlite3
            '--storage-config-file', mcap_options,
            '--qos-profile-overrides-path', qos_overrides,
            # add topics
            'diagnostics',
            'zed_node/point_cloud/cloud_registered',
            'zed_node/left/color/rect/image',
            'zed_node/right/color/rect/image', 
            'zed_node/left/color/rect/image/camera_info',
            'zed_node/right/color/rect/image/camera_info',
            'mavros/odometry/out',
            'mavros/imu/data',
            'mavros/hps167_pub',
            'csi_cam/image_raw/compressed',
            'csi_cam/camera_info',
            'scan',
        ],
        output='screen',
    )

    rosbag_record = TimerAction(
        period=10.0,  # 10 second delay for sensor initialization
        actions=[rosbag_record_cmd],
        condition=IfCondition(use_rosbag)
    )

    return LaunchDescription([
        # Environment
        set_rmw,
        set_cyclone_uri,

        # Launch arguments
        use_mavros_arg,
        use_rosbag_arg,
        bag_output_path_arg,
        rosbag_delay_arg,
        use_zenoh_bridge_arg,

        # Nodes
        autonomy_stack_container,
        csi_camera_container,
        mavros_node,
        # wifi_scan_node,
        jsp_node,
        map_to_odom_tf,
        jetson_stats_node,
        zenoh_bridge,
        rosbag_record,
    ])
