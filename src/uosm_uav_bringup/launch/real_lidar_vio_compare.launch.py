#!/usr/bin/env python3
"""Standalone launch to run ZED VIO and 2D lidar odom (KISS-ICP) side-by-side with no fusion.
Use to compare /odom/out (VIO) vs /odom/lidar_2d (tilt-gated KISS-ICP) in RViz or Foxglove."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

def generate_launch_description():
    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='true',
        description='Launch Foxglove bridge for visualization',
        choices=['true', 'false']
    )
    use_foxglove = LaunchConfiguration('use_foxglove')

    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

    urdf_dir = get_package_share_directory('uosm_robot_viewer')
    urdf_xacro_path = os.path.join(urdf_dir, 'urdf', 'uosm_uav_platform.urdf.xacro')
    bringup_package = get_package_share_directory('uosm_uav_bringup')
    zed_config_common = os.path.join(bringup_package, 'config', 'sensor_config', 'common_stereo.yaml')
    zed_config_camera = os.path.join(bringup_package, 'config', 'sensor_config', 'zedm.yaml')
    zed_node_parameters = [zed_config_common, zed_config_camera]

    # ZED VIO
    zed_component = ComposableNode(
        package='zed_components',
        plugin='stereolabs::ZedCamera',
        name='zed_node',
        parameters=zed_node_parameters,
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # RPLidar
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

    # VIO only (no fusion): ZED odom -> base_link -> /odom/out for comparison
    odom_republisher_component = ComposableNode(
        package='odom_republisher',
        plugin='uosm::perception::OdomRepublisher',
        name='odom_republisher_node',
        parameters=[{
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'camera_frame': 'zedm_camera_link',
            'broadcast_tf': True,
            'fuse_lidar_2d': False,
        }],
        remappings=[
            ('/vio/odom', '/zed_node/odom'),
            ('/odom/out', '/odom/vio'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # 2D lidar pipeline: scan -> cloud -> KISS-ICP -> tilt-gated /odom/lidar_2d
    scan_to_pointcloud_component = ComposableNode(
        package='lidar_2d_odom',
        plugin='uosm::perception::ScanToPointCloudComponent',
        name='scan_to_pointcloud_node',
        remappings=[
            ('scan', '/scan'),
            ('scan_cloud', '/scan_cloud'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    tilt_gated_odom_component = ComposableNode(
        package='lidar_2d_odom',
        plugin='uosm::perception::TiltGatedOdomComponent',
        name='tilt_gated_odom_node',
        parameters=[{'max_tilt_deg': 15.0}, {'odom_frame': 'odom'}, {'base_frame': 'base_link'}],
        remappings=[
            ('odom_in', '/kiss/odometry'),
            ('orientation', '/odom/vio'),
            ('odom/lidar_2d', '/odom/lidar_2d'),
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

    autonomy_stack_container = ComposableNodeContainer(
        name='autonomy_stack_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        composable_node_descriptions=[
            zed_component,
            rplidar_component,
            odom_republisher_component,
            scan_to_pointcloud_component,
            tilt_gated_odom_component,
            rsp_component,
        ]
    )

    # KISS-ICP (no odom TF so VIO drives the tree; we only compare odom messages)
    kiss_icp_dir = get_package_share_directory('kiss_icp')
    kiss_icp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(kiss_icp_dir, 'launch', 'odometry.launch.py')),
        launch_arguments=[
            ('topic', '/scan_cloud'),
            ('visualize', 'false'),
            ('use_sim_time', 'false'),
            ('base_frame', 'base_link'),
            ('lidar_odom_frame', 'odom'),
            ('publish_odom_tf', 'false'),
        ]
    )

    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'odom'
        ],
        output='screen'
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        condition=IfCondition(use_foxglove),
        parameters=[{'port': 8765, 'address': '0.0.0.0'}],
        output='screen',
    )

    return LaunchDescription([
        set_rmw,
        use_foxglove_arg,
        autonomy_stack_container,
        kiss_icp_launch,
        map_to_odom_tf,
        jsp_node,
        foxglove_bridge,
    ])
