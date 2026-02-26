#!/usr/bin/env python3
"""Launch scan_to_pointcloud and tilt_gated_odom for 2D lidar odom (use with KISS-ICP)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    max_tilt_arg = DeclareLaunchArgument(
        'max_tilt_deg',
        default_value='15.0',
        description='Max roll/pitch (deg) to publish lidar odom',
    )
    max_tilt_deg = LaunchConfiguration('max_tilt_deg', default='15.0')

    scan_to_cloud = ComposableNode(
        package='lidar_2d_odom',
        plugin='uosm::perception::ScanToPointCloudComponent',
        name='scan_to_pointcloud_node',
        remappings=[
            ('scan', '/scan'),
            ('scan_cloud', '/scan_cloud'),
        ],
    )

    tilt_gated = ComposableNode(
        package='lidar_2d_odom',
        plugin='uosm::perception::TiltGatedOdomComponent',
        name='tilt_gated_odom_node',
        parameters=[
            {'max_tilt_deg': max_tilt_deg},
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
        ],
        remappings=[
            ('odom_in', '/kiss_icp/odometry'),
            ('orientation', '/odom/out'),
            ('odom/lidar_2d', '/odom/lidar_2d'),
        ],
    )

    container = ComposableNodeContainer(
        name='lidar_2d_odom_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[scan_to_cloud, tilt_gated],
        output='screen',
    )

    return LaunchDescription([
        max_tilt_arg,
        container,
    ])
