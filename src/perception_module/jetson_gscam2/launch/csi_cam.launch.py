#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    jetson_gscam2_dir = get_package_share_directory('jetson_gscam2')
    csi_cam_config = os.path.join(jetson_gscam2_dir, 'config', 'streaming_1080p_preset.yaml') # [default_preset.yaml, wide_angle_preset.yaml, latency_preset.yaml, streaming_1080p_preset.yaml]
    csi_cam_info_url = 'package://jetson_gscam2/config/camera_calibration.yaml'
    qos_overrides = os.path.join(jetson_gscam2_dir, 'config', 'qos_overrides.yaml')

    csi_camera_component = ComposableNode(
        package='jetson_gscam2',
        plugin='uosm::perception::CSICameraComponent',
        name='csi_camera_node',
        namespace='',
        parameters=[
            csi_cam_config,
            # {'camera_info_url': csi_cam_info_url},
        ],
        remappings=[
            ('image_raw', 'csi_cam/image_raw'),
            ('image_raw/compressed', 'csi_cam/image_raw/compressed'),
            ('camera_info', 'csi_cam/camera_info'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Create container with the component
    csi_container = ComposableNodeContainer(
        name='csi_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[csi_camera_component],
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )

    # Foxglove Bridge — relays compressed topic over WebSocket to Foxglove Studio
    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='true',
        description='Whether to launch Foxglove Bridge for WiFi streaming',
        choices=['true', 'false']
    )

    foxglove_bridge = Node(
        condition=IfCondition(LaunchConfiguration('use_foxglove')),
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'send_buffer_limit': 500_000,
            'num_threads': 1,
        }],
        output='screen',
    )

    use_rosbag_arg = DeclareLaunchArgument(
        'use_rosbag',
        default_value='true',
        description='Whether to record ROS2 bag',
        choices=['true', 'false']
    )

    bag_output_path_arg = DeclareLaunchArgument(
        'bag_output_path',
        default_value='bags/csi_cam_recording',
        description='Output directory for ROS2 bag files'
    )

    use_rosbag = LaunchConfiguration('use_rosbag')
    bag_output_path = LaunchConfiguration('bag_output_path')

    rosbag_record = ExecuteProcess(
        condition=IfCondition(use_rosbag),
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_output_path,
            '--compression-mode', 'none',
            '--qos-profile-overrides-path', qos_overrides,
            '/csi_cam/image_raw/compressed',
        ],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(use_foxglove_arg)
    ld.add_action(use_rosbag_arg)
    ld.add_action(bag_output_path_arg)
    ld.add_action(csi_container)
    ld.add_action(foxglove_bridge)
    ld.add_action(rosbag_record)

    return ld
