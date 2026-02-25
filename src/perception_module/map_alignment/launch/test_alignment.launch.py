#!/usr/bin/env python3
"""
Test launch file for map alignment with mock trunk observations.

This allows testing the alignment algorithm without real sensors by:
1. Publishing mock trunk observations at known positions
2. Running the map alignment node
3. Visualizing results in RViz

Usage:
  # Test with default offset (0.5, -0.3)
  ros2 launch map_alignment test_alignment.launch.py

  # Test with custom offset (simulates different takeoff positions)
  ros2 launch map_alignment test_alignment.launch.py offset_x:=1.5 offset_y:=-0.8

  # Test with larger offset and more noise
  ros2 launch map_alignment test_alignment.launch.py offset_x:=2.0 offset_y:=1.0 noise_std:=0.2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch arguments for simulated offset (the "error" from initial guess)
    offset_x_arg = DeclareLaunchArgument(
        'offset_x', default_value='0.5',
        description='Simulated X offset from initial guess (meters)'
    )
    offset_y_arg = DeclareLaunchArgument(
        'offset_y', default_value='-0.3',
        description='Simulated Y offset from initial guess (meters)'
    )
    offset_yaw_arg = DeclareLaunchArgument(
        'offset_yaw', default_value='0.05',
        description='Simulated yaw offset (radians)'
    )
    noise_std_arg = DeclareLaunchArgument(
        'noise_std', default_value='0.1',
        description='Observation noise standard deviation (meters)'
    )
    
    # Get package paths
    map_align_dir = get_package_share_directory('map_alignment')
    map_align_config = os.path.join(map_align_dir, 'config', 'map_alignment_params.yaml')
    map_pcd_path = os.path.join(map_align_dir, 'maps', 'tuanmee_site.pcd')
    
    # Mock trunk observation publisher
    mock_trunk_node = Node(
        package='map_alignment',
        executable='mock_trunk_publisher.py',
        name='mock_trunk_publisher',
        output='screen',
        parameters=[{
            'offset_x': LaunchConfiguration('offset_x'),
            'offset_y': LaunchConfiguration('offset_y'),
            'offset_yaw': LaunchConfiguration('offset_yaw'),
            'noise_std': LaunchConfiguration('noise_std'),
            'publish_rate': 2.0,
        }]
    )
    
    # Map alignment component
    map_align_component = ComposableNode(
        package='map_alignment',
        plugin='uosm::perception::MapAlignmentComponent',
        name='map_alignment_node',
        parameters=[
            {'pcd_file_path': map_pcd_path},
            map_align_config,
        ],
        remappings=[
            ('map_alignment_markers', '/map_alignment_markers'),
            ('observed_landmarks', '/trunk_observations'),
            ('alignment_done', '/map_alignment/alignment_done'),
            ('alignment_transform', '/map_alignment/alignment_transform'),
        ]
    )
    
    # Container for map alignment
    alignment_container = ComposableNodeContainer(
        name='alignment_test_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[map_align_component],
        output='screen'
    )
    
    # RViz for visualization
    robot_viewer_dir = get_package_share_directory('uosm_robot_viewer')
    rviz_config = os.path.join(robot_viewer_dir, 'rviz', 'viewer_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        # Arguments
        offset_x_arg,
        offset_y_arg,
        offset_yaw_arg,
        noise_std_arg,
        
        # Nodes
        mock_trunk_node,
        alignment_container,
        rviz_node,
    ])
