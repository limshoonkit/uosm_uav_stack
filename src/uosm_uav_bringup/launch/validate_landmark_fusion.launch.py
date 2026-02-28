#!/usr/bin/env python3
"""
Replay a post landmark fusion corrected odometry bag.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def _launch_setup(context, *args, **kwargs):
    pcd_z_offset = float(LaunchConfiguration('pcd_z_offset').perform(context))
    resolved_bag_path = os.path.expanduser(
        LaunchConfiguration('bag_path').perform(context))

    bringup_dir = get_package_share_directory('uosm_uav_bringup')
    planner_config = os.path.join(bringup_dir, 'config', 'planner.yaml')
    rviz_config = os.path.join(
        bringup_dir, 'config', 'rviz', 'validate_landmark_fusion.rviz')

    robot_viewer_dir = get_package_share_directory('uosm_robot_viewer')
    urdf_xacro_path = os.path.join(
        robot_viewer_dir, 'urdf', 'uosm_uav_platform.urdf.xacro')

    trunk_seg_dir = get_package_share_directory('trunk_segmentation')
    trunk_seg_config = os.path.join(
        trunk_seg_dir, 'config', 'trunk_segmentation_params.yaml')

    map_proc_dir = get_package_share_directory('map_processor')
    map_proc_config = os.path.join(
        map_proc_dir, 'config', 'map_processor_params.yaml')
    map_proc_pcd_path = os.path.join(
        map_proc_dir, 'maps', 'klk', 'tuanmee_site.pcd')

    # ── composable nodes ───────────────────────────────────────────────

    trunk_seg_component = ComposableNode(
        package='trunk_segmentation',
        plugin='uosm::perception::TrunkSegmentationComponent',
        name='trunk_segmentation_node',
        namespace='',
        parameters=[
            trunk_seg_config,
            {'use_sim_time': True},
        ],
        remappings=[
            ('trunk_seg/scan', '/scan'),
            ('trunk_seg/odom', '/odom/corrected'),
            ('trunk_observations', '/trunk_observations'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    planner_component = ComposableNode(
        package='planner_manager',
        plugin='uosm::path_planning::EgoPlanner',
        name='ego_planner_node',
        namespace='',
        parameters=[
            {'planner.waypoint_csv_file_path': ''},
            {'use_sim_time': True},
            {'grid_map.is_sim': False},
            planner_config,
        ],
        remappings=[
            ('odometry', '/odom/corrected'),
            ('grid_map/cloud', '/zed_node/point_cloud/cloud_registered'),
            ('grid_map/scan', '/scan'),
            ('alignment_done', '/map_alignment/alignment_done'),
            ('alignment_transform', '/map_alignment/alignment_transform'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    rsp_component = ComposableNode(
        package='robot_state_publisher',
        plugin='robot_state_publisher::RobotStatePublisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_xacro_path]),
            'package_path': robot_viewer_dir,
            'use_sim_time': True,
        }],
    )

    composable_container = ComposableNodeContainer(
        name='replay_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        composable_node_descriptions=[
            trunk_seg_component,
            planner_component,
            rsp_component,
        ],
    )

    # ── standalone nodes ───────────────────────────────────────────────

    map_processor_node = Node(
        package='map_processor',
        executable='map_processor_node',
        name='map_processor_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'pcd_file_path': map_proc_pcd_path},
            {'starting_point.z': pcd_z_offset},
            {'use_sim_time': True},
            map_proc_config,
        ],
        remappings=[
            ('global_cloud', '/map_generator/global_cloud'),
        ],
    )

    odom_visualization_node = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name='odom_visualization',
        output='screen',
        remappings=[
            ('odom', '/odom/corrected'),
            ('robot', 'vis/robot'),
            ('path', 'vis/path'),
            ('time_gap', 'vis/time_gap'),
        ],
        parameters=[
            {'use_sim_time': True},
            {'robot_scale': 1.0},
            {'tf45': False},
        ],
    )

    odom_to_tf_node = Node(
        package='uosm_uav_bringup',
        executable='odom_to_tf_node.py',
        name='odom_to_tf_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': '/odom/corrected',
        }],
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'port': 8765},
        ],
        condition=UnlessCondition(use_rviz),
    )

    # ── static map → odom TF ─────────────────────────────────────────
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'odom',
        ],
        output='screen',
    )

    # ── bag playback ───────────────────────────────────────────────────
    bag_play = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('use_bag_player')),
        cmd=[
            'ros2', 'bag', 'play', resolved_bag_path, '--clock',
            '--topics',
            '/odom/corrected',
            '/scan',
            '/zed_node/point_cloud/cloud_registered',
            '/zed_node/left/color/rect/image',
            '/zed_node/left/color/rect/image/camera_info',
            '/zed_node/right/color/rect/image',
            '/zed_node/right/color/rect/image/camera_info',
            '/csi_cam/image_raw',
            '/csi_cam/image_raw/compressed',
            '/csi_cam/camera_info',
            '/mavros/imu/data',
            '/mavros/hps167_pub',
            '/grid_map/occupancy_inflate',
            '/diagnostics',
        ],
        output='screen',
    )

    return [
        composable_container,
        map_processor_node,
        odom_visualization_node,
        odom_to_tf_node,
        static_map_to_odom,
        jsp_node,
        rviz_node,
        foxglove_bridge_node,
        bag_play,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value='./bags/real/klk/test_manual_1_corrected',
            description='Path to the recorded fusion bag',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz2',
        ),
        DeclareLaunchArgument(
            'use_bag_player',
            default_value='true',
            description='Whether to auto-play the bag file from this launch',
        ),
        DeclareLaunchArgument(
            'pcd_z_offset',
            default_value='0.0',
            description='Vertical offset to lower/raise the PCD map (starting_point.z)',
        ),

        OpaqueFunction(function=_launch_setup),
    ])
