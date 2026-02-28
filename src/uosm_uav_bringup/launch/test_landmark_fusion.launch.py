#!/usr/bin/env python3
"""
Launch file for testing iSAM2 landmark fusion with rosbag playback.

Runs trunk segmentation, map alignment (one-shot), and odom_republisher
(iSAM2 landmark fusion).  Two odometry paths are visualised side-by-side:
  - RED  path:  raw VIO from the bag  (/mavros/odometry/out)
  - GREEN path: iSAM2-corrected       (/odom/corrected)

Usage:
  ros2 launch uosm_uav_bringup test_landmark_fusion.launch.py \
      bag_path:=./bags/real/klk/test_manual_1_decoded

Topics expected from bag:
  - /mavros/odometry/out       (nav_msgs/msg/Odometry)
  - /scan                      (sensor_msgs/msg/LaserScan)
"""

import math
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def _launch_setup(context, *args, **kwargs):
    initial_yaw_deg = float(LaunchConfiguration('initial_yaw_deg').perform(context))
    initial_yaw_rad = initial_yaw_deg * math.pi / 180.0
    resolved_bag_path = os.path.expanduser(
        LaunchConfiguration('bag_path').perform(context))

    keyframe_dist = float(
        LaunchConfiguration('keyframe_dist_threshold').perform(context))
    keyframe_yaw = float(
        LaunchConfiguration('keyframe_yaw_threshold').perform(context))
    log_level = LaunchConfiguration('log_level').perform(context)
    

    # ── package paths ──────────────────────────────────────────────────
    trunk_seg_dir = get_package_share_directory('trunk_segmentation')
    trunk_seg_config = os.path.join(
        trunk_seg_dir, 'config', 'trunk_segmentation_params.yaml')

    map_align_dir = get_package_share_directory('map_alignment')
    map_align_config = os.path.join(
        map_align_dir, 'config', 'map_alignment_params.yaml')
    map_pcd_path = os.path.join(
        map_align_dir, 'maps', 'klk', 'tuanmee_site.pcd')

    odom_repub_dir = get_package_share_directory('odom_republisher')
    odom_repub_config = os.path.join(
        odom_repub_dir, 'config', 'odom_republisher_params.yaml')

    robot_viewer_dir = get_package_share_directory('uosm_robot_viewer')
    urdf_xacro_path = os.path.join(
        robot_viewer_dir, 'urdf', 'uosm_uav_platform.urdf.xacro')

    bringup_dir = get_package_share_directory('uosm_uav_bringup')
    rviz_config = os.path.join(bringup_dir, 'config', 'rviz', 'landmark_fusion_bag_test.rviz')

    # ── composable nodes ───────────────────────────────────────────────
    trunk_seg = ComposableNode(
        package='trunk_segmentation',
        plugin='uosm::perception::TrunkSegmentationComponent',
        name='trunk_segmentation_node',
        parameters=[
            trunk_seg_config,
            {'use_sim_time': True},
        ],
        remappings=[
            ('trunk_seg/scan', '/scan'),
            ('trunk_seg/odom', '/mavros/odometry/out'),
            ('trunk_observations', '/trunk_observations'),
        ],
    )

    map_align = ComposableNode(
        package='map_alignment',
        plugin='uosm::perception::MapAlignmentComponent',
        name='map_alignment_node',
        parameters=[
            {'pcd_file_path': map_pcd_path},
            map_align_config,
            {
                'use_sim_time': True,
                'initial_yaw_rad': initial_yaw_rad,
            },
        ],
        remappings=[
            ('map_alignment_markers', '/map_alignment_markers'),
            ('observed_landmarks', '/trunk_observations'),
            ('alignment_done', '/map_alignment/alignment_done'),
            ('alignment_transform', '/map_alignment/alignment_transform'),
        ],
    )

    odom_repub = ComposableNode(
        package='odom_republisher',
        plugin='uosm::perception::OdomRepublisher',
        name='odom_republisher_node',
        parameters=[
            odom_repub_config,
            {
                'use_sim_time': True,
                'camera_frame': 'base_link',
                'use_landmark_fusion': True,
                'keyframe_dist_threshold': keyframe_dist,
                'keyframe_yaw_threshold': keyframe_yaw,
            },
        ],
        remappings=[
            ('vio/odom', '/mavros/odometry/out'),
            ('odom/out', '/odom/corrected'),
            ('trunk_observations', '/trunk_observations'),
            ('alignment_done', '/map_alignment/alignment_done'),
            ('alignment_transform', '/map_alignment/alignment_transform'),
        ],
    )

    rsp = ComposableNode(
        package='robot_state_publisher',
        plugin='robot_state_publisher::RobotStatePublisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_xacro_path]),
            'package_path': robot_viewer_dir,
            'use_sim_time': True,
        }],
    )

    container = ComposableNodeContainer(
        name='landmark_fusion_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
        composable_node_descriptions=[
            trunk_seg,
            map_align,
            odom_repub,
            rsp,
        ],
    )

    # ── raw VIO path (red) ─────────────────────────────────────────────
    raw_viz = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name='raw_vio_viz',
        output='screen',
        remappings=[
            ('odom', '/mavros/odometry/out'),
            ('robot', '/vis/raw_robot'),
            ('path', '/vis/raw_path'),
            ('time_gap', '/vis/raw_time_gap'),
        ],
        parameters=[{
            'use_sim_time': True,
            'robot_scale': 1.0,
            'tf45': False,
            'color.r': 1.0,
            'color.g': 0.2,
            'color.b': 0.2,
            'color.a': 0.6,
        }],
    )

    # ── corrected path (green) ─────────────────────────────────────────
    corrected_viz = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name='corrected_viz',
        output='screen',
        remappings=[
            ('odom', '/odom/corrected'),
            ('robot', '/vis/corrected_robot'),
            ('path', '/vis/corrected_path'),
            ('time_gap', '/vis/corrected_time_gap'),
        ],
        parameters=[{
            'use_sim_time': True,
            'robot_scale': 1.0,
            'tf45': False,
            'color.r': 0.1,
            'color.g': 1.0,
            'color.b': 0.2,
            'color.a': 0.9,
        }],
    )

    # ── TF broadcaster (odom → base_link from bag VIO) ────────────────
    odom_tf = Node(
        package='uosm_uav_bringup',
        executable='odom_to_tf_node.py',
        name='odom_to_tf_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── joint state publisher (for URDF links) ────────────────────────
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── RViz ───────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # ── bag playback ───────────────────────────────────────────────────
    bag_play = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('use_bag_player')),
        cmd=['ros2', 'bag', 'play', resolved_bag_path, '--clock'],
        output='screen',
    )

    # ── optional bag recording of fusion results ──────────────────────
    record_path = LaunchConfiguration('record_path').perform(context)
    record_bag = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record')),
        cmd=[
            'ros2', 'bag', 'record',
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
            '/tf', '/tf_static',
            '--use-sim-time',
            '-s', 'mcap',
            '-o', record_path,
        ],
        output='screen',
    )

    return [
        container,
        raw_viz,
        corrected_viz,
        odom_tf,
        jsp,
        rviz,
        bag_play,
        record_bag,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value='./bags/real/klk/test_manual_1_decoded',
            description='Path to the rosbag to play',
        ),
        DeclareLaunchArgument(
            'initial_yaw_deg', default_value='0.0',
            description='Drone initial yaw offset in degrees',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz2',
        ),
        DeclareLaunchArgument(
            'use_bag_player', default_value='true',
            description='Auto-play the rosbag',
        ),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='Log level (debug to see per-keyframe iSAM2 output)',
        ),

        # ── recording ──────────────────────────────────────────────────
        DeclareLaunchArgument(
            'record', default_value='false',
            description='Record fusion results to a bag',
        ),
        DeclareLaunchArgument(
            'record_path', default_value='./bags/real/klk/test_manual_1_corrected',
            description='Output path for the recorded bag',
        ),

        # ── iSAM2 tuning (override from CLI) ──────────────────────────
        DeclareLaunchArgument(
            'keyframe_dist_threshold', default_value='0.15',
            description='Keyframe distance threshold (m)',
        ),
        DeclareLaunchArgument(
            'keyframe_yaw_threshold', default_value='0.05',
            description='Keyframe yaw threshold (rad, ~3 deg)',
        ),

        OpaqueFunction(function=_launch_setup),
    ])
