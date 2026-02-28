#!/usr/bin/env python3
"""
Launch file for map alignment bag playback testing.

Usage:

  # Auto-play bag from the launch file:
  ros2 launch uosm_uav_bringup test_map_alignment.launch.py bag_path:=./bags/my_bag

  # Handle yaw offset
  ros2 launch uosm_uav_bringup test_map_alignment.launch.py initial_yaw_deg:=90.0

  # Adjust PCD vertical offset:
  ros2 launch uosm_uav_bringup test_map_alignment.launch.py pcd_z_offset:=1.5

  # Use Foxglove instead of RViz (use_rviz:=false launches Foxglove by default):
  ros2 launch uosm_uav_bringup test_map_alignment.launch.py use_rviz:=false

Topics expected from bag:
  - /mavros/odometry/out       (nav_msgs/msg/Odometry)
  - /scan                      (sensor_msgs/msg/LaserScan)
  - /zed_node/point_cloud/cloud_registered (sensor_msgs/msg/PointCloud2)
"""

import math
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

# Enable coloured terminal output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def _launch_setup(context, *args, **kwargs):
    initial_yaw_deg = float(LaunchConfiguration('initial_yaw_deg').perform(context))
    initial_yaw_rad = initial_yaw_deg * math.pi / 180.0
    pcd_z_offset = float(LaunchConfiguration('pcd_z_offset').perform(context))
    resolved_bag_path = os.path.expanduser(LaunchConfiguration('bag_path').perform(context))
    use_alignment = LaunchConfiguration('use_alignment').perform(context).lower() == 'true'

    bringup_dir = get_package_share_directory('uosm_uav_bringup')
    planner_config = os.path.join(bringup_dir, 'config', 'planner.yaml')
    rviz_config = os.path.join(bringup_dir, 'config', 'rviz', 'map_alignment_bag_test.rviz')

    robot_viewer_dir = get_package_share_directory('uosm_robot_viewer')
    urdf_xacro_path = os.path.join(robot_viewer_dir, 'urdf', 'uosm_uav_platform.urdf.xacro')

    trunk_seg_dir = get_package_share_directory('trunk_segmentation')
    trunk_seg_config = os.path.join(trunk_seg_dir, 'config', 'trunk_segmentation_params.yaml')

    map_align_dir = get_package_share_directory('map_alignment')
    map_align_config = os.path.join(map_align_dir, 'config', 'map_alignment_params.yaml')
    map_pcd_path = os.path.join(map_align_dir, 'maps', 'klk', 'tuanmee_site.pcd')

    map_proc_dir = get_package_share_directory('map_processor')
    map_proc_config = os.path.join(map_proc_dir, 'config', 'map_processor_params.yaml')
    map_proc_pcd_path = os.path.join(map_proc_dir, 'maps', 'klk', 'tuanmee_site.pcd')

    waypoint_csv = LaunchConfiguration('waypoint_csv').perform(context)
    if not waypoint_csv:
        waypoint_csv = os.path.join(map_proc_dir, 'maps', 'klk', 'single_id_26.csv')

    # Trunk segmentation
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
            ('trunk_seg/odom', '/mavros/odometry/out'),
            ('trunk_observations', '/trunk_observations'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # Map alignment
    map_align_component = ComposableNode(
        package='map_alignment',
        plugin='uosm::perception::MapAlignmentComponent',
        name='map_alignment_node',
        namespace='',
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
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # Ego planner (passive — grid map only)
    planner_component = ComposableNode(
        package='planner_manager',
        plugin='uosm::path_planning::EgoPlanner',
        name='ego_planner_node',
        namespace='',
        parameters=[
            {'planner.waypoint_csv_file_path': ''},  # no auto
            {'use_sim_time': True},
            {'grid_map.is_sim': False},  # use real point cloud callback path
            planner_config,
        ],
        remappings=[
            ('odometry', '/mavros/odometry/out'),
            ('grid_map/cloud', '/zed_node/point_cloud/cloud_registered'),  # disabled: LiDAR-only test
            ('grid_map/scan', '/scan'),
            ('alignment_done', '/map_alignment/alignment_done'),
            ('alignment_transform', '/map_alignment/alignment_transform'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # Robot state publisher
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

    container_components = [
        trunk_seg_component,
        planner_component,
        rsp_component,
    ]
    if use_alignment:
        container_components.insert(1, map_align_component)

    composable_container = ComposableNodeContainer(
        name='alignment_test_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        composable_node_descriptions=container_components,
    )

    # Map processor — reference PCD map display
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

    # Waypoint visualizer
    waypoint_viz_node = Node(
        package='uosm_uav_bringup',
        executable='waypoint_visualizer_node.py',
        name='waypoint_visualizer_node',
        output='screen',
        parameters=[
            {'waypoint_csv_file_path': waypoint_csv},
            {'flight_height': 3.0},
            {'frame_id': 'map'},
            {'use_sim_time': True},
        ],
        remappings=[
            ('waypoint_markers', '/waypoint_markers'),
            ('alignment_done', '/map_alignment/alignment_done'),
            ('alignment_transform', '/map_alignment/alignment_transform'),
        ],
    )

    # Odom visualization
    odom_visualization_node = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name='odom_visualization',
        output='screen',
        remappings=[
            ('odom', '/mavros/odometry/out'),
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

    # Odom → base_link TF broadcaster
    odom_to_tf_node = Node(
        package='uosm_uav_bringup',
        executable='odom_to_tf_node.py',
        name='odom_to_tf_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Joint state publisher
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # RViz
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

    # Foxglove bridge when RViz is disabled
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

    # Bag playback
    use_bag_player = LaunchConfiguration('use_bag_player')
    bag_play = ExecuteProcess(
        condition=IfCondition(use_bag_player),
        cmd=['ros2', 'bag', 'play', resolved_bag_path, '--clock'],
        output='screen',
    )

    entities = [
        composable_container,
        map_processor_node,
        waypoint_viz_node,
        odom_visualization_node,
        odom_to_tf_node,
        jsp_node,
        rviz_node,
        foxglove_bridge_node,
        bag_play,
    ]

    if not use_alignment:
        # Static identity map → odom when alignment is disabled
        static_map_to_odom = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )
        entities.append(static_map_to_odom)

    return entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value='./bags/real/klk/test_manual_1_decoded',
            description='Path to the rosbag to play',
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
            'waypoint_csv',
            default_value='',
            description='Path to waypoint CSV file (empty = default id_41.csv)',
        ),
        DeclareLaunchArgument(
            'initial_yaw_deg',
            default_value='0.0',
            description='Drone initial yaw offset in degrees (ENU/NED). '
                        '0 = drone faces east while map is aligned north.',
        ),
        DeclareLaunchArgument(
            'pcd_z_offset',
            default_value='0.0',
            description='Vertical offset to lower/raise the PCD map (starting_point.z)',
        ),
        DeclareLaunchArgument(
            'use_alignment',
            default_value='true',
            description='Enable map alignment. When false, only trunk segmentation '
                        'runs with a static identity map->odom TF.',
        ),

        OpaqueFunction(function=_launch_setup),
    ])
