#!/usr/bin/env python3
"""Minimal launch: ZEDM odom + ego planner only."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def generate_launch_description():
    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

    use_foxglove = LaunchConfiguration('use_foxglove')
    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='false',
        description='Whether to launch Foxglove Bridge',
        choices=['true', 'false']
    )

    # Paths
    urdf_dir = get_package_share_directory('uosm_robot_viewer')
    urdf_xacro_path = os.path.join(urdf_dir, 'urdf', 'uosm_uav_platform.urdf.xacro')

    planner_dir = get_package_share_directory('planner_manager')
    px4_pluginlists_path = os.path.join(planner_dir, 'config', 'mavros_pluginlists.yaml')
    px4_config_path = os.path.join(planner_dir, 'config', 'mavros_config.yaml')

    bringup_package = get_package_share_directory('uosm_uav_bringup')
    fc_config = os.path.join(bringup_package, 'config', 'fc.yaml')
    planner_config = os.path.join(bringup_package, 'config', 'planner.yaml')
    zed_config_common = os.path.join(bringup_package, 'config', 'sensor_config', 'common_stereo.yaml')
    zed_config_camera = os.path.join(bringup_package, 'config', 'sensor_config', 'zedm.yaml')
    # zed_config_camera = os.path.join(bringup_package, 'config', 'sensor_config', 'zedm_gen3.yaml')

    waypoint_dir = get_package_share_directory('map_processor')
    waypoint_config = os.path.join(waypoint_dir, 'maps', 'uosm', 'uosm_indoor.csv')

    # Flight controller (no alignment wait)
    fc_component = ComposableNode(
        package='planner_manager',
        namespace='',
        plugin='uosm::path_planning::FlightController',
        name='flight_controller_node',
        parameters=[
            fc_config,
            {'fc.wait_for_alignment': False},
        ],
        remappings=[
            ('odometry', 'mavros/odometry/out'),
            ('alignment_done', '/map_alignment/alignment_done'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Ego planner (ZED point cloud only; no /scan from LiDAR)
    planner_component = ComposableNode(
        package='planner_manager',
        namespace='',
        plugin='uosm::path_planning::EgoPlanner',
        name='ego_planner_node',
        parameters=[
            {'planner.waypoint_csv_file_path': waypoint_config},
            planner_config,
        ],
        remappings=[
            ('odometry', 'mavros/odometry/out'),
            ('grid_map/cloud', 'zed_node/point_cloud/cloud_registered'),
            ('grid_map/scan', '/scan'),
            ('alignment_done', '/map_alignment/alignment_done'),
            ('alignment_transform', '/map_alignment/alignment_transform'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    zed_node_parameters = [zed_config_common, zed_config_camera]
    zed_component = ComposableNode(
        package='zed_components',
        plugin='stereolabs::ZedCamera',
        name='zed_node',
        parameters=zed_node_parameters,
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # ZEDM odom: VIO odom -> base_link, published to mavros/odometry/out
    perception_component = ComposableNode(
        package='odom_republisher',
        namespace='',
        plugin='uosm::perception::OdomRepublisher',
        name='odom_republisher_node',
        parameters=[{
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'camera_frame': 'zedm_camera_link',
            'broadcast_tf': True,
        }],
        remappings=[
            ('/vio/odom', '/zed_node/odom'),
            ('/odom/out', '/mavros/odometry/out'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    rsp_component = ComposableNode(
        package='robot_state_publisher',
        plugin='robot_state_publisher::RobotStatePublisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_xacro_path]),
            'package_path': bringup_package,
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
            fc_component,
            zed_component,
            perception_component,
            planner_component,
            rsp_component,
        ]
    )

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        parameters=[
            px4_pluginlists_path,
            px4_config_path,
            {
                'fcu_url': '/dev/ttyACM0:2000000',
                'gcs_url': 'udp://@192.168.0.2:14550',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': 'v2.0',
                'respawn_mavros': 'false',
                'namespace': 'mavros',
            }
        ],
        arguments=['--ros-args', '--log-level', 'fatal']
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # map -> odom identity (no map alignment)
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
        output='screen'
    )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        condition=IfCondition(use_foxglove),
        output='screen',
        parameters=[{'port': 8765}]
    )

    return LaunchDescription([
        set_rmw,
        use_foxglove_arg,
        autonomy_stack_container,
        mavros_node,
        jsp_node,
        map_to_odom_tf,
        foxglove_bridge_node,
    ])
