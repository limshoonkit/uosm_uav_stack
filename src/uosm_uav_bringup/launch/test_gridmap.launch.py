#!/usr/bin/env python3
"""
Launch file for testing grid map visualization.

Usage:
  Hardware mode (default):
    ros2 launch uosm_uav_bringup test_gridmap.launch.py

  Bag mode (play bag in separate terminal):
    ros2 launch uosm_uav_bringup test_gridmap.launch.py use_bag:=true
    ros2 bag play <path_to_bag> --clock

Topics expected from bag:
  - /zed_node/point_cloud/cloud_registered (PointCloud2)
  - /mavros/odometry/out (Odometry)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

def generate_launch_description():
    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz2'
    )

    use_bag_arg = DeclareLaunchArgument(
        'use_bag',
        default_value='false',
        description='Use bag playback mode (no hardware). Play bag separately with: ros2 bag play <bag_path> --clock'
    )

    use_rviz = LaunchConfiguration('use_rviz')
    use_bag = LaunchConfiguration('use_bag')

    # Configuration paths
    urdf_dir = get_package_share_directory('uosm_robot_viewer')
    urdf_xacro_path = os.path.join(urdf_dir, 'urdf', 'uosm_uav_platform.urdf.xacro')

    bringup_package = get_package_share_directory('uosm_uav_bringup')
    planner_config = os.path.join(bringup_package, 'config', 'planner.yaml')
    zed_config_common = os.path.join(bringup_package, 'config', 'sensor_config', 'common_stereo.yaml')
    zed_config_camera = os.path.join(bringup_package, 'config', 'sensor_config', 'zedm.yaml')

    # Planner component for hardware mode
    planner_component_hw = ComposableNode(
        package='planner_manager',
        namespace='',
        plugin='uosm::path_planning::EgoPlanner',
        name='ego_planner_node',
        parameters=[
            {"planner.waypoint_csv_file_path": ""},
            planner_config
        ],
        remappings=[
            ('odometry', '/mavros/odometry/out'),
            ('grid_map/cloud', '/zed_node/point_cloud/cloud_registered'),
            ('grid_map/scan', '/scan')
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Planner component for bag mode (with use_sim_time)
    planner_component_bag = ComposableNode(
        package='planner_manager',
        namespace='',
        plugin='uosm::path_planning::EgoPlanner',
        name='ego_planner_node',
        parameters=[
            {"planner.waypoint_csv_file_path": ""},
            {"use_sim_time": True},
            planner_config
        ],
        remappings=[
            ('odometry', '/mavros/odometry/out'),
            ('grid_map/cloud', '/zed_node/point_cloud/cloud_registered'),
            ('grid_map/scan', '/scan')
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Stereo Camera component (hardware only)
    zed_component = ComposableNode(
        package='zed_components',
        plugin='stereolabs::ZedCamera',
        name='zed_node',
        parameters=[
            zed_config_common,
            zed_config_camera,
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # 2D LiDAR component (hardware only)
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

    # Perception component (hardware only)
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

    rsp_component_bag = ComposableNode(
        package='robot_state_publisher',
        plugin='robot_state_publisher::RobotStatePublisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_xacro_path]),
            'package_path': bringup_package,
            'use_sim_time': True
        }]
    )

    # Hardware mode container (all sensors + planner)
    hardware_container = ComposableNodeContainer(
        name='autonomy_stack_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        condition=UnlessCondition(use_bag),
        composable_node_descriptions=[
            zed_component,
            rplidar_component,
            perception_component,
            planner_component_hw,
            rsp_component
        ]
    )

    # Bag mode container (only planner + robot state publisher)
    bag_container = ComposableNodeContainer(
        name='autonomy_stack_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        condition=IfCondition(use_bag),
        composable_node_descriptions=[
            planner_component_bag,
            rsp_component_bag
        ]
    )

    # Joint State Publisher
    jsp_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration('use_bag')}]
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
        parameters=[{"use_sim_time": LaunchConfiguration('use_bag')}],
        output='screen'
    )

    # Odom -> base_link TF broadcaster (bag mode only)
    odom_to_tf_node = Node(
        package='uosm_uav_bringup',
        executable='odom_to_tf_node.py',
        name='odom_to_tf_node',
        output='screen',
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_bag)
    )

    rviz_config_path = os.path.join(bringup_package, 'config', 'rviz', 'gridmap_test.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{"use_sim_time": LaunchConfiguration('use_bag')}],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        use_rviz_arg,
        use_bag_arg,
        hardware_container,
        bag_container,
        jsp_node,
        map_to_odom_tf,
        odom_to_tf_node,
        rviz_node
    ])
