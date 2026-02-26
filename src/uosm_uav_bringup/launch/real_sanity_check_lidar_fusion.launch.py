#!/usr/bin/env python3
"""Same as real_sanity_check but with 2D lidar fusion: KISS-ICP + scan_to_pointcloud + tilt_gated_odom + odom_republisher fuse_lidar_2d."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

def generate_launch_description():
    # LaunchConfiguration
    use_rosbag = LaunchConfiguration('use_rosbag')
    bag_output_path = LaunchConfiguration('bag_output_path')
    rosbag_delay = LaunchConfiguration('rosbag_delay')
    use_foxglove = LaunchConfiguration('use_foxglove')

    use_rosbag_arg = DeclareLaunchArgument(
        'use_rosbag',
        default_value='true',
        description='Whether to record ROS2 bag',
        choices=['true', 'false']
    )
    bag_output_path_arg = DeclareLaunchArgument(
        'bag_output_path',
        default_value='bags/real/test1_lidar_fusion',
        description='Output directory for ROS2 bag files'
    )
    rosbag_delay_arg = DeclareLaunchArgument(
        'rosbag_delay',
        default_value='10.0',
        description='Delay in seconds before starting rosbag recording (wait for sensors)'
    )
    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='true',
        description='Whether to launch Foxglove websocket bridge for visualization',
        choices=['true', 'false']
    )

    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

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

    zed_node_parameters = [zed_config_common, zed_config_camera]

    zed_component = ComposableNode(
        package='zed_components',
        plugin='stereolabs::ZedCamera',
        name='zed_node',
        parameters=zed_node_parameters,
        extra_arguments=[{'use_intra_process_comms': True}]
    )

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

    # Odom republisher with 2D lidar fusion enabled
    odom_republisher_component = ComposableNode(
        package='odom_republisher',
        namespace='',
        plugin='uosm::perception::OdomRepublisher',
        name='odom_republisher_node',
        parameters=[{
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'camera_frame': 'zedm_camera_link',
            'broadcast_tf': True,
            'fuse_lidar_2d': True,
        }],
        remappings=[
            ('/vio/odom', '/zed_node/odom'),
            ('/odom/out', '/mavros/odometry/out'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

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
            ('orientation', '/odom/out'),
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

    jetson_gscam2_dir = get_package_share_directory('jetson_gscam2')
    csi_cam_config = os.path.join(jetson_gscam2_dir, 'config', 'wide_angle_preset.yaml')
    csi_cam_info_url = 'package://jetson_gscam2/config/camera_calibration.yaml'

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
            csi_camera_component,
        ]
    )

    # KISS-ICP (from submodule src/third_party/kiss-icp/ros)
    kiss_icp_dir = get_package_share_directory('kiss_icp')
    kiss_icp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(kiss_icp_dir, 'launch', 'odometry.launch.py')),
        launch_arguments=[
            ('topic', '/scan_cloud'),
            ('visualize', 'false'),
            ('use_sim_time', 'false'),
            ('base_frame', 'base_link'),
            ('lidar_odom_frame', 'odom'),
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
                'fcu_protocol': "v2.0",
                'respawn_mavros': "false",
                'namespace': "mavros",
            }
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    jetson_stats_node = Node(
        package='ros2_jetson_stats',
        executable='ros2_jtop',
        name='ros2_jtop',
        condition=IfCondition(use_rosbag),
        parameters=[{'interval': 1.0}],
        output='screen',
    )

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
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'odom'
        ],
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

    rosbag_record_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_output_path,
            '--storage', 'mcap',
            '--storage-config-file', mcap_options,
            '--qos-profile-overrides-path', qos_overrides,
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
            'scan_cloud',
            'odom/lidar_2d',
            'kiss/odometry',
        ],
        output='screen',
    )

    rosbag_record = TimerAction(
        period=10.0,
        actions=[rosbag_record_cmd],
        condition=IfCondition(use_rosbag)
    )

    return LaunchDescription([
        set_rmw,
        use_rosbag_arg,
        bag_output_path_arg,
        rosbag_delay_arg,
        use_foxglove_arg,
        autonomy_stack_container,
        kiss_icp_launch,
        mavros_node,
        jsp_node,
        map_to_odom_tf,
        jetson_stats_node,
        foxglove_bridge,
        rosbag_record,
    ])
