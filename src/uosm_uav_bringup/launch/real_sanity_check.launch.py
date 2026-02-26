#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import ComposableNodeContainer, Node
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
    use_gridmap = LaunchConfiguration('use_gridmap')

    # Declare launch argument to enable ROS2 bag recording
    use_rosbag_arg = DeclareLaunchArgument(
        'use_rosbag',
        default_value='true',
        description='Whether to record ROS2 bag',
        choices=['true', 'false']
    )

    # Declare launch argument for bag output path
    bag_output_path_arg = DeclareLaunchArgument(
        'bag_output_path',
        default_value='bags/real/test1', # relative
        description='Output directory for ROS2 bag files'
    )

    # Declare launch argument for rosbag startup delay
    rosbag_delay_arg = DeclareLaunchArgument(
        'rosbag_delay',
        default_value='10.0',
        description='Delay in seconds before starting rosbag recording (wait for sensors)'
    )

    # Declare launch argument to enable Foxglove websocket bridge
    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='true',
        description='Whether to launch Foxglove websocket bridge for visualization',
        choices=['true', 'false']
    )

    use_gridmap_arg = DeclareLaunchArgument(
        'use_gridmap',
        default_value='true',
        description='Whether to run grid map only (EgoPlanner with empty waypoints, no path planning)',
        choices=['true', 'false']
    )

    # Use CycloneDDS with default network settings (allows Foxglove websocket to work)
    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

    # Configuration paths
    urdf_dir = get_package_share_directory('uosm_robot_viewer')
    urdf_xacro_path = os.path.join(urdf_dir, 'urdf', 'uosm_uav_platform.urdf.xacro')

    planner_dir = get_package_share_directory('planner_manager')
    px4_pluginlists_path = os.path.join(planner_dir, 'config', 'mavros_pluginlists.yaml')
    px4_config_path = os.path.join(planner_dir, 'config', 'mavros_config.yaml')

    bringup_package = get_package_share_directory('uosm_uav_bringup')
    planner_config = os.path.join(bringup_package, 'config', 'planner.yaml')
    zed_config_common = os.path.join(bringup_package, 'config', 'sensor_config', 'common_stereo.yaml')
    zed_config_camera = os.path.join(bringup_package, 'config', 'sensor_config', 'zedm.yaml')
    zed_config_camera = os.path.join(bringup_package, 'config', 'sensor_config', 'zedm_gen3.yaml')

    mcap_options = os.path.join(bringup_package, 'config', 'mcap_writer_options.yaml')
    qos_overrides = os.path.join(bringup_package, 'config', 'qos_overrides.yaml')

    zed_node_parameters = [zed_config_common, zed_config_camera]

    # Sensor component
    zed_component = ComposableNode(
        package='zed_components',
        plugin='stereolabs::ZedCamera',
        name='zed_node',
        parameters=zed_node_parameters,
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # RPLIDAR component
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

    # Odom republisher: transforms VIO odom from camera frame -> base_link
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
            'package_path': bringup_package
        }]
    )

    # Grid map only: EgoPlanner with empty waypoints (same as test_gridmap, no path planning)
    gridmap_planner_component = ComposableNode(
        package='planner_manager',
        namespace='',
        plugin='uosm::path_planning::EgoPlanner',
        name='ego_planner_node',
        parameters=[
            {'planner.waypoint_csv_file_path': ''},
            planner_config,
        ],
        remappings=[
            ('odometry', '/mavros/odometry/out'),
            ('grid_map/cloud', '/zed_node/point_cloud/cloud_registered'),
            ('grid_map/scan', '/scan'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    jetson_gscam2_dir = get_package_share_directory('jetson_gscam2')
    csi_cam_config = os.path.join(jetson_gscam2_dir, 'config', 'streaming_1080p_preset.yaml')
    csi_cam_info_url = 'package://jetson_gscam2/config/camera_calibration.yaml'

    # CSI camera component
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

    # Autonomy stack container
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
            rsp_component,
        ]
    )

    # CSI camera in separate container to isolate from ZED/autonomy stack
    csi_camera_container = ComposableNodeContainer(
        name='csi_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        composable_node_descriptions=[
            csi_camera_component,
        ]
    )

    # Grid map only container (EgoPlanner with empty waypoints; subscribes to odom, point cloud, scan)
    gridmap_container = ComposableNodeContainer(
        name='gridmap_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        condition=IfCondition(use_gridmap),
        composable_node_descriptions=[
            gridmap_planner_component,
        ],
    )

    # MAVROS
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        parameters=[
            px4_pluginlists_path,
            px4_config_path,
            {
                'fcu_url': '/dev/ttyACM0:2000000',
                #'fcu_url': '/dev/ttyTHS1:1152000',
                #'gcs_url': 'udp://@127.0.0.1:14550',
                'gcs_url': 'udp://@192.168.0.2:14550',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': "v2.0",
                'respawn_mavros': "false",
                'namespace': "mavros",
            }
        ],
        arguments=['--ros-args', '--log-level', 'error']  # 'debug', 'info', 'error', 'fatal'
    )

    # Jetson Stats Node
    jetson_stats_node = Node(
        package='ros2_jetson_stats',
        executable='ros2_jtop',
        name='ros2_jtop',
        condition=IfCondition(use_rosbag),
        parameters=[
            {'interval': 1.0} # 0.1 ~ 10Hz max
        ],
        output='screen',
    )

    # Joint State Publisher
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
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom'
        ],
        output='screen'
    )

    # Foxglove websocket bridge for visualization
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        condition=IfCondition(use_foxglove),
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
        }],
        output='screen',
    )

    # ROS2 bag recording with delay for sensor initialization
    rosbag_record_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_output_path,
            '--storage', 'mcap', # mcap, sqlite3
            '--storage-config-file', mcap_options,
            '--qos-profile-overrides-path', qos_overrides,
            # add topics
            'diagnostics',
            # ZED
            'zed_node/odom',
            'zed_node/point_cloud/cloud_registered',
            'zed_node/left/color/rect/image',
            'zed_node/right/color/rect/image',
            'zed_node/left/color/rect/image/camera_info',
            'zed_node/right/color/rect/image/camera_info',
            # Mavros
            'mavros/odometry/out',
            'mavros/imu/data',
            'mavros/hps167_pub',
            # CSI camera
            'csi_cam/image_raw/compressed',
            'csi_cam/camera_info',
            # 2D lidar raw scan
            'scan',
            # Grid map (when use_gridmap:=true)
            'grid_map/occupancy_inflate',
            # TF
            '/tf',
            '/tf_static',
        ],
        output='screen',
    )

    rosbag_record = TimerAction(
        period=10.0,  # 10 second delay for sensor initialization
        actions=[rosbag_record_cmd],
        condition=IfCondition(use_rosbag)
    )

    return LaunchDescription([
        # Use CycloneDDS with default network settings
        set_rmw,

        # Launch arguments
        use_rosbag_arg,
        bag_output_path_arg,
        rosbag_delay_arg,
        use_foxglove_arg,
        use_gridmap_arg,

        # Nodes
        autonomy_stack_container,
        gridmap_container,
        csi_camera_container,
        mavros_node,
        jsp_node,
        map_to_odom_tf,
        jetson_stats_node,
        foxglove_bridge,
        rosbag_record,
    ])
