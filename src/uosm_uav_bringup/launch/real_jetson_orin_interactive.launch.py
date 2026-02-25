#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

def generate_launch_description():    
    # LaunchConfiguration
    use_mavros = LaunchConfiguration('use_mavros')
    use_foxglove = LaunchConfiguration('use_foxglove')
    use_rosbag = LaunchConfiguration('use_rosbag')
    bag_output_path = LaunchConfiguration('bag_output_path')

    # Declare launch argument to enable MAVROS
    use_mavros_arg = DeclareLaunchArgument(
        'use_mavros',
        default_value='true',
        description='Whether to launch MAVROS',
        choices=['true', 'false']
    )

    # Declare launch argument to enable foxglove
    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='true',
        description='Whether to launch Foxglove Bridge',
        choices=['true', 'false']
    )

    # Declare launch argument to enable ROS2 bag recording
    use_rosbag_arg = DeclareLaunchArgument(
        'use_rosbag',
        default_value='false',
        description='Whether to record ROS2 bag',
        choices=['true', 'false']
    )

    # Declare launch argument for bag output path
    bag_output_path_arg = DeclareLaunchArgument(
        'bag_output_path',
        default_value='bags/test1', # relative
        description='Output directory for ROS2 bag files'
    )

    # Configuration paths
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
    mcap_options = os.path.join(bringup_package, 'config', 'mcap_writer_options.yaml')
    qos_overrides = os.path.join(bringup_package, 'config', 'qos_overrides.yaml')

    
    # Flight controller component
    fc_component = ComposableNode(
        package='planner_manager',
        namespace='',
        plugin='uosm::path_planning::FlightController',
        name='flight_controller_node',
        parameters=[fc_config],
        remappings=[
            ('odometry', "mavros/odometry/out"), # fused odom from sensors
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Planner component
    planner_component = ComposableNode(
        package='planner_manager',
        namespace='',
        plugin='uosm::path_planning::EgoPlanner',
        name='ego_planner_node',
        parameters=[
            {"planner.waypoint_csv_file_path": ""}, # Not specified here, using interactive mode
            planner_config
        ],
        remappings=[
            ('odometry', "mavros/odometry/out"), # combined odom from sensors
            ('grid_map/cloud', 'zed_node/point_cloud/cloud_registered'), # point cloud from stereo camera
            ('grid_map/scan', '/scan') # 2D LiDAR scan for grid map fusion
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    zed_node_parameters = [zed_config_common, zed_config_camera]
    # zed_node_parameters.append(
    #     {
    #         'zed_node.left.image_rect_color.ffmpeg.encoder': 'h264_nvmpi',
    #         'zed_node.left.image_rect_color.ffmpeg.gop_size': 10,
    #         'zed_node.left.image_rect_color.ffmpeg.bit_rate': 2000000,
    #         'zed_node.left.image_rect_color.ffmpeg.qmax': 10,
    #         'zed_node.left.image_rect_color.ffmpeg.encoder_av_options': 'profile:main',

    #         'zed_node.right.image_rect_color.ffmpeg.encoder': 'h264_nvmpi',
    #         'zed_node.right.image_rect_color.ffmpeg.gop_size': 10,
    #         'zed_node.right.image_rect_color.ffmpeg.bit_rate': 2000000,
    #         'zed_node.right.image_rect_color.ffmpeg.qmax': 10,
    #         'zed_node.right.image_rect_color.ffmpeg.encoder_av_options': 'profile:main',
    #     }
    # )

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
            'package_path': bringup_package
        }]
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
            fc_component,
            zed_component,
            rplidar_component,
            perception_component,
            planner_component,
            rsp_component
        ]
    )

    jetson_gscam2_dir = get_package_share_directory('jetson_gscam2')
    csi_cam_config = os.path.join(jetson_gscam2_dir, 'config', 'csi_cam_params.yaml')
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

    # CSI camera container
    csi_camera_container = ComposableNodeContainer(
        name='csi_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        composable_node_descriptions=[
            csi_camera_component
        ]
    )

    # MAVROS
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        condition=IfCondition(use_mavros),
        parameters=[
            px4_pluginlists_path,
            px4_config_path,
            {
                'fcu_url': '/dev/ttyACM0:2000000',
                #'fcu_url': '/dev/ttyTHS1:1152000',
                #'gcs_url': 'udp://@127.0.0.1:14550',
                'gcs_url': 'udp://@192.168.1.100:14550',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': "v2.0",
                'respawn_mavros': "false",
                'namespace': "mavros",
            }
        ],
        arguments=['--ros-args', '--log-level', 'warn']  # 'debug', 'info', 'warn', 'error', 'fatal'
    )

    # Joint State Publisher
    jsp_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen"
    )

    # Foxglove bridge
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        condition=IfCondition(use_foxglove),
        output='screen',
        parameters=[{
            "port": 8765,
            # Optional: "allowed_origins": ["*"] or specific domains
        }]
    )

    # Jetson Stats Node
    jetson_stats_node = Node(
        package='ros2_jetson_stats',
        executable='ros2_jtop',
        name='ros2_jtop',
        parameters=[
            {'interval': 1.0} # 0.1 ~ 10Hz max
        ],
        output='screen',
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

    # ROS2 bag recording
    rosbag_record = ExecuteProcess(
        condition=IfCondition(use_rosbag),
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_output_path,
            '--storage', 'mcap', # mcap, sqlite3
            '--storage-config-file', mcap_options,
            '--qos-profile-overrides-path', qos_overrides,
            # add topics
            'diagnostics',
            'ego_planner/poly_traj',
            'zed_node/point_cloud/cloud_registered',
            'zed_node/left/color/rect/image',
            'zed_node/right/color/rect/image',
            'mavros/odometry/out',
            'mavros/imu/data',
            'mavros/hps167_pub',
            'csi_cam/image_raw/compressed',
            'scan',
        ],
        output='screen',
    )

    return LaunchDescription([
        # Launch arguments
        use_mavros_arg,
        use_foxglove_arg,
        use_rosbag_arg,
        bag_output_path_arg,
        
        # Nodes
        autonomy_stack_container,
        csi_camera_container,
        mavros_node,
        jsp_node,
        jetson_stats_node,
        map_to_odom_tf,
        rosbag_record,
        # foxglove_bridge_node
    ])