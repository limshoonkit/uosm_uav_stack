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
    bag_output_path = LaunchConfiguration('bag_output_path')

    # Declare launch argument for bag output path
    bag_output_path_arg = DeclareLaunchArgument(
        'bag_output_path',
        default_value='bags/calib', # relative
        description='Output directory for ROS2 bag files'
    )

    # Configuration paths
    planner_dir = get_package_share_directory('planner_manager')
    px4_pluginlists_path = os.path.join(planner_dir, 'config', 'mavros_pluginlists.yaml')
    px4_config_path = os.path.join(planner_dir, 'config', 'mavros_config.yaml')
    
    bringup_package = get_package_share_directory('uosm_uav_bringup')
    zed_config_common = os.path.join(bringup_package, 'config', 'sensor_config', 'common_stereo.yaml')
    zed_config_camera = os.path.join(bringup_package, 'config', 'sensor_config', 'zedm.yaml')
    qos_overrides = os.path.join(bringup_package, 'config', 'qos_overrides.yaml')

    urdf_dir = get_package_share_directory('uosm_robot_viewer')
    urdf_xacro_path = os.path.join(urdf_dir, 'urdf', 'uosm_uav_calib.urdf.xacro')

    # Camera component
    zed_component = ComposableNode(
        package='zed_components',
        plugin='stereolabs::ZedCamera',
        name='zed_node',
        parameters=[
        # YAML files
            zed_config_common,  # Common parameters
            zed_config_camera,  # Camera related parameters
        ],
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

    rsp_component = ComposableNode(
        package='robot_state_publisher',
        plugin='robot_state_publisher::RobotStatePublisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_xacro_path]),
            'package_path': bringup_package
        }]
    )

    perception_module_container = ComposableNodeContainer(
        name='perception_module_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        composable_node_descriptions=[
            zed_component,
            rplidar_component,
            rsp_component,
        ]
    )

    jetson_gscam2_dir = get_package_share_directory('jetson_gscam2')
    csi_cam_config = os.path.join(jetson_gscam2_dir, 'config', 'default_preset.yaml')

    # CSI camera component
    csi_camera_component = ComposableNode(
        package='jetson_gscam2',
        plugin='uosm::perception::CSICameraComponent',
        name='csi_camera_node',
        namespace='',
        parameters=[
            csi_cam_config,
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
        parameters=[
            px4_pluginlists_path,
            px4_config_path,
            {
                'fcu_url': '/dev/ttyACM0:2000000',
                #'fcu_url': '/dev/ttyTHS1:1152000',
                'gcs_url': 'udp://@127.0.0.1:14550',
                # 'gcs_url': 'udp://@192.168.1.100:14550',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': "v2.0",
                'respawn_mavros': "false",
                'namespace': "mavros",
            }
        ],
        arguments=['--ros-args', '--log-level', 'error']  # 'debug', 'info', 'error', 'fatal'
    )

    # ROS2 bag recording
    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_output_path,
            '--compression-mode', 'file',
            '--compression-format', 'zstd',
            '--qos-profile-overrides-path', qos_overrides,
            # add topics
            'zed_node/right/color/rect/image',
            'zed_node/left/color/rect/image',
            # 'zed_node/point_cloud/cloud_registered',
            'mavros/imu/data',
            # 'csi_cam/image_raw/compressed',
            # 'scan'
        ],
        output='screen',
    )

    return LaunchDescription([
        # Launch arguments
        bag_output_path_arg,
        
        # Nodes
        perception_module_container,
        # csi_camera_container,
        mavros_node,
        rosbag_record,
    ])
