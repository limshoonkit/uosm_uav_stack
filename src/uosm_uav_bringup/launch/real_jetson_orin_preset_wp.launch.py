#!/usr/bin/env python3i
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
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

    # Launch argument for map alignment integration
    wait_for_alignment_arg = DeclareLaunchArgument(
        'wait_for_alignment',
        default_value='true',
        description='Whether to wait for map alignment before starting planner',
        choices=['true', 'false']
    )
    wait_for_alignment = LaunchConfiguration('wait_for_alignment')

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
    
    waypoint_dir = get_package_share_directory('map_processor')
    waypoint_config = os.path.join(waypoint_dir, 'maps', 'uosm_indoor.csv') # NOTE: Change preset wp here!

    trunk_seg_dir = get_package_share_directory('trunk_segmentation')
    trunk_seg_config = os.path.join(trunk_seg_dir, 'config', 'trunk_segmentation_params.yaml')

    map_align_dir = get_package_share_directory('map_alignment')
    map_align_config = os.path.join(map_align_dir, 'config', 'map_alignment_params.yaml')
    map_pcd_path = os.path.join(map_align_dir, 'maps', 'tuanmee_site.pcd')

    # Trunk segmentation component
    trunk_seg_component = ComposableNode(
        package='trunk_segmentation',
        namespace='',
        plugin='uosm::perception::TrunkSegmentationComponent',
        name='trunk_segmentation_node',
        parameters=[trunk_seg_config],
        remappings=[
            ('trunk_seg/scan', '/scan'),
            ('trunk_seg/odom', '/mavros/odometry/out'),
            ('trunk_observations', '/trunk_observations'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Map alignment component
    map_align_component = ComposableNode(
        package='map_alignment',
        namespace='',
        plugin='uosm::perception::MapAlignmentComponent',
        name='map_alignment_node',
        parameters=[
            {'pcd_file_path': map_pcd_path},
            {'container_name': 'autonomy_stack_container'},
            map_align_config,
        ],
        remappings=[
            ('map_alignment_markers', '/map_alignment_markers'),
            ('observed_landmarks', '/trunk_observations'),
            ('alignment_done', '/map_alignment/alignment_done'),
            ('alignment_transform', '/map_alignment/alignment_transform'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Flight controller component
    fc_component = ComposableNode(
        package='planner_manager',
        namespace='',
        plugin='uosm::path_planning::FlightController',
        name='flight_controller_node',
        parameters=[
            fc_config,
            {'fc.wait_for_alignment': wait_for_alignment}
        ],
        remappings=[
            ('odometry', "mavros/odometry/out"), # fused odom from sensors
            ('alignment_done', '/map_alignment/alignment_done'),
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
            {"planner.waypoint_csv_file_path": waypoint_config},
            planner_config
        ],
        remappings=[
            ('odometry', "mavros/odometry/out"), # fused odom from sensors
            ('grid_map/cloud', 'zed_node/point_cloud/cloud_registered'), # point cloud from stereo camera
            ('grid_map/scan', '/scan'), # 2D LiDAR scan for grid map fusion
            ('alignment_done', '/map_alignment/alignment_done'),
            ('alignment_transform', '/map_alignment/alignment_transform'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Sensor component
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

    # Conditionally load trunk segmentation and map alignment only when waiting for alignment
    load_alignment_nodes = LoadComposableNodes(
        target_container='autonomy_stack_container',
        condition=IfCondition(wait_for_alignment),
        composable_node_descriptions=[
            trunk_seg_component,
            map_align_component,
        ]
    )

    jetson_gscam2_dir = get_package_share_directory('jetson_gscam2')
    csi_cam_config = os.path.join(jetson_gscam2_dir, 'config', 'wide_angle_preset.yaml')
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
                'gcs_url': 'udp://@192.168.0.2:14550',
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

    # Static transform: map -> odom (identity transform when NOT using map alignment)
    # When wait_for_alignment is true, the map_alignment node will publish the actual transform
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        condition=UnlessCondition(wait_for_alignment),  # Only run when NOT waiting for alignment
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
    rosbag_record_cmd = ExecuteProcess(
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
            'zed_node/left/color/rect/image/camera_info',
            'zed_node/right/color/rect/image/camera_info',
            'mavros/odometry/out',
            'mavros/imu/data',
            'mavros/hps167_pub',
            'csi_cam/image_raw/compressed',
            'csi_cam/camera_info',
            'scan',
        ],
        output='screen',
    )

    rosbag_record = TimerAction(
        period=10.0,  # 10 second delay for sensor initialization
        actions=[rosbag_record_cmd],
        condition=IfCondition(use_rosbag)
    )

    return LaunchDescription([
        # Launch arguments
        use_mavros_arg,
        use_foxglove_arg,
        use_rosbag_arg,
        bag_output_path_arg,
        wait_for_alignment_arg,

        # Nodes
        autonomy_stack_container,
        load_alignment_nodes,
        csi_camera_container,
        mavros_node,
        jsp_node,
        jetson_stats_node,
        map_to_odom_tf,
        # rosbag_record,
        # foxglove_bridge_node
    ])
