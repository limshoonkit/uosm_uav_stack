#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # LaunchConfigurations for sensing/visualization
    init_x = LaunchConfiguration('init_x_', default=0.0)
    init_y = LaunchConfiguration('init_y_', default=0.0)
    init_z = LaunchConfiguration('init_z_', default=0.0)
    map_size_x_ = LaunchConfiguration('map_size_x_', default=30.0)
    map_size_y_ = LaunchConfiguration('map_size_y_', default=30.0)
    map_size_z_ = LaunchConfiguration('map_size_z_', default=5.0)
    c_num = LaunchConfiguration('c_num', default=5)
    p_num = LaunchConfiguration('p_num', default=20)
    min_dist = LaunchConfiguration('min_dist', default=1.0)
    odometry_topic = LaunchConfiguration('odometry_topic', default='/mavros/odometry/in')
    drone_id = LaunchConfiguration('drone_id', default=0)
    
    # LaunchConfiguration for MAVROS, RViz
    use_mavros = LaunchConfiguration('use_mavros')
    use_rviz = LaunchConfiguration('use_rviz')
    use_rosbag = LaunchConfiguration('use_rosbag')
    bag_output_path = LaunchConfiguration('bag_output_path')
    controller_mode = LaunchConfiguration('controller_mode')

    # Declare launch arguments for sensing/visualization
    init_x_arg = DeclareLaunchArgument('init_x_', default_value=init_x, description='Initial X position')
    init_y_arg = DeclareLaunchArgument('init_y_', default_value=init_y, description='Initial Y position')
    init_z_arg = DeclareLaunchArgument('init_z_', default_value=init_z, description='Initial Z position')
    map_size_x_arg = DeclareLaunchArgument('map_size_x_', default_value=map_size_x_, description='Map size X')
    map_size_y_arg = DeclareLaunchArgument('map_size_y_', default_value=map_size_y_, description='Map size Y')
    map_size_z_arg = DeclareLaunchArgument('map_size_z_', default_value=map_size_z_, description='Map size Z')
    c_num_arg = DeclareLaunchArgument('c_num', default_value=c_num, description='Circle number')
    p_num_arg = DeclareLaunchArgument('p_num', default_value=p_num, description='Polygon number')
    min_dist_arg = DeclareLaunchArgument('min_dist', default_value=min_dist, description='Minimum distance')
    odometry_topic_arg = DeclareLaunchArgument('odometry_topic', default_value=odometry_topic, description='Odometry topic')
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    
    # Declare launch argument to enable MAVROS
    use_mavros_arg = DeclareLaunchArgument(
        'use_mavros',
        default_value='true',
        description='Whether to launch MAVROS'
    )

    # Declare launch argument to enable RViz
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    use_rosbag_arg = DeclareLaunchArgument(
        'use_rosbag',
        default_value='false',
        description='Whether to record ROS2 bag',
        choices=['true', 'false']
    )

    controller_mode_arg = DeclareLaunchArgument(
        'controller_mode',
        default_value='feedforward',
        description='Controller mode: "feedforward" or "nmpc" (nmpc is beta/experimental)',
        choices=['feedforward', 'nmpc']
    )

    # Declare launch argument for bag output path
    bag_output_path_arg = DeclareLaunchArgument(
        'bag_output_path',
        default_value='bags/sim/test1', # relative
        description='Output directory for ROS2 bag files'
    )

    # Configuration paths for planner
    planner_dir = get_package_share_directory('planner_manager')
    fc_config = os.path.join(planner_dir, 'config', 'fc_params.yaml')
    planner_config = os.path.join(planner_dir, 'config', 'ego_planner_params.yaml')
    px4_pluginlists_path = os.path.join(planner_dir, 'config', 'mavros_pluginlists.yaml')
    px4_config_path = os.path.join(planner_dir, 'config', 'mavros_config.yaml')
    
    robot_viewer_package = get_package_share_directory('uosm_robot_viewer')
    xacro_path = os.path.join(robot_viewer_package, 'urdf', 'uosm_uav_platform.urdf.xacro')

    rviz_config_file = os.path.join(
        get_package_share_directory('uosm_uav_bringup'),
        'config', 'rviz',
        'simulation_test.rviz'
    )

    # Camera parameters from YAML file
    camera_params_file = os.path.join(
        get_package_share_directory('local_sensing'),
        'params',
        'camera.yaml'
    )

    # odom_visualization node
    odom_visualization_node = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name=['odom_visualization'],
        output='screen',
        remappings=[
            ('odom', odometry_topic),
            ('robot', 'vis/robot'),
            ('path', 'vis/path'),
            ('time_gap', 'vis/time_gap'),
        ],
        parameters=[
            {'color.a': 1.0},
            {'color.r': 0.0},
            {'color.g': 0.0},
            {'color.b': 0.0},
            {'covariance_scale': 100.0},
            {'robot_scale': 1.0},
            {'tf45': False},
            {'drone_id': drone_id}
        ]
    )

    # pcl_render node
    pcl_render_node = Node(
        package='local_sensing',
        executable='pcl_render_node',
        name=['pcl_render_node'],
        output='screen',
        parameters=[
            camera_params_file,
            {'sensing_horizon': 5.0},
            {'sensing_rate': 30.0},
            {'estimation_rate': 30.0},
            {'map.x_size': map_size_x_},
            {'map.y_size': map_size_y_},
            {'map.z_size': map_size_z_},
        ],
        remappings=[
            ('global_map', '/map_generator/global_cloud'),
            ('odometry', odometry_topic),
            ('pcl_render_node/cloud', '/pcl_render_node/cloud'),
        ]
    )

    # Flight controller component
    fc_component = ComposableNode(
        package='planner_manager',
        namespace='',
        plugin='uosm::path_planning::FlightController',
        name='flight_controller_node',
        parameters=[
            fc_config,
            {'fc.controller_mode': controller_mode}
        ],
        remappings=[
            ('odometry', odometry_topic)
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
            {"planner.waypoint_csv_file_path": ""},
            planner_config
        ],
        remappings=[
            ('odometry', odometry_topic),
            ('grid_map/cloud', '/pcl_render_node/cloud')
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    rsp_component = ComposableNode(
        package='robot_state_publisher',
        plugin='robot_state_publisher::RobotStatePublisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_path]),
            'package_path': robot_viewer_package
        }]
    )

    # Planner container
    planner_container = ComposableNodeContainer(
        name='planner_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        composable_node_descriptions=[
            fc_component,
            planner_component,
            rsp_component
        ]
    )

    # MAVROS node
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        condition=IfCondition(use_mavros),
        parameters=[
            px4_pluginlists_path,
            px4_config_path,
            {
                'fcu_url': 'udp://:14540@14557',
                'gcs_url': 'udp://@127.0.0.1:14550',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': "v2.0",
                'respawn_mavros': "false",
                'namespace': "mavros",
            }
        ],
        arguments=['--ros-args', '--log-level', 'error']  # 'debug', 'info', 'warn', 'error', 'fatal'
    )

    map_share_dir = get_package_share_directory('map_processor')
    map_pcd_file_path = os.path.join(map_share_dir, 'maps', 'klk', 'tuanmee_site.pcd')
    map_config_file_path = os.path.join(map_share_dir, 'config', 'map_processor_params.yaml')

    map_generator_node = Node(
        package='map_processor',
        executable='map_processor_node',
        name='map_processor_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        remappings=[
            ('global_cloud', '/map_generator/global_cloud'),
        ],
        parameters=[
            {'pcd_file_path': map_pcd_file_path},
            map_config_file_path,
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Static transform: map -> odom
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

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # ROS2 bag recording
    rosbag_record = ExecuteProcess(
        condition=IfCondition(use_rosbag),
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_output_path,
            # add topics
            '/ego_planner/poly_traj',
            'mavros/setpoint_raw/local',
            'mavros/odometry/in'
        ],
        output='screen',
    )

    return LaunchDescription([
        # Launch arguments
        init_x_arg,
        init_y_arg,
        init_z_arg,
        map_size_x_arg,
        map_size_y_arg,
        map_size_z_arg,
        c_num_arg,
        p_num_arg,
        min_dist_arg,
        odometry_topic_arg,
        drone_id_arg,
        use_mavros_arg,
        use_rviz_arg,
        use_rosbag_arg,
        bag_output_path_arg,
        controller_mode_arg,
        
        # Nodes
        odom_visualization_node,
        pcl_render_node,
        planner_container,
        mavros_node,
        map_to_odom_tf,
        map_generator_node,
        rviz_node,
        jsp_node,
        rosbag_record
    ])