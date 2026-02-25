import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # Declare launch argument to enable MAVROS
    use_mavros_arg = DeclareLaunchArgument(
        'use_mavros',
        default_value='true',
        description='Whether to launch MAVROS'
    )

    use_mavros = LaunchConfiguration('use_mavros')

    # Planner manager config
    planner_dir = get_package_share_directory('planner_manager')
    fc_config = os.path.join(planner_dir, 'config', 'fc_params.yaml')
    planner_config = os.path.join(planner_dir, 'config', 'ego_planner_params.yaml')
    px4_pluginlists_path = os.path.join(planner_dir, 'config', 'mavros_pluginlists.yaml')
    px4_config_path = os.path.join(planner_dir, 'config', 'mavros_config.yaml')

    waypoint_dir = get_package_share_directory('map_processor')
    waypoint_config = os.path.join(waypoint_dir, 'maps', 'sample_data3.csv')

    fc_component = ComposableNode(
        package='planner_manager',
        namespace='',
        plugin='uosm::path_planning::FlightController',
        name='flight_controller_node',
        parameters=[fc_config],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    planner_component = ComposableNode(
        package='planner_manager',
        namespace='',
        plugin='uosm::path_planning::EgoPlanner',
        name='ego_planner_node',
        parameters=[
            {"planner.waypoint_csv_file_path" : waypoint_config},
            planner_config
            ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    planner_container = ComposableNodeContainer(
        name='planner_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        composable_node_descriptions=[
            fc_component,
            planner_component
            ]
    )

    mavros = Node(
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
        }],
    )

    ld = LaunchDescription()

    ld.add_action(use_mavros_arg)
    ld.add_action(planner_container)
    ld.add_action(mavros)

    return ld
