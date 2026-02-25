from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_rosbag = LaunchConfiguration('use_rosbag')
    bag_path = LaunchConfiguration('bag_path')
    
    use_rosbag_arg = DeclareLaunchArgument(
        'use_rosbag',
        default_value='true',
        description='Whether to play rosbag for testing'
    )
    
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='./bags/test_distance',
        description='Path to the rosbag to play'
    )

    robot_viewer_dir = get_package_share_directory('uosm_robot_viewer')
    xacro_file_path = PathJoinSubstitution([robot_viewer_dir, 'urdf', 'uosm_uav_platform.urdf.xacro'])

    bringup_package = get_package_share_directory('uosm_uav_bringup')
    planner_config = os.path.join(bringup_package, 'config', 'planner.yaml')
    planner_component = ComposableNode(
        package='planner_manager',
        namespace='',
        plugin='uosm::path_planning::EgoPlanner',
        name='ego_planner_node',
        parameters=[
            {"planner/waypoint_csv_file_path": ""},
            planner_config
        ],
        remappings=[
            ('odometry', "mavros/odometry/out"), # odometry
            ('grid_map/cloud', 'zed_node/point_cloud/cloud_registered'),
            ('alignment_done', '/map_alignment/alignment_done'),
            ('alignment_transform', '/map_alignment/alignment_transform'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file_path])
        }]
    )
        
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )
        

    robot_viewer_node = Node(
        package='uosm_robot_viewer',
        executable='odom_to_tf_publisher.py',
        name='odom_to_tf_publisher',
        output='screen',
    )

    trunk_seg_dir = get_package_share_directory('trunk_segmentation')
    trunk_seg_config = os.path.join(trunk_seg_dir, 'config', 'trunk_segmentation_params.yaml')
    trunk_seg_component = ComposableNode(
        package='trunk_segmentation',
        namespace='',
        plugin='uosm::perception::TrunkSegmentationComponent',
        name='trunk_segmentation_node',
        parameters=[
            trunk_seg_config,
        ],
        remappings=[
            ('trunk_seg/scan', '/scan'),
            ('trunk_seg/odom', '/mavros/odometry/out'),
            ('trunk_observations', '/trunk_observations'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    map_align_dir = get_package_share_directory('map_alignment')
    map_align_config = os.path.join(map_align_dir, 'config', 'map_alignment_params.yaml')
    map_pcd_path = os.path.join(map_align_dir, 'maps', 'klk', 'tuanmee_site.pcd')
    map_align_component = ComposableNode(
        package='map_alignment',
        namespace='',
        plugin='uosm::perception::MapAlignmentComponent',
        name='map_alignment_node',
        parameters=[
            {'pcd_file_path': map_pcd_path},
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

    container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[
            planner_component,
            trunk_seg_component, 
            map_align_component],
        output='screen',
    )

    rosbag_playback = ExecuteProcess(
        condition=IfCondition(use_rosbag),
        cmd=['ros2', 'bag', 'play', bag_path, '--clock'],
        output='screen',
    )

    rviz_config_path = os.path.join(robot_viewer_dir, 'rviz', 'viewer_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
    )

    return LaunchDescription([
        # Launch arguments
        use_rosbag_arg,
        bag_path_arg,
        
        # Nodes
        container,
        rsp_node,
        jsp_node,
        robot_viewer_node,
        rosbag_playback,
        rviz_node
    ])