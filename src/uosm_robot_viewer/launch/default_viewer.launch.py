import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Paths
    pkg_dir = get_package_share_directory('uosm_robot_viewer')
    
    # RViz configuration
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'urdf_config.rviz')
    
    xacro_file_path = PathJoinSubstitution([
        pkg_dir,
        'urdf',
        LaunchConfiguration('xacro_file_name')
    ])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        
        DeclareLaunchArgument(
            'xacro_file_name',
            default_value='uosm_uav_platform.urdf.xacro',
            description='Name of the XACRO file in the urdf directory'),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro ', xacro_file_path])
            }]),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }]),
        

        Node(
            package='uosm_robot_viewer',
            executable='odom_to_tf_publisher.py',
            name='odom_to_tf_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Static map to odom transform (needed for complete TF tree)
        Node(
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
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config_path]
        )
    ])