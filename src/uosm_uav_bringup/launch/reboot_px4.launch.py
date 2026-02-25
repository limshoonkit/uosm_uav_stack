import os

from launch import LaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_mavros = 'true'

    planner_dir = get_package_share_directory('planner_manager')
    px4_pluginlists_path = os.path.join(planner_dir, 'config', 'mavros_pluginlists.yaml')
    px4_config_path = os.path.join(planner_dir, 'config', 'mavros_config.yaml')

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        condition=IfCondition(use_mavros),
        parameters=[
            px4_pluginlists_path,
            px4_config_path,
            {
                'fcu_url': '/dev/ttyACM0:2000000',
                # 'fcu_url': '/dev/ttyTHS1:1152000',
                'gcs_url': 'udp://@192.168.1.100',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': "v2.0",
                'respawn_mavros': "false",
                'namespace': "mavros",
            }
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    reboot_node = TimerAction(
        period=5.0,  # wait 5 seconds to allow MAVROS to connect
        actions=[
            Node(
                package='uosm_uav_bringup',
                executable='reboot_px4_node.py',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        mavros_node,
        reboot_node,
    ])
