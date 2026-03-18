#!/usr/bin/env python3
"""
Replay a bag through trunk_segmentation + map_alignment and record
detection topics for offline evaluation.

Usage (ground-truth-free, no alignment):
  ros2 launch uosm_uav_bringup evaluate_trunk_detection.launch.py \
      bag_path:=./bags/real/klk/test_manual_1_corrected \
      odom_topic:=/odom/corrected

Usage (with map alignment for mutual map-vs-detection evaluation):
  ros2 launch uosm_uav_bringup evaluate_trunk_detection.launch.py \
      bag_path:=./bags/real/klk/test_manual_1_corrected \
      odom_topic:=/odom/corrected \
      use_alignment:=true \
      initial_yaw_deg:=0.0

Output bag is written to <bag_path>_eval/ with topics:
  /trunk_observations                (TrunkObservationArray)
  /active_tracks                     (MarkerArray)
  /lost_tracks                       (MarkerArray)
  /map_alignment/alignment_transform (TransformStamped, if alignment enabled)
  /map_alignment/alignment_done      (Bool, if alignment enabled)
"""

import math
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

RECORD_TOPICS = [
    "/trunk_observations",
    "/active_tracks",
    "/lost_tracks",
    "/map_alignment/alignment_transform",
    "/map_alignment/alignment_done",
]


def _launch_setup(context, *args, **kwargs):
    bag_path = os.path.expanduser(
        LaunchConfiguration("bag_path").perform(context))
    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    use_alignment = LaunchConfiguration(
        "use_alignment").perform(context).lower() == "true"
    initial_yaw_deg = float(
        LaunchConfiguration("initial_yaw_deg").perform(context))
    initial_yaw_rad = initial_yaw_deg * math.pi / 180.0
    out_bag = bag_path.rstrip("/") + "_eval"

    trunk_seg_dir = get_package_share_directory("trunk_segmentation")
    trunk_seg_config = os.path.join(
        trunk_seg_dir, "config", "trunk_segmentation_params.yaml")

    trunk_seg_component = ComposableNode(
        package="trunk_segmentation",
        plugin="uosm::perception::TrunkSegmentationComponent",
        name="trunk_segmentation_node",
        namespace="",
        parameters=[
            trunk_seg_config,
            {"use_sim_time": True},
            {"enable_diagnostics": True},
        ],
        remappings=[
            ("trunk_seg/scan", "/scan"),
            ("trunk_seg/odom", odom_topic),
            ("trunk_observations", "/trunk_observations"),
            ("active_tracks", "/active_tracks"),
            ("lost_tracks", "/lost_tracks"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    composable_nodes = [trunk_seg_component]

    if use_alignment:
        map_align_dir = get_package_share_directory("map_alignment")
        map_align_config = os.path.join(
            map_align_dir, "config", "map_alignment_params.yaml")
        map_pcd_path = os.path.join(
            map_align_dir, "maps", "klk", "tuanmee_site.pcd")

        map_align_component = ComposableNode(
            package="map_alignment",
            plugin="uosm::perception::MapAlignmentComponent",
            name="map_alignment_node",
            namespace="",
            parameters=[
                {"pcd_file_path": map_pcd_path},
                map_align_config,
                {
                    "use_sim_time": True,
                    "initial_yaw_rad": initial_yaw_rad,
                },
            ],
            remappings=[
                ("map_alignment_markers", "/map_alignment_markers"),
                ("observed_landmarks", "/trunk_observations"),
                ("alignment_done", "/map_alignment/alignment_done"),
                ("alignment_transform",
                 "/map_alignment/alignment_transform"),
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        composable_nodes.append(map_align_component)

    container = ComposableNodeContainer(
        name="eval_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        arguments=["--ros-args", "--log-level", "info"],
        output="screen",
        composable_node_descriptions=composable_nodes,
    )

    entities = [container]

    # ── visualization ─────────────────────────────────────────────────
    use_rviz = LaunchConfiguration("use_rviz").perform(context).lower() == "true"

    if use_rviz:
        bringup_dir = get_package_share_directory("uosm_uav_bringup")
        rviz_config = os.path.join(
            bringup_dir, "config", "rviz", "evaluate_trunk_detection.rviz")
        rviz_args = ["-d", rviz_config] if os.path.isfile(rviz_config) else []
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=rviz_args,
            parameters=[{"use_sim_time": True}],
        )
        entities.append(rviz_node)
    else:
        foxglove_bridge_node = Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[
                {"use_sim_time": True},
                {"port": 8765},
            ],
        )
        entities.append(foxglove_bridge_node)

    if not use_alignment:
        static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_tf",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "map", "--child-frame-id", "odom",
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
        )
        entities.append(static_tf)

    # Record output topics
    record_cmd = [
        "ros2", "bag", "record",
        "-o", out_bag,
        "--use-sim-time",
    ] + RECORD_TOPICS
    bag_record = ExecuteProcess(cmd=record_cmd, output="screen")
    entities.append(bag_record)

    # Play input bag (slight delay so recorder + nodes are ready)
    bag_play = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag_path, "--clock"],
                output="screen",
            ),
        ],
    )
    entities.append(bag_play)

    return entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "bag_path",
            default_value="./bags/real/klk/test_manual_1_corrected",
            description="Path to the input rosbag",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/odom/corrected",
            description="Odometry topic in the bag",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz2 for visualization; "
                        "when false, launches Foxglove bridge instead",
        ),
        DeclareLaunchArgument(
            "use_alignment",
            default_value="false",
            description="Enable map alignment (required for mutual "
                        "map-vs-detection evaluation)",
        ),
        DeclareLaunchArgument(
            "initial_yaw_deg",
            default_value="0.0",
            description="Drone initial yaw offset in degrees",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
