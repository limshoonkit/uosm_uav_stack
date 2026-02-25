#!/usr/bin/env python3
"""
Waypoint visualizer node for map alignment bag playback testing.

Loads waypoints from a CSV file and publishes MarkerArray visualizations:
  - GREEN markers: original waypoints in the map frame (pre-alignment)
  - BLUE markers:  transformed waypoints in the odom frame (post-alignment)

Subscribes to map alignment topics to know when alignment is complete and
to obtain the alignment transform for waypoint offset computation.
"""

import csv
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, ColorRGBA
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np


def quaternion_to_rotation_matrix(q):
    """Convert quaternion [x, y, z, w] to 3x3 rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def transform_stamped_to_matrix(transform_msg):
    """Convert a geometry_msgs/TransformStamped to a 4x4 homogeneous matrix."""
    t = transform_msg.transform
    q = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
    R = quaternion_to_rotation_matrix(q)
    T = np.eye(4)
    T[:3, :3] = R
    T[0, 3] = t.translation.x
    T[1, 3] = t.translation.y
    T[2, 3] = t.translation.z
    return T


def transform_point(T, point):
    """Apply a 4x4 homogeneous transform to a 3D point."""
    p = np.array([point[0], point[1], point[2], 1.0])
    result = T @ p
    return result[:3]


class WaypointVisualizer(Node):
    def __init__(self):
        super().__init__('waypoint_visualizer_node')

        # Parameters
        self.declare_parameter('waypoint_csv_file_path', '')
        self.declare_parameter('flight_height', 3.0)
        self.declare_parameter('frame_id', 'map')

        csv_path = self.get_parameter('waypoint_csv_file_path').get_parameter_value().string_value
        self.flight_height_ = self.get_parameter('flight_height').get_parameter_value().double_value
        self.frame_id_ = self.get_parameter('frame_id').get_parameter_value().string_value

        # State
        self.waypoints_map_ = []      # (x, y, z, state) in map frame
        self.waypoints_odom_ = []     # (x, y, z) in odom frame (after alignment)
        self.alignment_done_ = False
        self.alignment_transform_ = None  # TransformStamped msg

        # Load waypoints
        if csv_path:
            self._load_waypoints(csv_path)
        else:
            self.get_logger().warn('No waypoint CSV file specified')

        # Publisher
        self.marker_pub_ = self.create_publisher(MarkerArray, 'waypoint_markers', 10)

        # Subscribers — volatile QoS to match alignment publishers
        # (late-joining is handled by the alignment node's periodic re-publish timer)
        self.alignment_status_sub_ = self.create_subscription(
            Bool, 'alignment_done', self._alignment_status_cb, 1)

        self.alignment_transform_sub_ = self.create_subscription(
            TransformStamped, 'alignment_transform', self._alignment_transform_cb, 1)

        # Publish timer at 1 Hz
        self.timer_ = self.create_timer(1.0, self._publish_markers)

        self.get_logger().info(
            f'Waypoint visualizer started: {len(self.waypoints_map_)} waypoints loaded, '
            f'flight_height={self.flight_height_}, frame={self.frame_id_}'
        )

    def _load_waypoints(self, csv_path: str):
        """Load waypoints from CSV (x_local, y_local, state)."""
        try:
            with open(csv_path, 'r') as f:
                reader = csv.reader(f)
                header = next(reader, None)  # skip header
                if header is None:
                    self.get_logger().error(f'Empty CSV file: {csv_path}')
                    return
                for i, row in enumerate(reader):
                    if len(row) < 3:
                        self.get_logger().warn(f'Skipping malformed line {i+2} in {csv_path}')
                        continue
                    try:
                        x = float(row[0])
                        y = float(row[1])
                        state = row[2].strip()
                        self.waypoints_map_.append((x, y, self.flight_height_, state))
                    except ValueError as e:
                        self.get_logger().warn(f'Invalid number at line {i+2}: {e}')
            self.get_logger().info(f'Loaded {len(self.waypoints_map_)} waypoints from {csv_path}')
        except FileNotFoundError:
            self.get_logger().error(f'CSV file not found: {csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load CSV: {e}')

    def _alignment_status_cb(self, msg: Bool):
        if msg.data and not self.alignment_done_:
            self.get_logger().info('Map alignment complete — transforming waypoints to odom frame')
            self.alignment_done_ = True
            self._compute_odom_waypoints()

    def _alignment_transform_cb(self, msg: TransformStamped):
        self.alignment_transform_ = msg
        self.get_logger().info(
            f'Received alignment transform: [{msg.transform.translation.x:.3f}, '
            f'{msg.transform.translation.y:.3f}, {msg.transform.translation.z:.3f}]'
        )
        # If alignment was already flagged done but we hadn't received the transform yet
        if self.alignment_done_ and not self.waypoints_odom_:
            self._compute_odom_waypoints()

    def _compute_odom_waypoints(self):
        """Transform waypoints from map frame to odom frame using the alignment transform.

        The alignment_transform is map->odom (parent=map, child=odom), representing
        T_map_odom. To transform points from map to odom coordinates we need the inverse:
        p_odom = T_map_odom^{-1} * p_map
        """
        if self.alignment_transform_ is None:
            self.get_logger().warn('Alignment done but no transform received yet')
            return
        if not self.waypoints_map_:
            return

        # T_map_odom: positions odom frame within map frame
        T_map_odom = transform_stamped_to_matrix(self.alignment_transform_)
        # Invert to get map-point -> odom-point
        T_odom_map = np.linalg.inv(T_map_odom)

        self.waypoints_odom_ = []
        for (x, y, z, _state) in self.waypoints_map_:
            p_odom = transform_point(T_odom_map, [x, y, z])
            self.waypoints_odom_.append((p_odom[0], p_odom[1], p_odom[2]))

        self.get_logger().info(f'Transformed {len(self.waypoints_odom_)} waypoints to odom frame')

    # ------------------------------------------------------------------ #
    #  Marker generation
    # ------------------------------------------------------------------ #

    def _make_color(self, r, g, b, a=1.0):
        c = ColorRGBA()
        c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
        return c

    def _build_markers(self, waypoints, frame_id, ns_prefix, color, id_offset=0):
        """Build line, sphere, and text markers for a set of waypoints."""
        markers = []
        now = self.get_clock().now().to_msg()

        if not waypoints:
            return markers

        # --- Line strip ---
        line = Marker()
        line.header.frame_id = frame_id
        line.header.stamp = now
        line.ns = f'{ns_prefix}_line'
        line.id = id_offset
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.08  # line width
        line.color = color
        line.pose.orientation.w = 1.0
        for wp in waypoints:
            p = Point()
            p.x, p.y, p.z = float(wp[0]), float(wp[1]), float(wp[2])
            line.points.append(p)
        markers.append(line)

        # --- Spheres + text ---
        for i, wp in enumerate(waypoints):
            # Sphere
            sphere = Marker()
            sphere.header.frame_id = frame_id
            sphere.header.stamp = now
            sphere.ns = f'{ns_prefix}_sphere'
            sphere.id = id_offset + i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(wp[0])
            sphere.pose.position.y = float(wp[1])
            sphere.pose.position.z = float(wp[2])
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.3
            sphere.color = color
            markers.append(sphere)

            # Text
            text = Marker()
            text.header.frame_id = frame_id
            text.header.stamp = now
            text.ns = f'{ns_prefix}_text'
            text.id = id_offset + i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(wp[0])
            text.pose.position.y = float(wp[1])
            text.pose.position.z = float(wp[2]) + 0.5
            text.pose.orientation.w = 1.0
            text.scale.z = 0.25
            text.color = self._make_color(1.0, 1.0, 1.0)
            label = f'{i}'
            if len(wp) > 3:
                label += f' ({wp[3]})'
            text.text = label
            markers.append(text)

        return markers

    def _publish_markers(self):
        """Timer callback: publish all waypoint markers."""
        if not self.waypoints_map_:
            return

        ma = MarkerArray()

        # Pre-alignment waypoints (GREEN, map frame)
        green = self._make_color(0.0, 0.9, 0.0, 0.9)
        ma.markers.extend(
            self._build_markers(self.waypoints_map_, self.frame_id_, 'waypoint', green, id_offset=0)
        )

        # Post-alignment waypoints (BLUE, odom frame)
        if self.waypoints_odom_:
            blue = self._make_color(0.2, 0.4, 1.0, 0.9)
            ma.markers.extend(
                self._build_markers(self.waypoints_odom_, 'odom', 'waypoint_odom', blue, id_offset=1000)
            )

        self.marker_pub_.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
