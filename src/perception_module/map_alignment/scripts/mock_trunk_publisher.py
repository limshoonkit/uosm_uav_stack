#!/usr/bin/env python3
"""
Mock trunk observation publisher for testing map alignment.

Simulates trunk observations as if the drone is at a specified position
with some noise, allowing you to test alignment without real sensors.

Usage:
  ros2 run map_alignment mock_trunk_publisher.py
  ros2 run map_alignment mock_trunk_publisher.py --ros-args -p offset_x:=1.5 -p offset_y:=-0.8
"""

import rclpy
from rclpy.node import Node
from uosm_uav_interface.msg import TrunkObservation, TrunkObservationArray
import numpy as np
import random


class MockTrunkPublisher(Node):
    def __init__(self):
        super().__init__('mock_trunk_publisher')
        
        # Parameters - the "true" offset from initial guess (simulates takeoff error)
        self.declare_parameter('offset_x', 0.5)  # meters from initial guess
        self.declare_parameter('offset_y', -0.3)  # meters from initial guess
        self.declare_parameter('offset_yaw', 0.05)  # radians
        self.declare_parameter('noise_std', 0.1)  # observation noise (meters)
        self.declare_parameter('publish_rate', 2.0)  # Hz
        
        self.offset_x = self.get_parameter('offset_x').value
        self.offset_y = self.get_parameter('offset_y').value
        self.offset_yaw = self.get_parameter('offset_yaw').value
        self.noise_std = self.get_parameter('noise_std').value
        publish_rate = self.get_parameter('publish_rate').value
        
        self.get_logger().info(f'Mock trunk publisher started')
        self.get_logger().info(f'  Simulated offset from initial guess: ({self.offset_x:.2f}, {self.offset_y:.2f}) m, yaw: {self.offset_yaw:.3f} rad')
        self.get_logger().info(f'  Observation noise std: {self.noise_std:.2f} m')
        
        # Publisher
        self.pub = self.create_publisher(
            TrunkObservationArray, 
            '/trunk_observations', 
            10
        )
        
        # Known trunk positions in MAP frame (should match your PCD map after recentering)
        # These are example positions - adjust to match your actual map
        self.trunk_positions_map = [
            (-2.0, 1.5),
            (0.0, 3.0),
            (2.5, 2.0),
            (3.0, -1.0),
            (-1.5, -2.5),
        ]
        
        self.get_logger().info(f'  Simulating {len(self.trunk_positions_map)} trunks in map frame')
        
        # Timer
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_observations)
        self.landmark_id = 0
        
    def publish_observations(self):
        """Publish trunk observations as they would appear in odom frame."""
        msg = TrunkObservationArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        
        # Rotation matrix for yaw offset
        cos_yaw = np.cos(self.offset_yaw)
        sin_yaw = np.sin(self.offset_yaw)
        
        for i, (mx, my) in enumerate(self.trunk_positions_map):
            # Transform from map to odom frame
            # odom_pos = R^T * (map_pos - offset)
            # where offset is the true position of odom origin in map frame
            
            dx = mx - self.offset_x
            dy = my - self.offset_y
            
            # Apply inverse rotation
            ox = cos_yaw * dx + sin_yaw * dy
            oy = -sin_yaw * dx + cos_yaw * dy
            
            # Add noise
            ox += random.gauss(0, self.noise_std)
            oy += random.gauss(0, self.noise_std)
            
            obs = TrunkObservation()
            obs.landmark_id = i
            obs.position.x = ox
            obs.position.y = oy
            obs.position.z = 0.0
            
            # Covariance (2x2 flattened)
            cov = self.noise_std ** 2
            obs.covariance = [cov, 0.0, 0.0, cov]
            
            msg.observations.append(obs)
        
        self.pub.publish(msg)
        self.get_logger().debug(f'Published {len(msg.observations)} trunk observations')


def main(args=None):
    rclpy.init(args=args)
    node = MockTrunkPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
