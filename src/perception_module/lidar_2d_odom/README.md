# lidar_2d_odom

Pipeline for 2D lidar odometry used in VIO + lidar fusion:

1. **ScanToPointCloudComponent**: Converts `sensor_msgs/LaserScan` to `sensor_msgs/PointCloud2` (points in scan plane, z=0) for [KISS-ICP](https://github.com/PRBonn/kiss-icp).
2. **TiltGatedOdomComponent**: Subscribes to odometry from KISS-ICP and current orientation; publishes `odom/lidar_2d` only when roll/pitch are below `max_tilt_deg` (default 15°), so fusion uses 2D lidar only when the drone is near-level.

## KISS-ICP (submodule)

KISS-ICP is included as a git submodule at `src/third_party/kiss-icp`. Build the workspace (including the `kiss_icp` package under `kiss-icp/ros`) before using lidar fusion.

- **Compare VIO vs 2D lidar (no fusion)**  
  `ros2 launch uosm_uav_bringup real_lidar_vio_compare.launch.py`  
  Publishes `/odom/vio` (ZED VIO) and `/odom/lidar_2d` (tilt-gated KISS-ICP). Use Foxglove or RViz to compare before enabling fusion.
- **Sanity check with lidar fusion** (includes KISS-ICP):  
  `ros2 launch uosm_uav_bringup real_sanity_check_lidar_fusion.launch.py`
- **Preset waypoint with lidar fusion**:  
  `ros2 launch uosm_uav_bringup real_jetson_orin_preset_wp.launch.py use_lidar_fusion:=true`  
  Then run KISS-ICP separately:  
  `ros2 launch kiss_icp odometry.launch.py topic:=/scan_cloud visualize:=false use_sim_time:=false`

Tilt_gated_odom subscribes to KISS-ICP odometry (`/kiss/odometry` by default; remap `odom_in` as needed).
