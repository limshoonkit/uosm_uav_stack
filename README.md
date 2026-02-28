# UoSM UAV Stack

A complete PX4-based autonomous UAV stack using a stereo camera (ZEDM) and 2D LiDAR (RPLidar S1) for under-canopy surveying in oil palm plantations.
Virtual map generation and global path planning are available at [PalmOilDetectree2](https://github.com/limshoonkit/PalmOilDetectree2/).

## Prerequisites

- **ROS 2 Humble** 
- **Jetpack 6.2** / **Ubuntu 22.04 LTS**

## A. Build

Install dependencies:
```bash
chmod +x ./scripts/setup.sh
./scripts/setup.sh
```

Build workspace:
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
```

Optional: experimental NMPC setup
```bash
# 1. Build acados vendor package
colcon build --packages-select acados_vendor_ros2
source install/setup.bash

# 2. Patch acados_vendor for Jetson (aarch64 t_renderer + Python 3.10 fix)
./src/nmpc_controller/scripts/patch_acados_vendor.sh

# 3. Ensure compatible numpy and scipy
pip3 install numpy==1.26.4 scipy==1.15.3

# 4. Generate the acados solver C code
cd src/nmpc_controller/acados_model
python3 generate_solver.py
cd ../../../

# 5. Build the NMPC controller
colcon build --packages-select nmpc_controller
```

## B. Run the Simulation

For interactive mode, once RViz opens use the **2D Nav Goal** tool to click waypoints. The ego planner will generate a collision-free trajectory and the controller will track it.

For preset waypoint mode, the planner will begin executing the loaded waypoints automatically after takeoff, waypoints and map currently in [maps](./src/virtual_module/map_processor/maps).

### Terminal 1 — Start PX4 SITL

```bash
cd ~/PX4-Autopilot && HEADLESS=1 make px4_sitl gz_x500
```

Wait until PX4 finishes initialization and launch [QGC](https://github.com/mavlink/qgroundcontrol).

### Terminal 2 — Launch the simulation

Two launch files are available depending on how you want to supply waypoints:

**Interactive mode** — click waypoints in RViz using the 2D Nav Goal tool:

```bash
source install/setup.bash
ros2 launch uosm_uav_bringup sim_interactive_single.launch.py
```

**Preset waypoint mode** — follows a pre-defined waypoint CSV file:

```bash
source install/setup.bash
ros2 launch uosm_uav_bringup sim_preset_wp_single.launch.py
```

## C. Deployment

Reboot PX4 Autopilot to reset internal state
```bash
source install/setup.bash
ros2 launch uosm_uav_bringup reboot_px4.launch.py
```

Sanity check for hardware and PX4 tuning (hover stability, vibration metrics, EKF2 health, PID behavior, battery). This helps confirm stable position-mode flight without GNSS.
```bash
source install/setup.bash
ros2 launch uosm_uav_bringup real_sanity_check.launch.py use_rosbag:=false
```

**IMPORTANT**: Real flight, make sure to fly in safe area.
Preflight checklist:
- Verify RC kill switch and failsafe behavior.
- Confirm battery level above arm threshold.
- Verify EKF2 and local position estimate are healthy. ReCalibrate if needed.
- Confirm long range telemetry working to maintainn ground control link.
- Confirm wifi/network static ip and dds setup correctly. 

**KNOWN ISSUE**: For rosbag recording, avoid recording too many heavy topics; File I/O operation can significantly impact flight performance.
```bash
source install/setup.bash
# ros2 launch uosm_uav_bringup real_jetson_orin_interactive.launch.py use_foxglove:=true
ros2 launch uosm_uav_bringup real_jetson_orin_preset_wp.launch.py use_foxglove:=true wait_for_alignment:=true use_rosbag:=true

ros2 launch uosm_uav_bringup real_minimal.launch.py use_foxglove:=true
```

## Repository Overview

### Root layout

| Path | Purpose |
|---|---|
| `src/` | Main ROS 2 packages |
| `scripts/` | Setup and helper scripts |
| `bags/` | Recorded rosbag data (real and simulation) |
| `px4_related/` | PX4 params and related artifacts |
| `jetson_config/` | Jetson device-specific configs |
| `build/`, `install/` | Colcon build outputs |

### `src/` package structure

| Group | Main packages | Purpose |
|---|---|---|
| Core bringup and interfaces | `uosm_uav_bringup`, `uosm_uav_interface`, `uosm_robot_viewer` | System launch/config, custom interfaces, robot model visualization |
| Planning and control | `nmpc_controller`, `path_planning_module/grid_map`, `path_planning_module/planner_manager`, `path_planning_module/planner_utils` | NMPC trajectory tracking, occupancy mapping, planning, trajectory utilities |
| Perception | `perception_module/odom_republisher`, `perception_module/map_alignment`, `perception_module/trunk_segmentation`, `perception_module/jetson_gscam2`, `perception_module/sllidar_ros2`, `perception_module/zed-ros2-wrapper` | Odometry fusion/alignment, sensing, camera/lidar integration |
| Virtual and simulation support | `virtual_module/local_sensing`, `virtual_module/map_processor`, `virtual_module/odom_visualization`, `virtual_module/pose_utils` | Simulated sensing, map loading, odometry display, pose/transform helpers |
| Third-party vendors/tools | `third_party/acados_vendor_ros2`, `third_party/ros2_jetson_stats` | External solver and Jetson monitoring integration |

## License

This project is licensed under the GNU General Public License v3.0 (GPL-3.0).
See [LICENSE](./LICENSE) for full terms.

## Citation???

Cite so that my supervisor can keep his job and get a promotion.

```bibtex
@article{uosm_uav_stack,
  title   = {xxx},
  author  = {Janitor},
  year    = {2026},
  url     = {xxx},
  note    = {xxx}
}
```

## Contact

- Email: skl1g14@soton.ac.uk / lsk950329@hotmail.com