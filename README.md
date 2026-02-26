# UoSM UAV Stack


## Prerequisites

- **ROS 2 Humble** 
- **Jetpack 6.2** / **Ubuntu 22.04 LTS**

## A. Build

TBD
```bash
chmod +x ./scripts/setup.sh
./scripts/setup.sh
```

Experimental nmpc
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

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
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

Sanity check to make sure hardware working as expected, also to test PX4 params tuning whether vehicle can hover stabily and other analysis (vibration metric, ekf2, pid control, battery), this enable vehicle to fly in position mode without GNSS
```bash
source install/setup.bash
ros2 launch uosm_uav_bringup real_sanity_check.launch.py use_rosbag:=false
```

**IMPORTANT**: Real flight, make sure to fly in safe area
```bash
source install/setup.bash
# ros2 launch uosm_uav_bringup real_jetson_orin_interactive.launch.py use_foxglove:=true
ros2 launch uosm_uav_bringup real_jetson_orin_preset_wp.launch.py use_foxglove:=true wait_for_alignment:=true

ros2 launch uosm_uav_bringup real_jetson_orin_zedm_ego_only.launch.py use_foxglove:=true
```

## Package Overview

| Package | Description |
|---|---|
| `nmpc_controller` | Acados-based NMPC solver (C++ library) |
| `acados_vendor_ros2` | ROS 2 vendor package for acados |
| `planner_manager` | Ego planner + flight controller components |
| `uosm_uav_bringup` | Launch files and configs |
| `uosm_uav_interface` | Custom ROS 2 message definitions |
| `grid_map` | Occupancy grid for planning |
| `planner_utils` | Polynomial trajectory utilities |
| `local_sensing` | Simulated depth camera |
| `map_processor` | Point cloud map loading |
| `odom_visualization` | Odometry visualization in RViz |
| `pose_utils` | Pose/transform utilities |
| `uosm_robot_viewer` | URDF model and robot state publisher |
