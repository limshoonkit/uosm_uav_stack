# Acados Model Code Generation

## Prerequisites

Build the workspace first to install acados via `acados_vendor_ros2`:

```bash
cd /home/ubuntu/Desktop/uosm_cirg/nmpc_quadrotor
source /opt/ros/humble/setup.bash
colcon build --packages-select acados_vendor_ros2
source install/setup.bash
```

## Generate Solver

```bash
cd src/nmpc_controller/acados_model
python3 generate_solver.py
```

This generates C code in `../acados_generated/` which is compiled into the
`nmpc_controller` library by CMake.

## Regenerating

Run `generate_solver.py` whenever you change:
- Dynamics model
- Horizon length (N) or step size (dt)
- Cost structure (e.g., switching from LINEAR_LS to EXTERNAL)
- Constraint types

Runtime-adjustable parameters (cost weights, constraint bounds) are set
in `nmpc_controller.cpp` at construction time from ROS parameters — no
regeneration needed for those.
