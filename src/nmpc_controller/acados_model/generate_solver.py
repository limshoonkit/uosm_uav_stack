#!/usr/bin/env python3
"""
Generate acados C solver for quadrotor NMPC (triple-integrator model).

Usage:
    source the acados_vendor_ros2 workspace, then:
    python3 generate_solver.py

This generates C code in ../acados_generated/ that gets compiled into
the nmpc_controller library.
"""

import os
import shutil
import numpy as np
from casadi import SX, vertcat, diag
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

# ============================================================
# Model: Triple integrator (point-mass, jerk as control input)
# State:   x = [px, py, pz, vx, vy, vz, ax, ay, az]  (9)
# Control: u = [jx, jy, jz]                            (3)
# ============================================================

def create_quadrotor_model() -> AcadosModel:
    model_name = "quadrotor_nmpc"

    # States
    px = SX.sym("px")
    py = SX.sym("py")
    pz = SX.sym("pz")
    vx = SX.sym("vx")
    vy = SX.sym("vy")
    vz = SX.sym("vz")
    ax = SX.sym("ax")
    ay = SX.sym("ay")
    az = SX.sym("az")
    x = vertcat(px, py, pz, vx, vy, vz, ax, ay, az)

    # Controls (jerk)
    jx = SX.sym("jx")
    jy = SX.sym("jy")
    jz = SX.sym("jz")
    u = vertcat(jx, jy, jz)

    # State derivatives (triple integrator)
    xdot = vertcat(vx, vy, vz, ax, ay, az, jx, jy, jz)

    # Explicit ODE
    f_expl = xdot

    # acados model
    model = AcadosModel()
    model.name = model_name
    model.x = x
    model.u = u
    model.xdot = SX.sym("xdot", 9)
    model.f_expl_expr = f_expl
    model.f_impl_expr = model.xdot - f_expl

    return model


def create_ocp() -> AcadosOcp:
    # ----- Parameters -----
    N = 20          # horizon steps
    dt = 0.05       # step size (1.0 s total horizon)
    nx = 9          # states
    nu = 3          # controls

    # Cost weights
    w_pos = np.array([80.0, 80.0, 120.0])
    w_vel = np.array([10.0, 10.0, 15.0])
    w_acc = np.array([5.0, 5.0, 8.0])
    w_jerk = np.array([1.0, 1.0, 1.0])

    # Constraints
    v_max = np.array([1.0, 1.0, 0.5])
    a_max = np.array([3.0, 3.0, 2.0])
    j_max = np.array([10.0, 10.0, 8.0])

    # ----- Model -----
    model = create_quadrotor_model()

    # ----- OCP -----
    ocp = AcadosOcp()
    ocp.model = model
    ocp.dims.N = N
    ocp.solver_options.tf = N * dt

    # ----- Cost: Linear least-squares -----
    # y = [x; u], y_ref = [x_ref; u_ref=0]
    # Stage cost: || Vy @ y - y_ref ||^2_W
    ny = nx + nu
    ny_e = nx

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    # Vx selects states, Vu selects controls
    Vx = np.eye(nx, nx)
    Vu = np.zeros((nx, nu))
    Vx_full = np.vstack([Vx, np.zeros((nu, nx))])
    Vu_full = np.vstack([np.zeros((nx, nu)), np.eye(nu)])

    ocp.cost.Vx = Vx_full
    ocp.cost.Vu = Vu_full
    ocp.cost.Vx_e = np.eye(nx)

    # Weight matrices
    Q = np.diag(np.concatenate([w_pos, w_vel, w_acc]))
    R = np.diag(w_jerk)
    W = np.block([
        [Q, np.zeros((nx, nu))],
        [np.zeros((nu, nx)), R]
    ])
    W_e = Q

    ocp.cost.W = W
    ocp.cost.W_e = W_e

    # Reference (will be set at runtime)
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)

    # ----- Constraints -----
    # State box constraints (velocity and acceleration)
    # x = [px, py, pz, vx, vy, vz, ax, ay, az]
    #      0    1    2   3    4    5   6    7    8
    ocp.constraints.idxbx = np.array([3, 4, 5, 6, 7, 8])
    ocp.constraints.lbx = np.concatenate([-v_max, -a_max])
    ocp.constraints.ubx = np.concatenate([v_max, a_max])

    # Control box constraints (jerk)
    ocp.constraints.idxbu = np.array([0, 1, 2])
    ocp.constraints.lbu = -j_max
    ocp.constraints.ubu = j_max

    # Initial state constraint (set at runtime)
    ocp.constraints.x0 = np.zeros(nx)

    # ----- Solver options -----
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.nlp_solver_max_iter = 1  # single RTI iteration
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.hpipm_mode = "SPEED"

    return ocp


def main():
    ocp = create_ocp()

    # Output directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, "..", "acados_generated")
    os.makedirs(output_dir, exist_ok=True)

    # Generate solver
    json_file = os.path.join(output_dir, "acados_ocp_quadrotor_nmpc.json")
    solver = AcadosOcpSolver(ocp, json_file=json_file)

    print(f"Solver generated successfully!")
    print(f"Output directory: {output_dir}")
    print(f"JSON file: {json_file}")

    # Move generated files to output directory
    # acados generates files in the current working directory,
    # so we need to move them
    gen_files = [
        "acados_solver_quadrotor_nmpc.h",
        "acados_solver_quadrotor_nmpc.c",
        "acados_sim_solver_quadrotor_nmpc.h",
        "acados_sim_solver_quadrotor_nmpc.c",
        "main_quadrotor_nmpc.c",
        "Makefile",
        "CMakeLists.txt",
    ]
    gen_dirs = [
        "quadrotor_nmpc_model",
        "c_generated_code",
    ]

    for f in gen_files:
        src = os.path.join(os.getcwd(), f)
        if os.path.exists(src):
            dst = os.path.join(output_dir, f)
            shutil.move(src, dst)
            print(f"  Moved {f}")

    for d in gen_dirs:
        src = os.path.join(os.getcwd(), d)
        if os.path.exists(src):
            dst = os.path.join(output_dir, d)
            if os.path.exists(dst):
                shutil.rmtree(dst)
            shutil.move(src, dst)
            print(f"  Moved {d}/")

    print("\nDone! Generated C code is in acados_generated/")
    print("You can now build the nmpc_controller package with colcon.")


if __name__ == "__main__":
    main()
