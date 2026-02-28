/*
 * SPDX-FileCopyrightText: University of Southampton Malaysia
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "nmpc_controller/nmpc_controller.hpp"

// acados generated headers
#include "acados_solver_quadrotor_nmpc.h"
#include "acados_c/ocp_nlp_interface.h"

#include <chrono>
#include <stdexcept>
#include <cstring>
#include <iostream>

namespace nmpc_controller
{

NmpcController::NmpcController(const NmpcParams &params)
    : params_(params)
{
    // Create the acados solver capsule
    auto *capsule = quadrotor_nmpc_acados_create_capsule();
    if (!capsule)
    {
        throw std::runtime_error("Failed to create acados solver capsule");
    }

    int status = quadrotor_nmpc_acados_create(capsule);
    if (status != 0)
    {
        quadrotor_nmpc_acados_free_capsule(capsule);
        throw std::runtime_error("Failed to create acados solver, status: " + std::to_string(status));
    }

    capsule_ = static_cast<void *>(capsule);

    // Set cost weights from params
    // W = diag([w_pos, w_vel, w_acc, w_jerk]) — 12x12 for stage cost
    // W_e = diag([w_pos, w_vel, w_acc]) — 9x9 for terminal cost
    int N = QUADROTOR_NMPC_N;
    int ny = QUADROTOR_NMPC_NY;    // 12 (9 states + 3 controls)
    int ny_e = QUADROTOR_NMPC_NYN; // 9 (states only)

    // Build stage cost weight matrix W (ny x ny)
    std::vector<double> W(ny * ny, 0.0);
    for (int i = 0; i < 3; ++i)
    {
        W[i * ny + i] = params_.w_pos[i];                     // pos
        W[(i + 3) * ny + (i + 3)] = params_.w_vel[i];         // vel
        W[(i + 6) * ny + (i + 6)] = params_.w_acc[i];         // acc
        W[(i + 9) * ny + (i + 9)] = params_.w_jerk[i];        // jerk (control)
    }

    // Build terminal cost weight matrix W_e (ny_e x ny_e)
    std::vector<double> W_e(ny_e * ny_e, 0.0);
    for (int i = 0; i < 3; ++i)
    {
        W_e[i * ny_e + i] = params_.w_pos[i];
        W_e[(i + 3) * ny_e + (i + 3)] = params_.w_vel[i];
        W_e[(i + 6) * ny_e + (i + 6)] = params_.w_acc[i];
    }

    ocp_nlp_config *nlp_config = quadrotor_nmpc_acados_get_nlp_config(capsule);
    ocp_nlp_dims *nlp_dims = quadrotor_nmpc_acados_get_nlp_dims(capsule);
    ocp_nlp_in *nlp_in = quadrotor_nmpc_acados_get_nlp_in(capsule);
    ocp_nlp_out *nlp_out = quadrotor_nmpc_acados_get_nlp_out(capsule);

    // Set cost weights for each stage
    for (int k = 0; k < N; ++k)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, k, "W", W.data());
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e.data());

    // Set constraint bounds from params
    double lbx[6], ubx[6];
    for (int i = 0; i < 3; ++i)
    {
        lbx[i] = -params_.v_max[i];     // velocity lower
        ubx[i] = params_.v_max[i];      // velocity upper
        lbx[i + 3] = -params_.a_max[i]; // acceleration lower
        ubx[i + 3] = params_.a_max[i];  // acceleration upper
    }
    double lbu[3], ubu[3];
    for (int i = 0; i < 3; ++i)
    {
        lbu[i] = -params_.j_max[i];
        ubu[i] = params_.j_max[i];
    }

    for (int k = 0; k < N; ++k)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, k, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, k, "ubx", ubx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, k, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, k, "ubu", ubu);
    }
    // Terminal state constraints
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "lbx", lbx);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "ubx", ubx);

    std::cout << "[NmpcController] Initialized: N=" << N
              << ", dt=" << params_.dt
              << ", ny=" << ny
              << ", ny_e=" << ny_e << std::endl;
}

NmpcController::~NmpcController()
{
    if (capsule_)
    {
        auto *capsule = static_cast<quadrotor_nmpc_solver_capsule *>(capsule_);
        quadrotor_nmpc_acados_free(capsule);
        quadrotor_nmpc_acados_free_capsule(capsule);
        capsule_ = nullptr;
    }
}

void NmpcController::setCurrentState(const NmpcState &state)
{
    auto *capsule = static_cast<quadrotor_nmpc_solver_capsule *>(capsule_);
    ocp_nlp_config *nlp_config = quadrotor_nmpc_acados_get_nlp_config(capsule);
    ocp_nlp_dims *nlp_dims = quadrotor_nmpc_acados_get_nlp_dims(capsule);
    ocp_nlp_in *nlp_in = quadrotor_nmpc_acados_get_nlp_in(capsule);
    ocp_nlp_out *nlp_out = quadrotor_nmpc_acados_get_nlp_out(capsule);

    // x0 = [px, py, pz, vx, vy, vz, ax, ay, az]
    double x0[9];
    x0[0] = state.pos.x();
    x0[1] = state.pos.y();
    x0[2] = state.pos.z();
    x0[3] = state.vel.x();
    x0[4] = state.vel.y();
    x0[5] = state.vel.z();
    x0[6] = state.acc.x();
    x0[7] = state.acc.y();
    x0[8] = state.acc.z();

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0);
}

void NmpcController::setReference(const std::vector<NmpcState> &refs)
{
    auto *capsule = static_cast<quadrotor_nmpc_solver_capsule *>(capsule_);
    ocp_nlp_config *nlp_config = quadrotor_nmpc_acados_get_nlp_config(capsule);
    ocp_nlp_dims *nlp_dims = quadrotor_nmpc_acados_get_nlp_dims(capsule);
    ocp_nlp_in *nlp_in = quadrotor_nmpc_acados_get_nlp_in(capsule);

    int N = QUADROTOR_NMPC_N;

    // Stage references: yref = [pos, vel, acc, jerk_ref=0]
    for (int k = 0; k < N; ++k)
    {
        double yref[12] = {0.0};
        if (k < static_cast<int>(refs.size()))
        {
            yref[0] = refs[k].pos.x();
            yref[1] = refs[k].pos.y();
            yref[2] = refs[k].pos.z();
            yref[3] = refs[k].vel.x();
            yref[4] = refs[k].vel.y();
            yref[5] = refs[k].vel.z();
            yref[6] = refs[k].acc.x();
            yref[7] = refs[k].acc.y();
            yref[8] = refs[k].acc.z();
            // jerk reference = 0 (indices 9, 10, 11)
        }
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, k, "yref", yref);
    }

    // Terminal reference: yref_e = [pos, vel, acc]
    double yref_e[9] = {0.0};
    int last = std::min(static_cast<int>(refs.size()) - 1, N);
    if (last >= 0)
    {
        yref_e[0] = refs[last].pos.x();
        yref_e[1] = refs[last].pos.y();
        yref_e[2] = refs[last].pos.z();
        yref_e[3] = refs[last].vel.x();
        yref_e[4] = refs[last].vel.y();
        yref_e[5] = refs[last].vel.z();
        yref_e[6] = refs[last].acc.x();
        yref_e[7] = refs[last].acc.y();
        yref_e[8] = refs[last].acc.z();
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);
}

int NmpcController::solve()
{
    auto *capsule = static_cast<quadrotor_nmpc_solver_capsule *>(capsule_);

    auto t_start = std::chrono::high_resolution_clock::now();
    int status = quadrotor_nmpc_acados_solve(capsule);
    auto t_end = std::chrono::high_resolution_clock::now();

    last_solve_time_ms_ = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    return status;
}

NmpcState NmpcController::getOptimalSetpoint() const
{
    auto *capsule = static_cast<quadrotor_nmpc_solver_capsule *>(capsule_);
    ocp_nlp_config *nlp_config = quadrotor_nmpc_acados_get_nlp_config(capsule);
    ocp_nlp_dims *nlp_dims = quadrotor_nmpc_acados_get_nlp_dims(capsule);
    ocp_nlp_out *nlp_out = quadrotor_nmpc_acados_get_nlp_out(capsule);

    // Get state at stage 1 (the first predicted step after x0)
    double x1[9];
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", x1);

    NmpcState setpoint;
    setpoint.pos << x1[0], x1[1], x1[2];
    setpoint.vel << x1[3], x1[4], x1[5];
    setpoint.acc << x1[6], x1[7], x1[8];

    return setpoint;
}

double NmpcController::getSolveTimeMs() const
{
    return last_solve_time_ms_;
}

} // namespace nmpc_controller
