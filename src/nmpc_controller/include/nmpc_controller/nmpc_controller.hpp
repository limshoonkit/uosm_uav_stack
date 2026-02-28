/*
 * SPDX-FileCopyrightText: University of Southampton Malaysia
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef NMPC_CONTROLLER__NMPC_CONTROLLER_HPP_
#define NMPC_CONTROLLER__NMPC_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <vector>
#include <Eigen/Dense>

namespace nmpc_controller
{

    struct NmpcParams
    {
        int horizon_steps = 20;
        double dt = 0.05;
        std::array<double, 3> w_pos = {80.0, 80.0, 120.0};
        std::array<double, 3> w_vel = {10.0, 10.0, 15.0};
        std::array<double, 3> w_acc = {5.0, 5.0, 8.0};
        std::array<double, 3> w_jerk = {1.0, 1.0, 1.0};
        std::array<double, 3> v_max = {1.0, 1.0, 0.5};
        std::array<double, 3> a_max = {3.0, 3.0, 2.0};
        std::array<double, 3> j_max = {10.0, 10.0, 8.0};
    };

    struct NmpcState
    {
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d vel = Eigen::Vector3d::Zero();
        Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    };

    class NmpcController
    {
    public:
        explicit NmpcController(const NmpcParams &params);
        ~NmpcController();

        // Disallow copy
        NmpcController(const NmpcController &) = delete;
        NmpcController &operator=(const NmpcController &) = delete;

        /// Set current state as initial constraint (x0)
        void setCurrentState(const NmpcState &state);

        /// Set reference trajectory over the horizon (size = N+1)
        void setReference(const std::vector<NmpcState> &refs);

        /// Solve the OCP. Returns acados status (0 = success).
        int solve();

        /// Get the optimal setpoint from stage 1 of the solution
        NmpcState getOptimalSetpoint() const;

        /// Get the solve time in milliseconds
        double getSolveTimeMs() const;

        /// Accessors for horizon parameters
        int getHorizonSteps() const { return params_.horizon_steps; }
        double getHorizonDt() const { return params_.dt; }

    private:
        NmpcParams params_;
        void *capsule_ = nullptr; // opaque pointer to acados solver capsule
        double last_solve_time_ms_ = 0.0;
    };

} // namespace nmpc_controller

#endif // NMPC_CONTROLLER__NMPC_CONTROLLER_HPP_
