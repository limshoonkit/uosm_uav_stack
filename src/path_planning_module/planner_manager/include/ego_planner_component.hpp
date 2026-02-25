#ifndef EGO_PLANNER_COMPONENT_HPP_
#define EGO_PLANNER_COMPONENT_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <atomic>
#include <thread>
#include <fstream>

#include "uosm_uav_interface/action/ego_planner.hpp"
#include "uosm_uav_interface/msg/minco_traj.hpp"
#include "uosm_uav_interface/msg/polynomial_traj.hpp"
#include "plan_container.hpp"
#include "plan_visualizer.hpp"
#include "poly_traj_optimizer.hpp"
#include "grid_map.hpp"
#include "timer_profile.hpp"

namespace uosm
{
    namespace path_planning
    {
        using EgoPlannerAction = uosm_uav_interface::action::EgoPlanner;
        using EgoPlannerGoal = uosm_uav_interface::action::EgoPlanner::Goal;
        using EgoPlannerServerGoalHandle = rclcpp_action::ServerGoalHandle<EgoPlannerAction>;
        using MincoTraj = uosm_uav_interface::msg::MincoTraj;
        using PolynomialTraj = uosm_uav_interface::msg::PolynomialTraj;

        class EgoPlanner : public rclcpp::Node
        {
        public:
            enum class State
            {
                IDLE = 0,
                SEQUENTIAL_START,
                GEN_NEW_TRAJ,
                REPLAN_TRAJ,
                EXEC_TRAJ,
                EMERGENCY_STOP
            };

            struct PlannerParams
            {
                std::string waypoint_csv_file_path_ = "";
                double flight_height_ = 0.0;
                double planner_freq_ = 0.0;
                double collision_check_freq_ = 0.0;

                /* planning algorithm parameters */
                double max_vel = 0.0;
                double max_acc = 0.0;
                double emergency_time = 0.0;

                /* orbit-specific overrides (otherwise uses max_vel/max_acc) */
                double orbit_flight_height = 0.0;
                double orbit_max_vel = 0.0;
                double orbit_max_acc = 0.0;
                double poly_traj_piece_length = 0.0; // distance between adjacent poly control points
                double feasibility_tolerance = 0.0;  // permitted ratio of vel/acc exceeding limits
                double planning_horizon = 0.0;
                double replan_threshold = 0.0;
                bool use_multitopology_trajs = false;
                bool fail_safe = false;
                int global_retrials = 1;
                int local_retrials = 1;
                int drone_id = 0;

                PlannerParams() = default;
            };

            explicit EgoPlanner(const rclcpp::NodeOptions &options);
            ~EgoPlanner() = default;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        private:
            PlannerParams params_;
            ego_planner::TrajContainer traj_;
            std::shared_ptr<ego_planner::GridMap> grid_map_;
            std::unique_ptr<ego_planner::PolyTrajOptimizer> poly_traj_opt_;
            std::shared_ptr<ego_planner::PlanVisualizer> plan_visualizer_;
            State current_state_ = State::IDLE;

            // Variables
            std::vector<Eigen::Vector3d> waypoints_;
            std::vector<std::string> waypoint_type_;
            int num_waypoints_ = 0;
            int current_wp_ = 0;
            int continuous_failures_count_ = 0;
            int repeated_state_count_ = 0;
            Eigen::Vector3d start_pt_, start_vel_, start_acc_;   // start state
            Eigen::Vector3d final_goal_;                         // goal state
            Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
            Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;     // odometry state
            double no_replan_threshold_ = 0.0;
            double modify_goal_tstep_ = 0.0; // step to modify goal point in obstacles

            // Flags
            std::atomic<bool> is_triggered_ = false; // mission active?
            std::atomic<bool> has_odom_ = false;
            bool touch_goal_ = false;
            bool is_init_ = false;
            bool is_in_emergency_ = false;
            bool is_next_waypoint_ready_ = false;
            bool is_manual_target_ = false;
            bool has_target_ = false;
            bool has_new_target_ = false;
            double goal_yaw_ = 0.0;

            // Helper functions
            bool loadWaypointsFromCSV();
            void initComponents();
            void applyParamsForWaypointType(const std::string &type);
            void changeState(const State new_state);
            void polyTraj2ROSMsg(PolynomialTraj &poly_msg, MincoTraj &MINCO_msg);
            bool modifyGoalPointInObstacles();
            bool transformWaypointsToOdomFrame();
            void alignmentStatusCallback(const std_msgs::msg::Bool::SharedPtr msg);
            void alignmentTransformCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);

            // Diagnostics
            std::map<std::string, std::chrono::nanoseconds> section_durations_;
            diagnostic_updater::Updater updater_;
            bool enable_diagnostics_ = false;

            // Subscriptions
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr manual_target_sub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alignment_status_sub_;
            rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr alignment_transform_sub_;

            // TF2 for map alignment
            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

            // Alignment state
            std::atomic<bool> is_alignment_received_ = false;
            bool waypoints_transformed_ = false;
            geometry_msgs::msg::TransformStamped cached_alignment_transform_;

            // Publishers
            rclcpp::Publisher<PolynomialTraj>::SharedPtr poly_traj_pub_;
            rclcpp::Publisher<MincoTraj>::SharedPtr minco_traj_pub_;

            // Timer
            rclcpp::TimerBase::SharedPtr init_timer;
            rclcpp::TimerBase::SharedPtr safety_timer;

            // Callbacks
            void checkCollisionCallback();

            // Action Server
            rclcpp_action::Server<EgoPlannerAction>::SharedPtr action_server_;
            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const EgoPlannerGoal> goal);
            rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<EgoPlannerServerGoalHandle> goal_handle);
            void handle_accepted(const std::shared_ptr<EgoPlannerServerGoalHandle> goal_handle);

            // Core Functions
            void mainPlanningLoop(const std::shared_ptr<EgoPlannerServerGoalHandle> goal_handle);
            bool planFromLocalTraj();
            bool planFromGlobalTraj();
            bool planNextWaypoint(const Eigen::Vector3d next_wp);
            bool callReboundReplan(bool flag_use_poly_init, bool flag_random_poly_traj);
            bool computeInitState(
                const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
                const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
                const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
                const bool flag_randomPolyTraj, const double &ts, poly_traj::MinJerkOpt &initMJO);
            bool reboundReplan(
                const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
                const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_pt,
                const Eigen::Vector3d &end_vel, const bool flag_polyInit,
                const bool flag_randomPolyTraj, const bool touch_goal);
            bool planGlobalTrajWaypoints(
                const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
                const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
                const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
            void getLocalTarget(
                const double planning_horizon,
                const Eigen::Vector3d &start_pt, const Eigen::Vector3d &global_end_pt,
                Eigen::Vector3d &local_target_pos, Eigen::Vector3d &local_target_vel,
                bool &touch_goal);
            void callEmergencyStop(Eigen::Vector3d stop_pos);
            bool checkCollision(int drone_id);
            bool setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal);
            inline double getSwarmClearance(void) { return poly_traj_opt_->get_swarm_clearance_(); }
            inline int getCpsNumPerPiece(void) { return poly_traj_opt_->get_cps_num_per_piece_(); }

            // State Handlers
            void handleIDLE(void);
            void handleSEQUENTIAL_START(void);
            void handleGEN_NEW_TRAJ(void);
            void handleREPLAN_TRAJ(void);
            void handleEXEC_TRAJ(void);
            void handleEMERGENCY_STOP(void);

        }; // class EgoPlanner
    } // namespace path_planning
} // namespace uosm
#endif // EGO_PLANNER_COMPONENT_HPP_