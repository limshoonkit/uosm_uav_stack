#ifndef FLIGHT_CONTROLLER_COMPONENT_HPP_
#define FLIGHT_CONTROLLER_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/command_home.hpp>
#include <mavros_msgs/msg/position_target.hpp>

#include <array>
#include <atomic>
#include <mutex>
#include <Eigen/Dense>
#include <std_msgs/msg/bool.hpp>
#include "poly_traj_utils.hpp"
#include "uosm_uav_interface/action/ego_planner.hpp"
#include "uosm_uav_interface/msg/polynomial_traj.hpp"

// Forward declaration — only instantiated when controller_mode == "nmpc"
namespace nmpc_controller {
    class NmpcController;
    struct NmpcParams;
    struct NmpcState;
}

namespace uosm
{
    namespace path_planning
    {
        using EgoPlannerAction = uosm_uav_interface::action::EgoPlanner;
        using EgoPlannerGoal = uosm_uav_interface::action::EgoPlanner::Goal;
        using EgoPlannerClientGoalHandle = rclcpp_action::ClientGoalHandle<EgoPlannerAction>;
        using PolynomialTraj = uosm_uav_interface::msg::PolynomialTraj;

        class FlightController : public rclcpp::Node
        {
        public:
            enum class State
            {
                DISARM = 0,
                REQUEST_OFFBOARD_ARM,
                HOVER,
                EXECUTE_PLANNER,
                LAND
            };

            explicit FlightController(const rclcpp::NodeOptions &options);
            ~FlightController();

        private:
            State current_state_ = State::DISARM;

            mavros_msgs::msg::State current_mavros_state_;
            mavros_msgs::msg::ExtendedState current_extended_state_;
            mavros_msgs::msg::PositionTarget raw_setpoint_;

            // Variables
            std::shared_ptr<poly_traj::Trajectory> traj_;
            rclcpp::Time traj_start_time_ = rclcpp::Time(0);
            double traj_duration_ = 0.0;
            double last_x_ = 0.0;
            double last_y_ = 0.0;
            double last_z_ = 0.0;
            double last_yaw_ = 0.0;
            double time_forward_ = 0.0;

            // Controller mode: "feedforward" or "nmpc"
            std::string controller_mode_ = "feedforward";

            double request_timeout_ = 0.0;
            double hover_period_ = 0.0;
            double preflight_check_timeout_ = 0.0;
            std::string frame_id_ = "";
            uint8_t service_result_;
            int max_manual_target_allowed_ = 0;

            // Flags
            bool has_odom_ = false;
            bool is_mission_complete_ = false;
            bool is_action_send_ = false;
            bool is_traj_received_ = false;
            bool is_manual_target_ = false;
            double goal_yaw_ = 0.0;
            bool has_goal_yaw_ = false;
            bool offboard_requested_ = false;
            bool arming_requested_ = false;
            bool landing_requested_ = false;
            int server_rejected_count_ = 0;
            int manual_target_count_ = 0;

            // Home position
            double home_lat_ = 0.0;
            double home_lon_ = 0.0;
            double home_alt_ = 0.0;
            bool home_position_set_ = false;

            // Map alignment state
            std::atomic<bool> is_alignment_received_ = false;
            bool wait_for_alignment_ = true;
            double alignment_timeout_ = 5.0;

            // NMPC members
            std::unique_ptr<nmpc_controller::NmpcController> nmpc_solver_;
            // State prediction for NMPC (30 Hz odom → 100 Hz control)
            nav_msgs::msg::Odometry latest_odom_;
            rclcpp::Time last_odom_time_ = rclcpp::Time(0);
            Eigen::Vector3d prev_acc_ = Eigen::Vector3d::Zero();
            bool use_state_prediction_ = true;
            std::mutex odom_mutex_;
            std::mutex traj_mutex_;

            // Helper functions
            bool loadWaypointsFromCSV();
            void mavrosExecutionLoop();
            void publishRawSetpointMavros();
            void publishRawSetpointNmpc();
            double calculateYaw(double t_cur, const Eigen::Vector3d &pos) const;

            // Interface
            void requestOffboardModeMavros();
            void requestArmingMavros();
            void requestLandingMavros();
            void requestEgoPlanner();
            void requestSetHome();

            // Publishers
            rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr raw_setpoint_pub_;

            // Subscribers
            rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_;
            rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_state_sub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
            rclcpp::Subscription<PolynomialTraj>::SharedPtr poly_traj_sub_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alignment_status_sub_;
            // Continuous odom subscription for NMPC state feedback
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr continuous_odom_sub_;

            // Services
            rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
            rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
            rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
            rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedPtr set_home_client_;

            // Action
            rclcpp_action::Client<EgoPlannerAction>::SharedPtr ego_planner_client_;
            EgoPlannerClientGoalHandle::SharedPtr ego_planner_goal_handle_;

            // Timers
            rclcpp::TimerBase::SharedPtr mission_exec_timer_;
            rclcpp::TimerBase::SharedPtr raw_setpoint_pub_timer_;
            rclcpp::Time mission_start_time_;
            rclcpp::Time last_request_time_;

            // Callbacks
            void executeMissionCallback();
            void publishRawSetpointCallback();

        }; // class FlightController
    } // namespace path_planning
} // namespace uosm

#endif // FLIGHT_CONTROLLER_COMPONENT_HPP_
