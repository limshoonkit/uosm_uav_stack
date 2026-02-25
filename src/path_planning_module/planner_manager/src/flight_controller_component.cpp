#include "flight_controller_component.hpp"
#include <nmpc_controller/nmpc_controller.hpp>
#include <node_params.hpp>

namespace uosm
{
    namespace path_planning
    {
        FlightController::FlightController(const rclcpp::NodeOptions &options)
            : Node("flight_controller_node", options)
        {
            RCLCPP_INFO(get_logger(), "********************************");
            RCLCPP_INFO(get_logger(), "  Flight Controller Component   ");
            RCLCPP_INFO(get_logger(), "********************************");
            RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
            RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
            RCLCPP_INFO(get_logger(), "********************************");

            using uosm::util::getParam;

            RCLCPP_INFO(get_logger(), "Flight Controller Parameters:");
            controller_mode_ = getParam<std::string>(this, "fc.controller_mode", "feedforward", " * Controller mode: ");
            auto starting_yaw_deg = getParam<double>(this, "fc.starting_yaw_deg", 0.0, " * Starting yaw deg: ");
            time_forward_ = getParam<double>(this, "fc.time_forward", 1.0, " * Time forward: ");
            auto takeoff_height = getParam<double>(this, "fc.takeoff_height", 2.0, " * Takeoff height: ");
            auto state_machine_rate = getParam<double>(this, "fc.state_machine_rate", 20.0, " * State machine rate: ");
            auto setpoint_publish_rate = getParam<double>(this, "fc.setpoint_publish_rate", 100.0, " * Setpoint publish rate: ");
            request_timeout_ = getParam<double>(this, "fc.request_timeout", 5.0, " * Request timeout: ");
            hover_period_ = getParam<double>(this, "fc.hover_period", 5.0, " * Hover period: ");
            preflight_check_timeout_ = getParam<double>(this, "fc.preflight_check_timeout", 3.0, " * Preflight check timeout: ");
            max_manual_target_allowed_ = getParam<int>(this, "fc.max_manual_target_allowed", 2, " * Max manual target: ");
            frame_id_ = getParam<std::string>(this, "fc.frame_id", "map", " * Frame ID: ");
            wait_for_alignment_ = getParam<bool>(this, "fc.wait_for_alignment", true, " * Wait for alignment: ");
            alignment_timeout_ = getParam<double>(this, "fc.alignment_timeout", 5.0, " * Alignment timeout: ");
            RCLCPP_INFO(get_logger(), "********************************");

            // NMPC parameters (declared unconditionally so YAML loading doesn't error)
            declare_parameter("fc.nmpc/horizon_steps", 20);
            declare_parameter("fc.nmpc/dt", 0.05);
            declare_parameter("fc.nmpc/w_pos", std::vector<double>{80.0, 80.0, 120.0});
            declare_parameter("fc.nmpc/w_vel", std::vector<double>{10.0, 10.0, 15.0});
            declare_parameter("fc.nmpc/w_acc", std::vector<double>{5.0, 5.0, 8.0});
            declare_parameter("fc.nmpc/w_jerk", std::vector<double>{1.0, 1.0, 1.0});
            declare_parameter("fc.nmpc/v_max", std::vector<double>{1.0, 1.0, 0.5});
            declare_parameter("fc.nmpc/a_max", std::vector<double>{3.0, 3.0, 2.0});
            declare_parameter("fc.nmpc/j_max", std::vector<double>{10.0, 10.0, 8.0});
            declare_parameter("fc.nmpc/use_state_prediction", true);

            if (controller_mode_ != "feedforward" && controller_mode_ != "nmpc")
            {
                RCLCPP_WARN(get_logger(), "Invalid controller_mode '%s'. Defaulting to 'feedforward'.",
                            controller_mode_.c_str());
                controller_mode_ = "feedforward";
            }

            mission_start_time_ = get_clock()->now();
            last_request_time_ = get_clock()->now();
            last_yaw_ = starting_yaw_deg * M_PI / 180.0;
            last_x_ = 0.0;
            last_y_ = 0.0;
            last_z_ = takeoff_height;

            // --- Initialize NMPC solver if needed ---
            if (controller_mode_ == "nmpc")
            {
                nmpc_controller::NmpcParams nmpc_params;
                nmpc_params.horizon_steps = get_parameter("fc.nmpc/horizon_steps").as_int();
                nmpc_params.dt = get_parameter("fc.nmpc/dt").as_double();

                auto w_pos = get_parameter("fc.nmpc/w_pos").as_double_array();
                auto w_vel = get_parameter("fc.nmpc/w_vel").as_double_array();
                auto w_acc = get_parameter("fc.nmpc/w_acc").as_double_array();
                auto w_jerk = get_parameter("fc.nmpc/w_jerk").as_double_array();
                auto v_max = get_parameter("fc.nmpc/v_max").as_double_array();
                auto a_max = get_parameter("fc.nmpc/a_max").as_double_array();
                auto j_max = get_parameter("fc.nmpc/j_max").as_double_array();

                for (int i = 0; i < 3; ++i)
                {
                    nmpc_params.w_pos[i] = w_pos[i];
                    nmpc_params.w_vel[i] = w_vel[i];
                    nmpc_params.w_acc[i] = w_acc[i];
                    nmpc_params.w_jerk[i] = w_jerk[i];
                    nmpc_params.v_max[i] = v_max[i];
                    nmpc_params.a_max[i] = a_max[i];
                    nmpc_params.j_max[i] = j_max[i];
                }

                use_state_prediction_ = get_parameter("fc.nmpc/use_state_prediction").as_bool();

                nmpc_solver_ = std::make_unique<nmpc_controller::NmpcController>(nmpc_params);
                RCLCPP_INFO(get_logger(), "NMPC solver initialized (N=%d, dt=%.3f)",
                            nmpc_params.horizon_steps, nmpc_params.dt);
            }

            // --- Odometry subscription (one-shot for has_odom_ flag) ---
            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                "/odometry",
                rclcpp::QoS(10),
                [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                {
                    has_odom_ = true;
                    RCLCPP_INFO(this->get_logger(), "Received initial odometry, unsubscribing.");
                    odom_sub_.reset();
                });

            // --- Continuous odom subscription for NMPC state feedback ---
            if (controller_mode_ == "nmpc")
            {
                continuous_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                    "/odometry",
                    rclcpp::QoS(10),
                    [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                    {
                        std::lock_guard<std::mutex> lock(odom_mutex_);
                        latest_odom_ = *msg;
                        last_odom_time_ = msg->header.stamp;
                    });
            }

            // --- Alignment status subscription ---
            alignment_status_sub_ = create_subscription<std_msgs::msg::Bool>(
                "alignment_done",
                rclcpp::QoS(1),
                [this](const std_msgs::msg::Bool::SharedPtr msg)
                {
                    if (msg->data && !is_alignment_received_.load())
                    {
                        RCLCPP_INFO(this->get_logger(), "Map alignment complete - ready to start planner");
                        is_alignment_received_ = true;
                    }
                });

            // --- MAVROS setup ---
            current_mavros_state_.connected = false;
            current_mavros_state_.armed = false;
            current_mavros_state_.mode = "";
            current_extended_state_.landed_state = mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED;

            raw_setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", 10);
            mavros_state_sub_ = create_subscription<mavros_msgs::msg::State>(
                "/mavros/state", rclcpp::QoS(1),
                [this](const mavros_msgs::msg::State::SharedPtr msg)
                {
                    current_mavros_state_ = *msg;
                });
            extended_state_sub_ = create_subscription<mavros_msgs::msg::ExtendedState>(
                "/mavros/extended_state", rclcpp::QoS(1),
                [this](const mavros_msgs::msg::ExtendedState::SharedPtr msg)
                {
                    current_extended_state_ = *msg;
                });
            arming_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
            set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
            land_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");

            // --- Polynomial trajectory subscription ---
            poly_traj_sub_ = create_subscription<uosm_uav_interface::msg::PolynomialTraj>(
                "ego_planner/poly_traj", rclcpp::QoS(5),
                [this](const uosm_uav_interface::msg::PolynomialTraj::SharedPtr msg)
                {
                    if (msg->order != 5)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Only support trajectory order equals 5 now!");
                        return;
                    }

                    int piece_nums = msg->duration.size();
                    std::vector<double> duration(piece_nums);
                    std::vector<poly_traj::CoefficientMat> cMats(piece_nums);

                    size_t i6 = 0;
                    auto coef_x = msg->coef_x;
                    auto coef_y = msg->coef_y;
                    auto coef_z = msg->coef_z;

                    for (int i = 0; i < piece_nums; ++i)
                    {
                        duration[i] = msg->duration[i];
                        cMats[i] << coef_x[i6], coef_x[i6 + 1], coef_x[i6 + 2], coef_x[i6 + 3], coef_x[i6 + 4], coef_x[i6 + 5],
                            coef_y[i6], coef_y[i6 + 1], coef_y[i6 + 2], coef_y[i6 + 3], coef_y[i6 + 4], coef_y[i6 + 5],
                            coef_z[i6], coef_z[i6 + 1], coef_z[i6 + 2], coef_z[i6 + 3], coef_z[i6 + 4], coef_z[i6 + 5];
                        i6 += 6;
                    }

                    {
                        std::lock_guard<std::mutex> lock(traj_mutex_);
                        traj_ = std::make_shared<poly_traj::Trajectory>(std::move(duration), std::move(cMats));
                        traj_start_time_ = msg->start_time;
                        traj_duration_ = traj_->getTotalDuration();
                    }
                    is_traj_received_ = true;
                    has_goal_yaw_ = false;
                });

            // --- Action Client ---
            ego_planner_client_ = rclcpp_action::create_client<EgoPlannerAction>(
                this,
                "ego_planner_action");

            // --- Timers ---
            mission_exec_timer_ = create_wall_timer(
                std::chrono::duration<double>(1.0 / state_machine_rate),
                std::bind(&FlightController::executeMissionCallback, this));

            raw_setpoint_pub_timer_ = create_wall_timer(
                std::chrono::duration<double>(1.0 / setpoint_publish_rate),
                std::bind(&FlightController::publishRawSetpointCallback, this));
        }

        FlightController::~FlightController() = default;

        // ========================
        // MAVROS service requests
        // ========================

        void FlightController::requestOffboardModeMavros()
        {
            if (!set_mode_client_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(get_logger(), "SetMode service not available");
                return;
            }
            auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            request->custom_mode = mavros_msgs::msg::State::MODE_PX4_OFFBOARD;
            auto future = set_mode_client_->async_send_request(
                request,
                [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
                {
                    auto response = future.get();
                    if (response->mode_sent)
                    {
                        RCLCPP_INFO(get_logger(), "Offboard mode successful");
                        offboard_requested_ = true;
                    }
                    else
                    {
                        RCLCPP_ERROR(get_logger(), "Offboard mode failed");
                    }
                });
        }

        void FlightController::requestArmingMavros()
        {
            if (!arming_client_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(get_logger(), "Arming service not available");
                return;
            }
            auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            request->value = true;
            auto future = arming_client_->async_send_request(
                request,
                [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future)
                {
                    auto response = future.get();
                    if (response->success)
                    {
                        RCLCPP_INFO(get_logger(), "Arming successful");
                        arming_requested_ = true;
                    }
                    else
                    {
                        RCLCPP_ERROR(get_logger(), "Arming failed");
                    }
                });
        }

        void FlightController::requestLandingMavros()
        {
            if (!land_client_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(get_logger(), "Land service not available");
                return;
            }
            auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
            request->latitude = 0.0;
            request->longitude = 0.0;
            request->altitude = 0.0;
            request->min_pitch = 0.0;
            request->yaw = 0.0;
            auto future = land_client_->async_send_request(
                request,
                [this](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future)
                {
                    auto response = future.get();
                    if (response->success)
                    {
                        RCLCPP_INFO(get_logger(), "Landing successful");
                        landing_requested_ = true;
                    }
                    else
                    {
                        RCLCPP_ERROR(get_logger(), "Landing failed");
                    }
                });
        }

        void FlightController::requestEgoPlanner()
        {
            RCLCPP_INFO(get_logger(), "Requesting Ego Planner");
            if (!ego_planner_client_->wait_for_action_server(std::chrono::seconds(static_cast<int>(request_timeout_))))
            {
                RCLCPP_WARN(get_logger(), "Action server not available, waiting...");
                return;
            }
            EgoPlannerGoal goal_msg;
            goal_msg.trigger = true;
            auto send_goal_options = rclcpp_action::Client<EgoPlannerAction>::SendGoalOptions();
            send_goal_options.goal_response_callback =
                [this](const EgoPlannerClientGoalHandle::SharedPtr &goal_handle)
            {
                if (!goal_handle)
                {
                    RCLCPP_ERROR(get_logger(), "Ego planner server rejected!");
                    server_rejected_count_++;
                    is_action_send_ = false;
                    ego_planner_goal_handle_.reset();
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "Ego planner server accepted, waiting for result!");
                    is_action_send_ = true;
                    ego_planner_goal_handle_ = goal_handle;
                }
            };

            send_goal_options.feedback_callback =
                [this](EgoPlannerClientGoalHandle::SharedPtr,
                       const std::shared_ptr<const EgoPlannerAction::Feedback> feedback)
            {
                is_manual_target_ = feedback->is_manual_target;
                if (is_manual_target_)
                {
                    goal_yaw_ = feedback->goal_yaw;
                    has_goal_yaw_ = true;
                }
            };

            send_goal_options.result_callback =
                [this](const EgoPlannerClientGoalHandle::WrappedResult &result)
            {
                is_mission_complete_ = result.result->success;
                is_action_send_ = false;
                ego_planner_goal_handle_.reset();

                if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                {
                    RCLCPP_INFO(get_logger(), "Ego Planner goal succeeded!");
                }
                else if (result.code == rclcpp_action::ResultCode::ABORTED)
                {
                    RCLCPP_WARN(get_logger(), "Ego Planner goal aborted!");
                }
                else if (result.code == rclcpp_action::ResultCode::CANCELED)
                {
                    RCLCPP_INFO(get_logger(), "Ego Planner goal canceled!");
                }
            };

            ego_planner_client_->async_send_goal(goal_msg, send_goal_options);
        }

        // ========================
        // State machine
        // ========================

        void FlightController::executeMissionCallback()
        {
            if (!has_odom_)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for odometry...");
                return;
            }
            mavrosExecutionLoop();
        }

        void FlightController::mavrosExecutionLoop()
        {
            switch (current_state_)
            {
            case State::DISARM:
            {
                RCLCPP_INFO_ONCE(get_logger(), "STATE: DISARM");
                if (is_mission_complete_ &&
                    current_extended_state_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND)
                {
                    RCLCPP_INFO(get_logger(), "MISSION COMPLETE!!! ^-^");
                    mission_exec_timer_->cancel();
                    rclcpp::shutdown();
                    return;
                }
                if (current_mavros_state_.connected &&
                    !current_mavros_state_.armed &&
                    current_mavros_state_.mode != mavros_msgs::msg::State::MODE_PX4_OFFBOARD)
                {
                    if ((get_clock()->now() - last_request_time_).seconds() > preflight_check_timeout_)
                    {
                        current_state_ = State::REQUEST_OFFBOARD_ARM;
                        last_request_time_ = get_clock()->now();
                        offboard_requested_ = false;
                        arming_requested_ = false;
                    }
                }
                break;
            }
            case State::REQUEST_OFFBOARD_ARM:
            {
                RCLCPP_INFO_ONCE(get_logger(), "STATE: REQUEST_OFFBOARD_ARM");
                if ((get_clock()->now() - last_request_time_).seconds() > request_timeout_)
                {
                    if (!offboard_requested_)
                    {
                        requestOffboardModeMavros();
                    }
                    if (!arming_requested_)
                    {
                        requestArmingMavros();
                    }
                    last_request_time_ = get_clock()->now();
                }
                if (current_mavros_state_.mode == mavros_msgs::msg::State::MODE_PX4_OFFBOARD &&
                    current_mavros_state_.armed)
                {
                    RCLCPP_INFO(get_logger(), "Successfully armed in OFFBOARD mode");
                    current_state_ = State::HOVER;
                    mission_start_time_ = get_clock()->now();
                }
                break;
            }
            case State::HOVER:
            {
                RCLCPP_INFO_ONCE(get_logger(), "STATE: HOVER");
                double hover_time = (get_clock()->now() - mission_start_time_).seconds();
                bool hover_period_elapsed = hover_time > hover_period_;
                bool alignment_ready = !wait_for_alignment_ || is_alignment_received_.load();
                bool alignment_timed_out = wait_for_alignment_ &&
                                           !is_alignment_received_.load() &&
                                           hover_time > alignment_timeout_;

                if (!alignment_ready && !alignment_timed_out)
                {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                         "Waiting for map alignment... (hover time: %.1fs / timeout: %.1fs)",
                                         hover_time, alignment_timeout_);
                }
                if (alignment_timed_out)
                {
                    RCLCPP_ERROR(get_logger(), "Map alignment FAILED - timeout after %.1fs. Landing for safety.",
                                 hover_time);
                    current_state_ = State::LAND;
                    landing_requested_ = false;
                    break;
                }
                if (hover_period_elapsed && alignment_ready)
                {
                    if (current_extended_state_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR)
                    {
                        current_state_ = State::EXECUTE_PLANNER;
                        manual_target_count_ = 0;
                        RCLCPP_INFO(get_logger(), "Starting planner execution (alignment: %s)",
                                    is_alignment_received_.load() ? "complete" : "skipped");
                    }
                }
                break;
            }
            case State::EXECUTE_PLANNER:
            {
                RCLCPP_INFO_ONCE(get_logger(), "STATE: EXECUTE_PLANNER");
                if (server_rejected_count_ > 3)
                {
                    RCLCPP_WARN(get_logger(), "Planner error - too many rejections, landing");
                    current_state_ = State::LAND;
                    break;
                }
                if (is_mission_complete_ &&
                    is_manual_target_ &&
                    manual_target_count_ <= max_manual_target_allowed_)
                {
                    RCLCPP_INFO(get_logger(), "Manual target %d received, setting new waypoint", manual_target_count_);
                    is_mission_complete_ = false;
                    ++manual_target_count_;
                    is_traj_received_ = false;
                }
                else if (is_mission_complete_)
                {
                    RCLCPP_INFO(get_logger(), "Mission complete or max manual targets reached, landing");
                    current_state_ = State::LAND;
                    landing_requested_ = false;
                    break;
                }
                if (!is_mission_complete_ &&
                    !is_action_send_ &&
                    !ego_planner_goal_handle_ &&
                    (get_clock()->now() - last_request_time_).seconds() > request_timeout_)
                {
                    requestEgoPlanner();
                    last_request_time_ = get_clock()->now();
                }
                break;
            }
            case State::LAND:
            {
                RCLCPP_INFO_ONCE(get_logger(), "STATE: LAND");
                if ((get_clock()->now() - last_request_time_).seconds() > request_timeout_)
                {
                    requestLandingMavros();
                    last_request_time_ = get_clock()->now();
                }
                if (current_mavros_state_.mode != mavros_msgs::msg::State::MODE_PX4_OFFBOARD &&
                    current_mavros_state_.mode == mavros_msgs::msg::State::MODE_PX4_LAND &&
                    current_extended_state_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_LANDING)
                {
                    current_state_ = State::DISARM;
                }
                break;
            }
            default:
                RCLCPP_ERROR(get_logger(), "Unknown state!");
                break;
            }
        }

        // ========================
        // Setpoint publishing
        // ========================

        void FlightController::publishRawSetpointCallback()
        {
            if (!has_odom_)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for odometry before publishing setpoint...");
                return;
            }

            if (controller_mode_ == "nmpc")
            {
                publishRawSetpointNmpc();
            }
            else
            {
                publishRawSetpointMavros();
            }
        }

        void FlightController::publishRawSetpointMavros()
        {
            rclcpp::Time time_now = get_clock()->now();
            raw_setpoint_.header.stamp = time_now;
            raw_setpoint_.header.frame_id = frame_id_;
            raw_setpoint_.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

            if (is_traj_received_ && current_state_ == State::EXECUTE_PLANNER)
            {
                double t_cur;
                Eigen::Vector3d pos(Eigen::Vector3d::Zero());
                Eigen::Vector3d vel(Eigen::Vector3d::Zero());
                Eigen::Vector3d acc(Eigen::Vector3d::Zero());

                {
                    std::lock_guard<std::mutex> lock(traj_mutex_);
                    t_cur = (time_now - traj_start_time_).seconds();
                    if (t_cur < traj_duration_ && t_cur >= 0.0)
                    {
                        pos = traj_->getPos(t_cur);
                        vel = traj_->getVel(t_cur);
                        acc = traj_->getAcc(t_cur);
                    }
                }

                if (t_cur < traj_duration_ && t_cur >= 0.0)
                {
                    const double yaw = calculateYaw(t_cur, pos);

                    raw_setpoint_.type_mask = raw_setpoint_.IGNORE_YAW_RATE;
                    raw_setpoint_.position.x = pos.x();
                    raw_setpoint_.position.y = pos.y();
                    raw_setpoint_.position.z = pos.z();
                    raw_setpoint_.velocity.x = vel.x();
                    raw_setpoint_.velocity.y = vel.y();
                    raw_setpoint_.velocity.z = vel.z();
                    raw_setpoint_.acceleration_or_force.x = acc.x();
                    raw_setpoint_.acceleration_or_force.y = acc.y();
                    raw_setpoint_.acceleration_or_force.z = acc.z();
                    raw_setpoint_.yaw = yaw;

                    last_x_ = pos.x();
                    last_y_ = pos.y();
                    last_z_ = pos.z();
                    last_yaw_ = yaw;
                }
            }
            else
            {
                raw_setpoint_.type_mask =
                    mavros_msgs::msg::PositionTarget::IGNORE_VX |
                    mavros_msgs::msg::PositionTarget::IGNORE_VY |
                    mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
                raw_setpoint_.position.x = last_x_;
                raw_setpoint_.position.y = last_y_;
                raw_setpoint_.position.z = last_z_;
                raw_setpoint_.yaw = has_goal_yaw_ ? goal_yaw_ : last_yaw_;
            }
            raw_setpoint_pub_->publish(raw_setpoint_);
        }

        void FlightController::publishRawSetpointNmpc()
        {
            rclcpp::Time time_now = get_clock()->now();
            raw_setpoint_.header.stamp = time_now;
            raw_setpoint_.header.frame_id = frame_id_;
            raw_setpoint_.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

            if (is_traj_received_ && current_state_ == State::EXECUTE_PLANNER && nmpc_solver_)
            {
                double t_cur;
                double traj_dur;
                std::shared_ptr<poly_traj::Trajectory> traj_local;
                {
                    std::lock_guard<std::mutex> lock(traj_mutex_);
                    t_cur = (time_now - traj_start_time_).seconds();
                    traj_dur = traj_duration_;
                    traj_local = traj_;
                }

                if (t_cur >= 0.0 && t_cur < traj_dur && traj_local)
                {
                    // --- State estimation with prediction ---
                    Eigen::Vector3d current_pos, current_vel, current_acc;
                    {
                        std::lock_guard<std::mutex> lock(odom_mutex_);
                        current_pos << latest_odom_.pose.pose.position.x,
                            latest_odom_.pose.pose.position.y,
                            latest_odom_.pose.pose.position.z;
                        current_vel << latest_odom_.twist.twist.linear.x,
                            latest_odom_.twist.twist.linear.y,
                            latest_odom_.twist.twist.linear.z;
                    }
                    current_acc = prev_acc_;

                    // Model-based state prediction between odom updates
                    if (use_state_prediction_ && last_odom_time_.nanoseconds() > 0)
                    {
                        double dt_pred = (time_now - last_odom_time_).seconds();
                        if (dt_pred > 0.0 && dt_pred < 0.2) // cap at 200ms staleness
                        {
                            current_pos += current_vel * dt_pred + 0.5 * prev_acc_ * dt_pred * dt_pred;
                            current_vel += prev_acc_ * dt_pred;
                        }
                    }

                    // --- Set current state ---
                    nmpc_controller::NmpcState x0;
                    x0.pos = current_pos;
                    x0.vel = current_vel;
                    x0.acc = current_acc;
                    nmpc_solver_->setCurrentState(x0);

                    // --- Sample reference trajectory over horizon ---
                    int N = nmpc_solver_->getHorizonSteps();
                    double dt_horizon = nmpc_solver_->getHorizonDt();
                    std::vector<nmpc_controller::NmpcState> refs(N + 1);
                    for (int k = 0; k <= N; ++k)
                    {
                        double t_ref = t_cur + k * dt_horizon;
                        if (t_ref > traj_dur)
                            t_ref = traj_dur;
                        refs[k].pos = traj_local->getPos(t_ref);
                        refs[k].vel = traj_local->getVel(t_ref);
                        refs[k].acc = traj_local->getAcc(t_ref);
                    }
                    nmpc_solver_->setReference(refs);

                    // --- Solve ---
                    int status = nmpc_solver_->solve();
                    if (status != 0)
                    {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                             "NMPC solver returned status %d, falling back to feedforward", status);
                        // Fallback: use feedforward at t_cur
                        Eigen::Vector3d pos = traj_local->getPos(t_cur);
                        Eigen::Vector3d vel = traj_local->getVel(t_cur);
                        Eigen::Vector3d acc = traj_local->getAcc(t_cur);
                        const double yaw = calculateYaw(t_cur, pos);

                        raw_setpoint_.type_mask = raw_setpoint_.IGNORE_YAW_RATE;
                        raw_setpoint_.position.x = pos.x();
                        raw_setpoint_.position.y = pos.y();
                        raw_setpoint_.position.z = pos.z();
                        raw_setpoint_.velocity.x = vel.x();
                        raw_setpoint_.velocity.y = vel.y();
                        raw_setpoint_.velocity.z = vel.z();
                        raw_setpoint_.acceleration_or_force.x = acc.x();
                        raw_setpoint_.acceleration_or_force.y = acc.y();
                        raw_setpoint_.acceleration_or_force.z = acc.z();
                        raw_setpoint_.yaw = yaw;

                        last_x_ = pos.x();
                        last_y_ = pos.y();
                        last_z_ = pos.z();
                        last_yaw_ = yaw;
                    }
                    else
                    {
                        // Extract optimal setpoint from stage 1
                        nmpc_controller::NmpcState optimal = nmpc_solver_->getOptimalSetpoint();
                        prev_acc_ = optimal.acc; // store for next prediction

                        const double yaw = calculateYaw(t_cur, optimal.pos);

                        raw_setpoint_.type_mask = raw_setpoint_.IGNORE_YAW_RATE;
                        raw_setpoint_.position.x = optimal.pos.x();
                        raw_setpoint_.position.y = optimal.pos.y();
                        raw_setpoint_.position.z = optimal.pos.z();
                        raw_setpoint_.velocity.x = optimal.vel.x();
                        raw_setpoint_.velocity.y = optimal.vel.y();
                        raw_setpoint_.velocity.z = optimal.vel.z();
                        raw_setpoint_.acceleration_or_force.x = optimal.acc.x();
                        raw_setpoint_.acceleration_or_force.y = optimal.acc.y();
                        raw_setpoint_.acceleration_or_force.z = optimal.acc.z();
                        raw_setpoint_.yaw = yaw;

                        last_x_ = optimal.pos.x();
                        last_y_ = optimal.pos.y();
                        last_z_ = optimal.pos.z();
                        last_yaw_ = yaw;
                    }
                }
            }
            else
            {
                // Hold position
                raw_setpoint_.type_mask =
                    mavros_msgs::msg::PositionTarget::IGNORE_VX |
                    mavros_msgs::msg::PositionTarget::IGNORE_VY |
                    mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
                raw_setpoint_.position.x = last_x_;
                raw_setpoint_.position.y = last_y_;
                raw_setpoint_.position.z = last_z_;
                raw_setpoint_.yaw = has_goal_yaw_ ? goal_yaw_ : last_yaw_;
            }
            raw_setpoint_pub_->publish(raw_setpoint_);
        }

        double FlightController::calculateYaw(double t_cur, const Eigen::Vector3d &pos) const
        {
            Eigen::Vector3d look_ahead_pos;
            {
                std::lock_guard<std::mutex> lock(const_cast<std::mutex &>(traj_mutex_));
                if (t_cur + time_forward_ <= traj_duration_)
                {
                    look_ahead_pos = traj_->getPos(t_cur + time_forward_);
                }
                else
                {
                    look_ahead_pos = traj_->getPos(traj_duration_);
                }
            }
            Eigen::Vector3d dir = look_ahead_pos - pos;

            double yaw;
            if (dir.norm() > 0.1)
            {
                yaw = atan2(dir(1), dir(0));
            }
            else
            {
                yaw = last_yaw_;
            }

            if (yaw > M_PI)
                yaw -= 2.0 * M_PI;
            if (yaw < -M_PI)
                yaw += 2.0 * M_PI;

            return yaw;
        }

    } // namespace path_planning
} // namespace uosm

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uosm::path_planning::FlightController)
