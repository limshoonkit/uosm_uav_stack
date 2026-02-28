/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Portions adapted from EGO-Planner-v2: https://github.com/ZJU-FAST-Lab/EGO-Planner-v2
 */

#include "ego_planner_component.hpp"
#include <node_params.hpp>
#include <tf2/utils.h>
#include <regex>

namespace uosm
{
    namespace path_planning
    {
        EgoPlanner::EgoPlanner(const rclcpp::NodeOptions &options)
            : Node("ego_planner_node", options),
              updater_(this)
        {
            RCLCPP_INFO(get_logger(), "********************************");
            RCLCPP_INFO(get_logger(), "  Ego Planner Component         ");
            RCLCPP_INFO(get_logger(), "********************************");
            RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
            RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
            RCLCPP_INFO(get_logger(), "********************************");

            using uosm::util::getParam;

            RCLCPP_INFO(get_logger(), "Ego Planner Parameters:");
            params_.waypoint_csv_file_path_ = getParam<std::string>(this, "planner.waypoint_csv_file_path", "", " * Waypoint CSV: ");
            params_.flight_height_ = getParam<float>(this, "planner.flight_height", 2.0f, " * Flight height: ");
            params_.planner_freq_ = getParam<float>(this, "planner.planner_freq", 50.0f, " * Planner freq: ");
            params_.collision_check_freq_ = getParam<float>(this, "planner.collision_check_freq", 20.0f, " * Collision check freq: ");
            params_.max_vel = getParam<float>(this, "planner.max_vel", 2.0f, " * Max velocity: ");
            params_.max_acc = getParam<float>(this, "planner.max_acc", 6.0f, " * Max acceleration: ");
            params_.orbit_flight_height = getParam<float>(this, "planner.orbit_flight_height", 2.0f, " * Orbit flight height: ");
            params_.orbit_max_vel = getParam<float>(this, "planner.orbit_max_vel", 0.75f, " * Orbit max velocity: ");
            params_.orbit_max_acc = getParam<float>(this, "planner.orbit_max_acc", 2.0f, " * Orbit max acceleration: ");
            params_.emergency_time = getParam<float>(this, "planner.emergency_time", 0.8f, " * Emergency time: ");
            params_.replan_threshold = getParam<float>(this, "planner.replan_threshold", 1.0f, " * Replan threshold: ");
            params_.global_retrials = getParam<int>(this, "planner.global_retrials", 10, " * Global retrials: ");
            params_.local_retrials = getParam<int>(this, "planner.local_retrials", 1, " * Local retrials: ");
            params_.poly_traj_piece_length = getParam<float>(this, "planner.poly_traj_piece_length", 0.0f, " * Poly traj piece len: ");
            params_.feasibility_tolerance = getParam<float>(this, "planner.feasibility_tolerance", 0.0f, " * Feasibility tol: ");
            params_.planning_horizon = getParam<float>(this, "planner.planning_horizon", 7.5f, " * Planning horizon: ");
            params_.use_multitopology_trajs = getParam<bool>(this, "planner.use_multitopology_trajs", false, " * Multitopology trajs: ");
            params_.drone_id = getParam<int>(this, "planner.drone_id", 0, " * Drone ID: ");
            enable_diagnostics_ = getParam<bool>(this, "planner.enable_diagnostics", false, " * Enable diagnostics: ");

            RCLCPP_INFO(get_logger(), "********************************");

            // TODO: add assertions to validate params

            no_replan_threshold_ = 0.5 * params_.emergency_time * params_.max_vel;

            if (!loadWaypointsFromCSV())
            {
                RCLCPP_INFO(get_logger(), "Using manual waypoint selection");
                is_manual_target_ = true;
                manual_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/move_base_simple/goal", rclcpp::QoS(1),
                    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
                    {
                        float z = msg->pose.position.z;
                        if (z <= 0.0f)
                        {
                            RCLCPP_WARN(get_logger(), "Received manual target too low, setting to default height");
                            z = params_.flight_height_;
                        }
                        if (is_init_ && has_odom_ && !has_target_) // only considering sending one waypoint at a time
                        {
                            continuous_failures_count_ = 0;
                            repeated_state_count_ = 0;
                            current_wp_ = 0;
                            waypoints_.clear();
                            waypoints_.emplace_back(Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, z));
                            goal_yaw_ = tf2::getYaw(msg->pose.orientation);
                            planNextWaypoint(waypoints_[current_wp_]);
                        }
                    });
            }

            // TF2 buffer and listener for map alignment
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // subscriptions
            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                "odometry",
                rclcpp::QoS(10),
                [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                {
                    odom_pos_ << msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z;

                    odom_vel_ << msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z;

                    has_odom_ = true;

                    if (is_init_)
                        grid_map_->setOdom(msg);
                });

            alignment_status_sub_ = create_subscription<std_msgs::msg::Bool>(
                "alignment_done",
                rclcpp::QoS(1),
                std::bind(&EgoPlanner::alignmentStatusCallback, this, std::placeholders::_1));

            alignment_transform_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
                "alignment_transform",
                rclcpp::QoS(1),
                std::bind(&EgoPlanner::alignmentTransformCallback, this, std::placeholders::_1));

            // publishers
            poly_traj_pub_ = create_publisher<PolynomialTraj>("ego_planner/poly_traj", rclcpp::QoS(5));
            minco_traj_pub_ = create_publisher<MincoTraj>("ego_planner/minco_traj", rclcpp::QoS(5));

            // timers
            init_timer = create_wall_timer(std::chrono::milliseconds(500), std::bind(&EgoPlanner::initComponents, this)); // 500ms warm up
            safety_timer = create_wall_timer(
                std::chrono::duration<double>(1.0 / params_.collision_check_freq_),
                std::bind(&EgoPlanner::checkCollisionCallback, this));

            action_server_ = rclcpp_action::create_server<EgoPlannerAction>(
                this,
                "ego_planner_action",
                std::bind(&EgoPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&EgoPlanner::handle_cancel, this, std::placeholders::_1),
                std::bind(&EgoPlanner::handle_accepted, this, std::placeholders::_1));

            if (enable_diagnostics_)
            {
                updater_.setHardwareID("EgoPlanner");
                updater_.add("Planner Update Time", [this](diagnostic_updater::DiagnosticStatusWrapper &stat)
                             {
                    for (const auto &pair : section_durations_) {
                        stat.add(pair.first, uosm::util::formatDuration(pair.second));
                    }
                    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Execution timing diagnostics"); });
            }
        }

        /**
         * @brief Initializes all components of the ego planner.
         * The method is called once which initialized all components of the ego planner.
         */
        void EgoPlanner::initComponents()
        {
            init_timer->cancel();
            auto node_ptr = shared_from_this();

            grid_map_ = std::make_shared<ego_planner::GridMap>(node_ptr);
            plan_visualizer_ = std::make_shared<ego_planner::PlanVisualizer>(node_ptr);
            poly_traj_opt_ = std::make_unique<ego_planner::PolyTrajOptimizer>(node_ptr);
            poly_traj_opt_->setEnvironment(grid_map_);
            poly_traj_opt_->setSwarmTrajs(&traj_.swarm_traj);
            poly_traj_opt_->setDroneId(params_.drone_id);
            is_init_ = true;

            modify_goal_tstep_ = grid_map_->getResolution() / params_.max_vel;
            RCLCPP_INFO(get_logger(), "Start to plan, current wp: %d, waypoints size: %d", current_wp_, num_waypoints_);
            if (!is_manual_target_)
            {
                applyParamsForWaypointType(waypoint_type_[current_wp_]);
                planNextWaypoint(waypoints_[current_wp_]);
            }
        }

        void EgoPlanner::applyParamsForWaypointType(const std::string &type)
        {
            static const std::regex orbit_re("orbit", std::regex_constants::icase);
            if (std::regex_search(type, orbit_re))
            {
                params_.max_vel = params_.orbit_max_vel;
                params_.max_acc = params_.orbit_max_acc;
            }
            else
            {
                params_.max_vel = get_parameter("planner.max_vel").as_double();
                params_.max_acc = get_parameter("planner.max_acc").as_double();
            }

            no_replan_threshold_ = 0.5 * params_.emergency_time * params_.max_vel;
            if (is_init_)
            {
                modify_goal_tstep_ = grid_map_->getResolution() / params_.max_vel;
                poly_traj_opt_->setDynamicLimits(params_.max_vel, params_.max_acc);
            }

            RCLCPP_INFO(get_logger(), "Applied '%s' params: max_vel=%.2f, max_acc=%.2f",
                        type.c_str(), params_.max_vel, params_.max_acc);
        }

        void EgoPlanner::checkCollisionCallback()
        {
            // --------- collision check data ----------
            const auto &info_ = traj_.local_traj;
            if (current_state_ == State::IDLE || info_.end_time <= 0 || is_triggered_)
                return;

            const double t_cur = get_clock()->now().seconds() - info_.start_time;
            const ego_planner::PtsChk_t &pts_chk = info_.pts_chk;
            RCLCPP_DEBUG(get_logger(), "Collision check at t = %f", t_cur);

            // ---------- find starting indices ----------
            auto findStartIndices = [&](double t_cur) -> std::pair<int, size_t>
            {
                for (int i = 0; i < static_cast<int>(pts_chk.size()); ++i)
                {
                    for (size_t j = 0; j < pts_chk[i].size(); ++j)
                    {
                        if (pts_chk[i][j].first > t_cur)
                        {
                            return {i, j};
                        }
                    }
                }
                return {static_cast<int>(pts_chk.size()), 0};
            };

            auto [i_start, j_start] = findStartIndices(t_cur);
            if (i_start >= static_cast<int>(pts_chk.size()))
            {
                return;
            }

            // ---------- collision detection helpers ----------
            auto checkStaticCollision = [&](const Eigen::Vector3d &pos) -> bool
            {
                return grid_map_->getInflateOccupancy(pos);
            };

            auto checkSwarmCollision = [&](const Eigen::Vector3d &pos, double t) -> bool
            {
                for (const auto &swarm_traj : traj_.swarm_traj)
                {
                    if (swarm_traj.drone_id == params_.drone_id)
                        continue;

                    double t_X = t + (info_.start_time - swarm_traj.start_time);
                    if (t_X <= 0 || t_X >= swarm_traj.duration)
                        continue;

                    Eigen::Vector3d swarm_pos = swarm_traj.traj.getPos(t_X);
                    double dist = (pos - swarm_pos).norm();
                    double required_dist = getSwarmClearance() + swarm_traj.des_clearance;

                    if (dist < required_dist)
                    {
                        RCLCPP_WARN(get_logger(), "Swarm collision risk: drone %d <-> %d, dist=%.2f",
                                    params_.drone_id, swarm_traj.drone_id, dist);
                        return true;
                    }
                }
                return false;
            };

            auto handleCollision = [&](double collision_time) -> void
            {
                if (planFromLocalTraj())
                {
                    RCLCPP_DEBUG(get_logger(), "Collision avoided by replanning at %.1f%%",
                                 100.0 * collision_time / info_.duration);
                    changeState(State::EXEC_TRAJ);
                    return;
                }

                double time_to_collision = collision_time - t_cur;
                if (time_to_collision < params_.emergency_time)
                {
                    RCLCPP_WARN(get_logger(), "Emergency stop! Collision in %.2fs", time_to_collision);
                    changeState(State::EMERGENCY_STOP);
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Collision detected, replanning trajectory");
                    changeState(State::REPLAN_TRAJ);
                }
            };

            // ---------- check trajectory ----------
            const bool touch_the_end = ((local_target_pt_ - final_goal_).norm() < 1e-2);
            size_t i_end = touch_the_end ? pts_chk.size() : pts_chk.size() * 3 / 4;

            for (size_t i = i_start; i < i_end; ++i)
            {
                size_t j_begin = (i == static_cast<size_t>(i_start)) ? j_start : 0;

                for (size_t j = j_begin; j < pts_chk[i].size(); ++j)
                {
                    double t = pts_chk[i][j].first;
                    const Eigen::Vector3d &pos = pts_chk[i][j].second;

                    bool is_dangerous = checkStaticCollision(pos) || checkSwarmCollision(pos, t);

                    if (is_dangerous)
                    {
                        handleCollision(t);
                        return;
                    }
                }
            }
        }

        void EgoPlanner::changeState(const State new_state)
        {
            if (new_state == current_state_)
                ++repeated_state_count_;
            else
                repeated_state_count_ = 1;

            static const char *state_str[8] = {"IDLE", "SEQUENTIAL_START", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
            int previous_state = static_cast<int>(current_state_);
            current_state_ = new_state;

            RCLCPP_INFO(get_logger(),
                        "Drone: %d, state transition: %s -> %s",
                        params_.drone_id,
                        state_str[previous_state],
                        state_str[static_cast<int>(new_state)]);
        }

        rclcpp_action::GoalResponse EgoPlanner::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const EgoPlannerGoal> goal)
        {
            if (is_triggered_)
            {
                RCLCPP_WARN(get_logger(), "Rejecting - currently triggered");
                return rclcpp_action::GoalResponse::REJECT;
            }
            is_triggered_ = goal->trigger;
            RCLCPP_INFO(get_logger(), "Received request to trigger Ego Planner from uuid %d", uuid.front());
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse EgoPlanner::handle_cancel(const std::shared_ptr<EgoPlannerServerGoalHandle> goal_handle)
        {
            is_triggered_ = goal_handle->get_goal()->trigger;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void EgoPlanner::handle_accepted(const std::shared_ptr<EgoPlannerServerGoalHandle> goal_handle)
        {
            std::thread{std::bind(&EgoPlanner::mainPlanningLoop, this, goal_handle)}.detach();
        }

        bool EgoPlanner::loadWaypointsFromCSV()
        {
            waypoints_.clear();
            waypoint_type_.clear();
            auto csv_file = params_.waypoint_csv_file_path_;
            if (csv_file.empty())
            {
                RCLCPP_WARN(get_logger(), "No CSV file specified");
                return false;
            }

            // Check file extension
            if (csv_file.substr(csv_file.find_last_of(".") + 1) != "csv")
            {
                RCLCPP_WARN(get_logger(), "Waypoint file does not have .csv extension: %s", csv_file.c_str());
                return false;
            }

            RCLCPP_INFO(get_logger(), "Loading waypoints from CSV file: %s", params_.waypoint_csv_file_path_.c_str());
            std::ifstream file(csv_file);
            if (!file.is_open())
            {
                RCLCPP_INFO(get_logger(), "Failed to open CSV file: %s", params_.waypoint_csv_file_path_.c_str());
                return false;
            }
            std::string line;
            int line_num = 0;

            // Skip header line
            if (std::getline(file, line))
            {
                line_num++;
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Failed to read header line from CSV file: %s", csv_file.c_str());
                file.close();
                return false;
            }
            while (std::getline(file, line))
            {
                line_num++;
                std::stringstream ss(line);
                std::string x_str, y_str, state_str;

                if (std::getline(ss, x_str, ',') &&
                    std::getline(ss, y_str, ',') &&
                    std::getline(ss, state_str, ','))
                {
                    try
                    {
                        double x = std::stod(x_str);
                        double y = std::stod(y_str);
                        static const std::regex orbit_csv_re("orbit", std::regex_constants::icase);
                        double z = std::regex_search(state_str, orbit_csv_re) ? params_.orbit_flight_height : params_.flight_height_;
                        waypoints_.emplace_back(x, y, z);
                        waypoint_type_.emplace_back(state_str);
                    }
                    catch (const std::invalid_argument &ia)
                    {
                        RCLCPP_WARN(get_logger(), "Invalid number in CSV file %s at line %d: %s. Skipping line.", csv_file.c_str(), line_num, ia.what());
                    }
                    catch (const std::out_of_range &oor)
                    {
                        RCLCPP_WARN(get_logger(), "Number out of range in CSV file %s at line %d: %s. Skipping line.", csv_file.c_str(), line_num, oor.what());
                    }
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Malformed line %d in CSV file: %s. Expected 3 columns.", line_num, csv_file.c_str());
                }
            }
            file.close();

            if (waypoints_.empty())
            {
                RCLCPP_WARN(get_logger(), "No valid waypoints loaded from CSV file: %s", csv_file.c_str());
                return false;
            }

            num_waypoints_ = waypoints_.size(); // Update waypoint_num_ based on loaded CSV data
            RCLCPP_INFO(get_logger(), "Successfully loaded %d waypoints from CSV: %s",
                        num_waypoints_, csv_file.c_str());
            assert(num_waypoints_ == waypoint_type_.size());
            return true;
        }

        void EgoPlanner::alignmentStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
        {
            if (msg->data && !is_alignment_received_.load())
            {
                RCLCPP_INFO(get_logger(), "Map alignment status received: alignment complete");
                is_alignment_received_ = true;
            }
        }

        void EgoPlanner::alignmentTransformCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
        {
            cached_alignment_transform_ = *msg;
            RCLCPP_INFO(get_logger(), "Received alignment transform: [%.3f, %.3f, %.3f]",
                        msg->transform.translation.x,
                        msg->transform.translation.y,
                        msg->transform.translation.z);
        }

        /**
         * @brief Transform waypoints from map frame to odom frame using the alignment transform.
         * This should be called after alignment is complete and before planning starts.
         * @return true if transformation was successful, false otherwise
         */
        bool EgoPlanner::transformWaypointsToOdomFrame()
        {
            if (waypoints_transformed_)
            {
                RCLCPP_DEBUG(get_logger(), "Waypoints already transformed");
                return true;
            }

            if (waypoints_.empty())
            {
                RCLCPP_WARN(get_logger(), "No waypoints to transform");
                return false;
            }

            // Try to get transform from TF tree first
            geometry_msgs::msg::TransformStamped map_to_odom;
            bool have_transform = false;

            try
            {
                // Get transform: odom = T_odom_map * map_point
                // We need map -> odom transform (parent: map, child: odom)
                // But tf2 lookupTransform(target_frame, source_frame) gives us source -> target
                // So lookupTransform("odom", "map") gives us map -> odom
                map_to_odom = tf_buffer_->lookupTransform("odom", "map", tf2::TimePointZero);
                have_transform = true;
                RCLCPP_INFO(get_logger(), "Using TF tree for map->odom transform");
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(get_logger(), "TF lookup failed: %s. Trying cached transform...", ex.what());

                // Fall back to cached transform from alignment topic
                if (is_alignment_received_.load())
                {
                    // Invert the transform
                    tf2::Transform tf_map_odom;
                    tf2::fromMsg(cached_alignment_transform_.transform, tf_map_odom);
                    tf2::Transform tf_odom_map = tf_map_odom.inverse();

                    map_to_odom.header.frame_id = "odom";
                    map_to_odom.child_frame_id = "map";
                    map_to_odom.transform = tf2::toMsg(tf_odom_map);
                    have_transform = true;
                    RCLCPP_INFO(get_logger(), "Using cached alignment transform (inverted)");
                }
            }

            if (!have_transform)
            {
                RCLCPP_ERROR(get_logger(), "No transform available for waypoint transformation");
                return false;
            }

            RCLCPP_INFO(get_logger(), "Transforming %zu waypoints from map to odom frame", waypoints_.size());
            RCLCPP_INFO(get_logger(), "Transform: translation=[%.3f, %.3f, %.3f]",
                        map_to_odom.transform.translation.x,
                        map_to_odom.transform.translation.y,
                        map_to_odom.transform.translation.z);

            for (size_t i = 0; i < waypoints_.size(); ++i)
            {
                geometry_msgs::msg::PointStamped pt_map, pt_odom;
                pt_map.header.frame_id = "map";
                pt_map.point.x = waypoints_[i].x();
                pt_map.point.y = waypoints_[i].y();
                pt_map.point.z = waypoints_[i].z();

                tf2::doTransform(pt_map, pt_odom, map_to_odom);

                RCLCPP_INFO(get_logger(), "  WP[%zu]: map[%.3f, %.3f, %.3f] -> odom[%.3f, %.3f, %.3f]",
                            i, waypoints_[i].x(), waypoints_[i].y(), waypoints_[i].z(),
                            pt_odom.point.x, pt_odom.point.y, pt_odom.point.z);

                waypoints_[i] = Eigen::Vector3d(pt_odom.point.x, pt_odom.point.y, pt_odom.point.z);
            }

            waypoints_transformed_ = true;
            RCLCPP_INFO(get_logger(), "Waypoint transformation complete");
            return true;
        }

        /**
         * @brief Main planning loop
         *
         * This function is responsible for running the main planning loop. It is called when a goal is accepted.
         *
         * @param goal_handle The goal handle to publish feedback to and set the result of.
         */
        void EgoPlanner::mainPlanningLoop(const std::shared_ptr<EgoPlannerServerGoalHandle> goal_handle)
        {
            if (!is_init_ && !has_odom_)
            {
                RCLCPP_ERROR(get_logger(), "Planner not initialized. Aborting mission.");
                goal_handle->abort(std::make_shared<EgoPlannerAction::Result>());
                return;
            }

            // Transform waypoints from map frame to odom frame if not already done
            // This uses the map alignment transform to offset preset waypoints
            if (!is_manual_target_ && !waypoints_transformed_)
            {
                if (!transformWaypointsToOdomFrame())
                {
                    RCLCPP_WARN(get_logger(), "Failed to transform waypoints - proceeding with original coordinates");
                    RCLCPP_WARN(get_logger(), "Waypoints may not be aligned with current odom frame");
                }
            }

            auto feedback = std::make_shared<EgoPlannerAction::Feedback>();
            auto result = std::make_shared<EgoPlannerAction::Result>();

            rclcpp::Rate loop_rate(params_.planner_freq_);

            while (rclcpp::ok())
            {
                if (goal_handle->is_canceling())
                {
                    result->success = false;
                    goal_handle->canceled(result);
                    is_triggered_ = false;
                    RCLCPP_INFO(get_logger(), "Mission canceled");
                    return;
                }

                feedback->current_waypoint_index = current_wp_;
                feedback->is_manual_target = is_manual_target_;
                feedback->goal_yaw = goal_yaw_;
                if (is_manual_target_)
                {
                    feedback->progress = static_cast<uint8_t>(1);
                }
                else
                {
                    feedback->progress = static_cast<uint8_t>(100 * current_wp_ / num_waypoints_);
                }
                goal_handle->publish_feedback(feedback);
                if (current_wp_ >= num_waypoints_ && !is_triggered_)
                {
                    result->success = true;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(get_logger(), "Mission succeeded");
                    return;
                }

                // Display next goal point
                if (waypoints_.size() > 0)
                {
                    ego_planner::PlanVisualizer::DisplayData data;
                    data.scale = 0.30;
                    data.id = current_wp_;
                    data.color = Eigen::Vector4d(0.8, 0.2, 0.2, 0.8);
                    plan_visualizer_->displayGoalPoint(waypoints_[current_wp_], data);
                }

                switch (current_state_)
                {
                case State::IDLE:
                    handleIDLE();
                    break;
                case State::SEQUENTIAL_START:
                    handleSEQUENTIAL_START();
                    break;
                case State::GEN_NEW_TRAJ:
                    handleGEN_NEW_TRAJ();
                    break;
                case State::REPLAN_TRAJ:
                    handleREPLAN_TRAJ();
                    break;
                case State::EXEC_TRAJ:
                    handleEXEC_TRAJ();
                    break;
                case State::EMERGENCY_STOP:
                    handleEMERGENCY_STOP();
                    break;
                default:
                    break;
                }

                loop_rate.sleep();
            }
        }

        void EgoPlanner::handleIDLE()
        {
            if (!has_target_ || !is_triggered_)
            {
                return;
            }
            else
            {
                changeState(State::SEQUENTIAL_START);
            }
        }

        void EgoPlanner::handleSEQUENTIAL_START()
        {
            if (params_.drone_id <= 0)
            {
                if (planFromGlobalTraj())
                {
                    changeState(State::EXEC_TRAJ);
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Failed to generate the first trajectory, keep trying");
                    // Remain in SEQUENTIAL_START state
                }
            }
        }

        void EgoPlanner::handleGEN_NEW_TRAJ()
        {
            if (planFromGlobalTraj())
            {
                changeState(State::EXEC_TRAJ);
                is_in_emergency_ = true;
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Failed to generate new trajectory, retrying...");
                // Remain in GEN_NEW_TRAJ state
            }
        }

        void EgoPlanner::handleREPLAN_TRAJ()
        {
            if (planFromLocalTraj())
            {
                changeState(State::EXEC_TRAJ);
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Failed to replan trajectory, retrying...");
                // Remain in REPLAN_TRAJ state
            }
        }

        void EgoPlanner::handleEXEC_TRAJ()
        {
            if (is_next_waypoint_ready_)
            {
                changeState(State::REPLAN_TRAJ);
                is_next_waypoint_ready_ = false;
                return;
            }

            const auto &local_traj_info = traj_.local_traj;
            double time_since_start = get_clock()->now().seconds() - local_traj_info.start_time;
            double t_cur = std::min(local_traj_info.duration, time_since_start);
            Eigen::Vector3d current_pos = local_traj_info.traj.getPos(t_cur);
            bool is_at_final_goal = (local_target_pt_ - final_goal_).norm() < 1e-2;

            const auto &checkpoints = local_traj_info.pts_chk;
            bool is_near_traj_end = !checkpoints.empty() && !checkpoints.back().empty() &&
                                    (checkpoints.back().back().first - t_cur < params_.emergency_time);
            bool is_ready_for_next_waypoint = (current_wp_ < num_waypoints_ - 1) &&
                                              (final_goal_ - current_pos).norm() < no_replan_threshold_ &&
                                              !is_manual_target_;
            bool reach_goal = (t_cur > local_traj_info.duration - 1e-2) && is_at_final_goal;
            bool is_replan_needed = (t_cur > params_.replan_threshold || (!is_at_final_goal && is_near_traj_end));
            if (modifyGoalPointInObstacles())
            {
                // Pass
                RCLCPP_DEBUG(get_logger(), "Modified goal point in obstacles");
            }
            else if (is_ready_for_next_waypoint)
            {
                // Advance to next wp
                RCLCPP_DEBUG(get_logger(), "Advance to next wp %d", current_wp_ + 1);
                current_wp_++;
                applyParamsForWaypointType(waypoint_type_[current_wp_]);
                planNextWaypoint(waypoints_[current_wp_]);
            }
            else if (reach_goal)
            {
                RCLCPP_DEBUG(get_logger(), "Reach goal");
                current_wp_ = num_waypoints_; // to trigger the mission complete
                has_target_ = false;
                is_triggered_ = false;
                changeState(State::IDLE);
            }
            else if (is_replan_needed)
            {
                changeState(State::REPLAN_TRAJ);
            }
        }

        void EgoPlanner::handleEMERGENCY_STOP()
        {
            if (is_in_emergency_)
            {
                callEmergencyStop(odom_pos_);
                PolynomialTraj poly_msg;
                MincoTraj minco_msg;
                polyTraj2ROSMsg(poly_msg, minco_msg);
                poly_traj_pub_->publish(poly_msg);
                minco_traj_pub_->publish(minco_msg);
            }
            else if (params_.fail_safe && odom_vel_.norm() < 0.1) // STOP_THRESHOLD
            {
                // TODO: FAILSAFE
                changeState(State::GEN_NEW_TRAJ);
            }
            is_in_emergency_ = false;
        }

        bool EgoPlanner::reboundReplan(
            const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
            const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
            const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
            const bool flag_randomPolyTraj, const bool touch_goal)
        {
            if (enable_diagnostics_)
                section_durations_.clear();
            auto t_start = get_clock()->now();
            // rclcpp::Duration t_opt = rclcpp::Duration::from_seconds(0.0);

            static int count = 0;
            RCLCPP_DEBUG(get_logger(), "\033[47;30m\n[%.9f] Drone %d Replan %d\033[0m",
                         t_start.seconds(), params_.drone_id, count++);
            // RCLCPP_INFO(get_logger(), "start: [%f, %f, %f], [%f, %f, %f], goal: [%f, %f, %f], [%f, %f, %f]",
            //             start_pt.x(), start_pt.y(), start_pt.z(), start_vel.x(), start_vel.y(), start_vel.z(),
            //             local_target_pt.x(), local_target_pt.y(), local_target_pt.z(), local_target_vel.x(), local_target_vel.y(), local_target_vel.z());

            /*** STEP 1: INIT ***/
            if (enable_diagnostics_)
                PROFILE_SECTION("computeInitState");
            poly_traj_opt_->setIfTouchGoal(touch_goal);
            double ts = params_.poly_traj_piece_length / params_.max_vel;

            poly_traj::MinJerkOpt initMJO;
            if (!computeInitState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                                  flag_polyInit, flag_randomPolyTraj, ts, initMJO))
            {
                return false;
            }

            Eigen::MatrixXd cstr_pts = initMJO.getInitConstraintPoints(poly_traj_opt_->get_cps_num_per_piece_());

            std::vector<std::pair<int, int>> segments;
            if (poly_traj_opt_->finelyCheckAndSetConstraintPoints(segments, initMJO, true) == ego_planner::PolyTrajOptimizer::CHK_RET::ERR)
            {
                return false;
            }
            // auto t_init = get_clock()->now() - t_start;

            std::vector<Eigen::Vector3d> point_set;
            for (int i = 0; i < cstr_pts.cols(); ++i)
                point_set.push_back(cstr_pts.col(i));

            ego_planner::PlanVisualizer::DisplayData data;
            data.points = point_set;
            data.scale = 0.15;
            data.id = 0;
            data.color = Eigen::Vector4d(0.5, 0.5, 0, 0.8);
            plan_visualizer_->displayInitPathList(data);

            // t_start = get_clock()->now();

            /*** STEP 2: OPTIMIZE ***/
            if (enable_diagnostics_)
                PROFILE_SECTION("optimizeTrajectory");
            bool flag_success = false;
            std::vector<std::vector<Eigen::Vector3d>> vis_trajs;
            poly_traj::MinJerkOpt best_MJO;

            if (params_.use_multitopology_trajs)
            {
                std::vector<ego_planner::ConstraintPoints> trajs = poly_traj_opt_->distinctiveTrajs(segments);
                Eigen::VectorXi success = Eigen::VectorXi::Zero(trajs.size());
                poly_traj::Trajectory initTraj = initMJO.getTraj();
                int PN = initTraj.getPieceNum();
                Eigen::MatrixXd all_pos = initTraj.getPositions();
                Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
                Eigen::Matrix<double, 3, 3> headState, tailState;
                headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
                tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
                double final_cost, min_cost = 999999.0;

                for (int i = trajs.size() - 1; i >= 0; i--)
                {
                    poly_traj_opt_->setConstraintPoints(trajs[i]);
                    poly_traj_opt_->setUseMultitopologyTrajs(true);
                    if (poly_traj_opt_->optimizeTrajectory(headState, tailState,
                                                           innerPts, initTraj.getDurations(), final_cost))
                    {
                        success[i] = true;

                        if (final_cost < min_cost)
                        {
                            min_cost = final_cost;
                            best_MJO = poly_traj_opt_->getMinJerkOpt();
                            flag_success = true;
                        }

                        // visualization
                        Eigen::MatrixXd ctrl_pts_temp = poly_traj_opt_->getMinJerkOpt().getInitConstraintPoints(poly_traj_opt_->get_cps_num_per_piece_());
                        std::vector<Eigen::Vector3d> point_set;
                        for (int j = 0; j < ctrl_pts_temp.cols(); j++)
                        {
                            point_set.push_back(ctrl_pts_temp.col(j));
                        }
                        vis_trajs.push_back(point_set);
                    }
                }

                // t_opt = get_clock()->now() - t_start;

                if (trajs.size() > 1)
                {
                    RCLCPP_DEBUG(get_logger(), "\033[1;33mmulti-trajs=%zu,\033[1;0m Success:fail=%d:%ld", trajs.size(), success.sum(), success.size() - success.sum());
                }

                plan_visualizer_->displayMultiOptimalPathList(vis_trajs, 0.1); // This display will take up several milliseconds.
            }
            else
            {
                poly_traj::Trajectory initTraj = initMJO.getTraj();
                int PN = initTraj.getPieceNum();
                Eigen::MatrixXd all_pos = initTraj.getPositions();
                Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
                Eigen::Matrix<double, 3, 3> headState, tailState;
                headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
                tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
                double final_cost;
                flag_success = poly_traj_opt_->optimizeTrajectory(headState, tailState,
                                                                  innerPts, initTraj.getDurations(), final_cost);
                best_MJO = poly_traj_opt_->getMinJerkOpt();
                // t_opt = get_clock()->now() - t_start;
            }

            /*** STEP 3: Store and display results ***/
            if (enable_diagnostics_)
                updater_.force_update();

            RCLCPP_DEBUG(get_logger(), "Success=%s", flag_success ? "yes" : "no");
            if (flag_success)
            {
                // static double sum_time = 0;
                // static int count_success = 0;
                // sum_time += (t_init + t_opt).seconds();
                // count_success++;
                // RCLCPP_INFO(get_logger(), "Time:\033[42m%.3fms,\033[0m init:%.3fms, optimize:%.3fms, avg=%.3fms",
                //             (t_init + t_opt).seconds() * 1000, t_init.seconds() * 1000, t_opt.seconds() * 1000, (sum_time / count_success) * 1000);

                setLocalTrajFromOpt(best_MJO, touch_goal);
                cstr_pts = best_MJO.getInitConstraintPoints(poly_traj_opt_->get_cps_num_per_piece_());
                plan_visualizer_->displayOptimalList(cstr_pts, 0);

                continuous_failures_count_ = 0;
            }
            else
            {
                cstr_pts = poly_traj_opt_->getMinJerkOpt().getInitConstraintPoints(poly_traj_opt_->get_cps_num_per_piece_());
                plan_visualizer_->displayFailedList(cstr_pts, 0);

                continuous_failures_count_++;
            }

            return flag_success;
        }

        bool EgoPlanner::computeInitState(
            const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
            const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
            const bool flag_polyInit, const bool flag_randomPolyTraj, const double &ts,
            poly_traj::MinJerkOpt &initMJO)
        {

            static bool flag_first_call = true;

            if (flag_first_call || flag_polyInit) /*** case 1: polynomial initialization ***/
            {
                flag_first_call = false;

                /* basic params */
                Eigen::Matrix3d headState, tailState;
                Eigen::MatrixXd innerPs;
                Eigen::VectorXd piece_dur_vec;
                int piece_nums;
                constexpr double init_of_init_totaldur = 2.0;
                headState << start_pt, start_vel, start_acc;
                tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

                /* determined or random inner point */
                if (!flag_randomPolyTraj)
                {
                    if (innerPs.cols() != 0)
                    {
                        RCLCPP_ERROR(get_logger(), "innerPs.cols() != 0");
                    }

                    piece_nums = 1;
                    piece_dur_vec.resize(1);
                    piece_dur_vec(0) = init_of_init_totaldur;
                }
                else
                {
                    Eigen::Vector3d horizontal_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
                    Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizontal_dir)).normalized();
                    innerPs.resize(3, 1);
                    innerPs = (start_pt + local_target_pt) / 2 +
                              (((double)rand()) / RAND_MAX - 0.5) *
                                  (start_pt - local_target_pt).norm() *
                                  horizontal_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                              (((double)rand()) / RAND_MAX - 0.5) *
                                  (start_pt - local_target_pt).norm() *
                                  vertical_dir * 0.4 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);

                    piece_nums = 2;
                    piece_dur_vec.resize(2);
                    piece_dur_vec = Eigen::Vector2d(init_of_init_totaldur / 2, init_of_init_totaldur / 2);
                }

                /* generate the init of init trajectory */
                initMJO.reset(headState, tailState, piece_nums);
                initMJO.generate(innerPs, piece_dur_vec);
                poly_traj::Trajectory initTraj = initMJO.getTraj();

                /* generate the real init trajectory */
                piece_nums = round((headState.col(0) - tailState.col(0)).norm() / params_.poly_traj_piece_length);
                if (piece_nums < 2)
                    piece_nums = 2;
                double piece_dur = init_of_init_totaldur / (double)piece_nums;
                piece_dur_vec.resize(piece_nums);
                piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, ts);
                innerPs.resize(3, piece_nums - 1);
                int id = 0;
                double t_s = piece_dur, t_e = init_of_init_totaldur - piece_dur / 2;
                for (double t = t_s; t < t_e; t += piece_dur)
                {
                    innerPs.col(id++) = initTraj.getPos(t);
                }
                if (id != piece_nums - 1)
                {
                    RCLCPP_ERROR(get_logger(), "Should not happen! x_x");
                    return false;
                }
                initMJO.reset(headState, tailState, piece_nums);
                initMJO.generate(innerPs, piece_dur_vec);
            }
            else /*** case 2: initialize from previous optimal trajectory ***/
            {
                if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
                {
                    RCLCPP_ERROR(get_logger(), "You are initializing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
                    return false;
                }

                /* the trajectory time system is a little bit complicated... */
                double passed_t_on_lctraj = get_clock()->now().seconds() - traj_.local_traj.start_time;
                double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;
                if (t_to_lc_end < 0)
                {
                    RCLCPP_DEBUG(get_logger(), "t_to_lc_end < 0, exit and wait for another call.");
                    return false;
                }
                double t_to_lc_tgt = t_to_lc_end +
                                     (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);
                int piece_nums = ceil((start_pt - local_target_pt).norm() / params_.poly_traj_piece_length);
                if (piece_nums < 2)
                    piece_nums = 2;

                Eigen::Matrix3d headState, tailState;
                Eigen::MatrixXd innerPs(3, piece_nums - 1);
                Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);
                headState << start_pt, start_vel, start_acc;
                tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

                double t = piece_dur_vec(0);
                for (int i = 0; i < piece_nums - 1; ++i)
                {
                    if (t < t_to_lc_end)
                    {
                        innerPs.col(i) = traj_.local_traj.traj.getPos(t + passed_t_on_lctraj);
                    }
                    else if (t <= t_to_lc_tgt)
                    {
                        double glb_t = t - t_to_lc_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
                        innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
                    }
                    else
                    {
                        RCLCPP_ERROR(get_logger(), "Should not happen! x_x 0x88 t=%.2f, t_to_lc_end=%.2f, t_to_lc_tgt=%.2f", t, t_to_lc_end, t_to_lc_tgt);
                    }

                    t += piece_dur_vec(i + 1);
                }

                initMJO.reset(headState, tailState, piece_nums);
                initMJO.generate(innerPs, piece_dur_vec);
            }

            return true;
        }

        void EgoPlanner::getLocalTarget(
            const double planning_horizon, const Eigen::Vector3d &start_pt,
            const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
            Eigen::Vector3d &local_target_vel, bool &touch_goal)
        {
            double t;
            touch_goal = false;

            traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

            double t_step = planning_horizon / 20 / params_.max_vel;
            // double dist_min = 9999, dist_min_t = 0.0;
            for (t = traj_.global_traj.glb_t_of_lc_tgt;
                 t < (traj_.global_traj.global_start_time + traj_.global_traj.duration);
                 t += t_step)
            {
                Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos(t - traj_.global_traj.global_start_time);
                double dist = (pos_t - start_pt).norm();

                if (dist >= planning_horizon)
                {
                    local_target_pos = pos_t;
                    traj_.global_traj.glb_t_of_lc_tgt = t;
                    break;
                }
            }

            if ((t - traj_.global_traj.global_start_time) >= traj_.global_traj.duration - 1e-5) // Last global point
            {
                local_target_pos = global_end_pt;
                traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + traj_.global_traj.duration;
                touch_goal = true;
            }

            if ((global_end_pt - local_target_pos).norm() < (params_.max_vel * params_.max_vel) / (2 * params_.max_acc))
            {
                local_target_vel = Eigen::Vector3d::Zero();
            }
            else
            {
                local_target_vel = traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time);
            }
        }

        bool EgoPlanner::setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal)
        {
            poly_traj::Trajectory traj = opt.getTraj();
            // int PN = traj.getPieceNum();
            // for (int i = 0; i < PN; ++i)
            // {
            //     const auto &cMat = traj.getPiece(i).getCoeffMat();
            //     RCLCPP_INFO(get_logger(), "traj[%d] = \n%f, %f, %f, %f, %f, %f\n %f, %f, %f, %f, %f, %f\n %f, %f, %f, %f, %f, %f", i,
            //                 cMat(0, 0), cMat(0, 1), cMat(0, 2), cMat(0, 3), cMat(0, 4), cMat(0, 5),
            //                 cMat(1, 0), cMat(1, 1), cMat(1, 2), cMat(1, 3), cMat(1, 4), cMat(1, 5),
            //                 cMat(2, 0), cMat(2, 1), cMat(2, 2), cMat(2, 3), cMat(2, 4), cMat(2, 5));
            // }
            Eigen::MatrixXd cps = opt.getInitConstraintPoints(getCpsNumPerPiece());
            ego_planner::PtsChk_t pts_to_check;
            bool ret = poly_traj_opt_->computePointsToCheck(traj, ego_planner::ConstraintPoints::two_thirds_id(cps, touch_goal), pts_to_check);
            if (ret && pts_to_check.size() >= 1 && pts_to_check.back().size() >= 1)
            {
                traj_.setLocalTraj(traj, pts_to_check, get_clock()->now().seconds());
            }

            return ret;
        }

        void EgoPlanner::callEmergencyStop(Eigen::Vector3d stop_pos)
        {
            auto ZERO = Eigen::Vector3d::Zero();
            Eigen::Matrix<double, 3, 3> headState, tailState;
            headState << stop_pos, ZERO, ZERO;
            tailState = headState;
            poly_traj::MinJerkOpt stopMJO;
            stopMJO.reset(headState, tailState, 2);
            stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));

            setLocalTrajFromOpt(stopMJO, false);
        }

        bool EgoPlanner::checkCollision(int drone_id)
        {
            if (traj_.local_traj.start_time < 1e9) // It means my first planning has not started
                return false;
            if (traj_.swarm_traj[drone_id].drone_id != drone_id) // The trajectory is invalid
                return false;

            double my_traj_start_time = traj_.local_traj.start_time;
            double other_traj_start_time = traj_.swarm_traj[drone_id].start_time;

            double t_start = std::max(my_traj_start_time, other_traj_start_time);
            double t_end = std::min(my_traj_start_time + traj_.local_traj.duration * 2 / 3,
                                    other_traj_start_time + traj_.swarm_traj[drone_id].duration);

            for (double t = t_start; t < t_end; t += 0.03)
            {
                if ((traj_.local_traj.traj.getPos(t - my_traj_start_time) -
                     traj_.swarm_traj[drone_id].traj.getPos(t - other_traj_start_time))
                        .norm() < (getSwarmClearance() + traj_.swarm_traj[drone_id].des_clearance))
                {
                    return true;
                }
            }

            return false;
        }

        bool EgoPlanner::planGlobalTrajWaypoints(
            const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
            const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
            const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
        {
            poly_traj::MinJerkOpt globalMJO;
            Eigen::Matrix<double, 3, 3> headState, tailState;
            headState << start_pos, start_vel, start_acc;
            tailState << waypoints.back(), end_vel, end_acc;
            Eigen::MatrixXd innerPts;

            if (waypoints.size() > 1)
            {
                innerPts.resize(3, waypoints.size() - 1);
                for (int i = 0; i < static_cast<int>(waypoints.size()) - 1; ++i)
                {
                    innerPts.col(i) = waypoints[i];
                }
            }
            else
            {
                if (innerPts.size() != 0)
                {
                    RCLCPP_ERROR(get_logger(), "innerPts.size() != 0");
                }
            }

            globalMJO.reset(headState, tailState, waypoints.size());

            double des_vel = params_.max_vel / 1.5;
            Eigen::VectorXd time_vec(waypoints.size());

            for (int j = 0; j < 2; ++j)
            {
                for (size_t i = 0; i < waypoints.size(); ++i)
                {
                    time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                                           : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
                }

                globalMJO.generate(innerPts, time_vec);

                if (globalMJO.getTraj().getMaxVelRate() < params_.max_vel ||
                    start_vel.norm() > params_.max_vel ||
                    end_vel.norm() > params_.max_vel)
                {
                    break;
                }

                des_vel /= 1.5;
            }

            traj_.setGlobalTraj(globalMJO.getTraj(), get_clock()->now().seconds());

            return true;
        }

        bool EgoPlanner::callReboundReplan(bool flag_use_poly_init, bool flag_random_poly_traj)
        {
            getLocalTarget(params_.planning_horizon, start_pt_, final_goal_,
                           local_target_pt_, local_target_vel_, touch_goal_);

            bool plan_success = reboundReplan(start_pt_, start_vel_, start_acc_,
                                              local_target_pt_, local_target_vel_,
                                              (has_new_target_ || flag_use_poly_init),
                                              flag_random_poly_traj, touch_goal_);

            has_new_target_ = false;

            if (plan_success)
            {
                PolynomialTraj poly_msg;
                MincoTraj minco_msg;
                polyTraj2ROSMsg(poly_msg, minco_msg);
                poly_traj_pub_->publish(poly_msg);
                minco_traj_pub_->publish(minco_msg);
            }

            return plan_success;
        }

        bool EgoPlanner::planFromLocalTraj()
        {
            const auto &info = traj_.local_traj;
            double t_cur = get_clock()->now().seconds() - info.start_time;

            start_pt_ = info.traj.getPos(t_cur);
            start_vel_ = info.traj.getVel(t_cur);
            start_acc_ = info.traj.getAcc(t_cur);
            // RCLCPP_INFO(get_logger(), "[planFromLocalTraj] start_pt_ = %f, %f, %f", start_pt_(0), start_pt_(1), start_pt_(2));
            bool success = callReboundReplan(false, false);

            if (!success)
            {
                success = callReboundReplan(true, false);
                if (!success)
                {
                    for (int i = 0; i < params_.local_retrials; i++)
                    {
                        success = callReboundReplan(true, true);
                        if (success)
                            break;
                    }
                    if (!success)
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        bool EgoPlanner::planFromGlobalTraj()
        {
            start_pt_ = odom_pos_;
            start_vel_ = odom_vel_;
            start_acc_.setZero();
            // RCLCPP_INFO(get_logger(), "[planFromGlobalTraj] odom_pos_ = %f, %f, %f", start_pt_(0), start_pt_(1), start_pt_(2));

            bool flag_random_poly_init;
            if (repeated_state_count_ == 1)
                flag_random_poly_init = false;
            else
                flag_random_poly_init = true;

            for (int i = 0; i < params_.global_retrials; i++)
            {
                if (callReboundReplan(true, flag_random_poly_init))
                {
                    return true;
                }
            }
            return false;
        }

        bool EgoPlanner::planNextWaypoint(const Eigen::Vector3d next_wp)
        {
            bool success = false;
            std::vector<Eigen::Vector3d> one_pt_wps;
            one_pt_wps.push_back(next_wp);
            RCLCPP_DEBUG(get_logger(), "[planNextWaypoint] next_wp = %f, %f, %f", next_wp(0), next_wp(1), next_wp(2));
            success = planGlobalTrajWaypoints(
                odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
                one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

            if (success)
            {
                final_goal_ = next_wp;

                /*** display ***/
                constexpr double step_size_t = 0.1;
                int i_end = std::floor(traj_.global_traj.duration / step_size_t);
                std::vector<Eigen::Vector3d> global_traj(i_end);
                for (int i = 0; i < i_end; i++)
                {
                    global_traj[i] = traj_.global_traj.traj.getPos(i * step_size_t);
                }

                has_target_ = true;
                has_new_target_ = true;
                if (current_state_ != State::IDLE)
                {
                    is_next_waypoint_ready_ = true;
                }

                ego_planner::PlanVisualizer::DisplayData visual_data;
                visual_data.points = global_traj;
                visual_data.scale = 0.1;
                visual_data.color = Eigen::Vector4d(1, 0, 0, 1);
                visual_data.id = 0;
                plan_visualizer_->displayGlobalPathList(visual_data);
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Unable to generate global trajectory!");
            }

            return success;
        }

        void EgoPlanner::polyTraj2ROSMsg(PolynomialTraj &poly_msg, MincoTraj &MINCO_msg)
        {
            auto data = &traj_.local_traj;
            Eigen::VectorXd durs = data->traj.getDurations();
            int piece_num = data->traj.getPieceNum();
            double secs = data->start_time;
            uint64_t nsecs = static_cast<uint64_t>(secs * 1e9);
            poly_msg.drone_id = params_.drone_id;
            poly_msg.traj_id = data->traj_id;
            poly_msg.start_time = rclcpp::Time(nsecs);
            poly_msg.order = 5; // only support order = 5 now.
            poly_msg.duration.resize(piece_num);
            poly_msg.coef_x.resize(6 * piece_num);
            poly_msg.coef_y.resize(6 * piece_num);
            poly_msg.coef_z.resize(6 * piece_num);
            // RCLCPP_INFO(node_->get_logger(), "[polyTraj2ROSMsg] piece_num = %d", piece_num);

            for (int i = 0; i < piece_num; ++i)
            {
                poly_msg.duration[i] = durs(i);

                poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
                int i6 = i * 6;
                for (int j = 0; j < 6; j++)
                {
                    poly_msg.coef_x[i6 + j] = cMat(0, j);
                    poly_msg.coef_y[i6 + j] = cMat(1, j);
                    poly_msg.coef_z[i6 + j] = cMat(2, j);
                    // RCLCPP_INFO(get_logger(), "poly_msg.coef_x[%d] = %f", i6 + j, poly_msg.coef_x[i6 + j]);
                    // RCLCPP_INFO(get_logger(), "poly_msg.coef_y[%d] = %f", i6 + j, poly_msg.coef_y[i6 + j]);
                    // RCLCPP_INFO(get_logger(), "poly_msg.coef_z[%d] = %f", i6 + j, poly_msg.coef_z[i6 + j]);
                }
            }

            MINCO_msg.drone_id = params_.drone_id;
            MINCO_msg.traj_id = data->traj_id;
            MINCO_msg.start_time = rclcpp::Time(nsecs);
            MINCO_msg.order = 5; // todo, only support order = 5 now.
            MINCO_msg.duration.resize(piece_num);
            Eigen::Vector3d vec;
            vec = data->traj.getPos(0);
            MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
            vec = data->traj.getVel(0);
            MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
            vec = data->traj.getAcc(0);
            MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
            vec = data->traj.getPos(data->duration);
            MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
            vec = data->traj.getVel(data->duration);
            MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
            vec = data->traj.getAcc(data->duration);
            MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);
            MINCO_msg.inner_x.resize(piece_num - 1);
            MINCO_msg.inner_y.resize(piece_num - 1);
            MINCO_msg.inner_z.resize(piece_num - 1);
            Eigen::MatrixXd pos = data->traj.getPositions();
            for (int i = 0; i < piece_num - 1; i++)
            {
                MINCO_msg.inner_x[i] = pos(0, i + 1);
                MINCO_msg.inner_y[i] = pos(1, i + 1);
                MINCO_msg.inner_z[i] = pos(2, i + 1);
            }
            for (int i = 0; i < piece_num; i++)
                MINCO_msg.duration[i] = durs[i];
        }

        bool EgoPlanner::modifyGoalPointInObstacles()
        {
            if (!grid_map_->getInflateOccupancy(final_goal_))
            {
                return false; // Already not in collision
            }

            Eigen::Vector3d orig_goal = final_goal_;
            Eigen::Vector3d pt;
            double t_step = modify_goal_tstep_;
            double t = traj_.global_traj.duration;

            while (t > 0)
            {
                pt = traj_.global_traj.traj.getPos(t);
                if (!grid_map_->getInflateOccupancy(pt) && planNextWaypoint(pt))
                {
                    RCLCPP_DEBUG(get_logger(),
                                 "Current in-collision waypoint (%.3f, %.3f %.3f) has been modified to (%.3f, %.3f %.3f)",
                                 orig_goal(0), orig_goal(1), orig_goal(2), final_goal_(0), final_goal_(1), final_goal_(2));
                    return true;
                }
                t -= t_step;
            }

            RCLCPP_ERROR(get_logger(), "Can't find any collision-free point on global traj.");
            return false;
        }

    } // namespace path_planning
} // namespace uosm

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uosm::path_planning::EgoPlanner)