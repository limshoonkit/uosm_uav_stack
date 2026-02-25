#include "../include/trunk_seg_component.hpp"

namespace uosm
{
    namespace perception
    {
        TrunkSegmentationComponent::TrunkSegmentationComponent(const rclcpp::NodeOptions &options)
            : Node("trunk_segmentation_node", options),
              updater_(this),
              pose_received_(false)
        {
            RCLCPP_INFO(get_logger(), "********************************");
            RCLCPP_INFO(get_logger(), "   Trunk Segmentation Component ");
            RCLCPP_INFO(get_logger(), "********************************");
            RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
            RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
            RCLCPP_INFO(get_logger(), "********************************");

            // Declare and get parameters
            map_frame_ = declare_parameter<std::string>("map_frame", "map");
            odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
            base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
            seg_params_.map_filter_z_min = declare_parameter<double>("map_filter_z_min", 0.0);
            seg_params_.map_filter_z_max = declare_parameter<double>("map_filter_z_max", 2.0);
            seg_params_.max_tilt_deg = declare_parameter<double>("max_tilt_deg", 10.0);
            seg_params_.max_tilt_rad = seg_params_.max_tilt_deg * M_PI / 180.0;
            seg_params_.clustering_method = declare_parameter<int>("clustering_method", 1);
            seg_params_.circle_fit_method = declare_parameter<int>("circle_fit_method", 1);
            clustering_params_.submap_search_radius = declare_parameter<double>("submap_search_radius", 0.5);
            clustering_params_.p = declare_parameter<double>("dbscan_p", 0.1);
            clustering_params_.eps_density = declare_parameter<double>("dbscan_eps_density", 0.5);
            clustering_params_.eps_clustering = declare_parameter<double>("dbscan_eps_clustering", 0.5);
            clustering_params_.dbscan_min_pts = declare_parameter<int>("dbscan_min_pts", 10);
            clustering_params_.pcl_cluster_tolerance = declare_parameter<double>("pcl_cluster_tolerance", 0.5);
            clustering_params_.pcl_min_cluster_size = declare_parameter<int>("pcl_min_cluster_size", 10);
            clustering_params_.pcl_max_cluster_size = declare_parameter<int>("pcl_max_cluster_size", 100);
            circle_fitting_params_.min_radius = declare_parameter<double>("min_radius", 0.25);
            circle_fitting_params_.max_radius = declare_parameter<double>("max_radius", 0.45);
            circle_fitting_params_.max_center_distance = declare_parameter<double>("max_center_distance", 0.5);
            circle_fitting_params_.ransac_distance_threshold = declare_parameter<double>("ransac_distance_threshold", 0.05);
            tracking_params_.association_threshold = declare_parameter<double>("association_threshold", 0.5);
            tracking_params_.distance_threshold = declare_parameter<double>("distance_threshold", 5.0);
            tracking_params_.gating_threshold = declare_parameter<double>("gating_threshold", 5.99);
            tracking_params_.min_hit_rate_for_confirmation = declare_parameter<double>("min_hit_rate_for_confirmation", 0.3);
            tracking_params_.min_observations_for_confirmation = declare_parameter<int>("min_observations_for_confirmation", 5);
            tracking_params_.max_miss_rate_for_lost = declare_parameter<double>("max_miss_rate_for_lost", 0.7);
            tracking_params_.min_observations_for_lost = declare_parameter<int>("min_observations_for_lost", 8);
            tracking_params_.detection_window_size = declare_parameter<int>("detection_window_size", 10);
            tracking_params_.process_noise_std = declare_parameter<double>("process_noise_std", 0.1);
            tracking_params_.measurement_noise_std = declare_parameter<double>("measurement_noise_std", 0.2);
            tracking_params_.radius_tolerance = declare_parameter<double>("radius_tolerance", 0.1);
            tracking_params_.radius_penalty_weight = declare_parameter<double>("radius_penalty_weight", 1.0);
            tracking_params_.max_lost_tracks = declare_parameter<int>("max_lost_tracks", 50);
            tracking_params_.max_lost_track_age_seconds = declare_parameter<double>("max_lost_track_age_seconds", 360.0);
            tracking_params_.lost_track_association_threshold = declare_parameter<double>("lost_track_association_threshold", 1.0);
            tracking_params_.covariance_inflation_factor = declare_parameter<double>("covariance_inflation_factor", 2.0);
            enable_diagnostics_ = declare_parameter<bool>("enable_diagnostics", false);

            RCLCPP_INFO(get_logger(), " * Trunk Segmentation parameters:");
            RCLCPP_INFO(get_logger(), " *   map_filter_z_min: %.2f", seg_params_.map_filter_z_min);
            RCLCPP_INFO(get_logger(), " *   map_filter_z_max: %.2f", seg_params_.map_filter_z_max);
            RCLCPP_INFO(get_logger(), " *   max_tilt_deg: %.2f", seg_params_.max_tilt_deg);
            RCLCPP_INFO(get_logger(), " *   clustering_method: %d", seg_params_.clustering_method);
            RCLCPP_INFO(get_logger(), " *   circle_fit_method: %d", seg_params_.circle_fit_method);
            RCLCPP_INFO(get_logger(), " *   submap_search_radius: %.2f", clustering_params_.submap_search_radius);
            RCLCPP_INFO(get_logger(), " *   dbscan_p: %.2f", clustering_params_.p);
            RCLCPP_INFO(get_logger(), " *   dbscan_eps_clustering: %.2f", clustering_params_.eps_clustering);
            RCLCPP_INFO(get_logger(), " *   dbscan_eps_density: %.2f", clustering_params_.eps_density);
            RCLCPP_INFO(get_logger(), " *   dbscan_min_pts: %d", clustering_params_.dbscan_min_pts);
            RCLCPP_INFO(get_logger(), " *   pcl_cluster_tolerance: %.2f", clustering_params_.pcl_cluster_tolerance);
            RCLCPP_INFO(get_logger(), " *   pcl_min_cluster_size: %d", clustering_params_.pcl_min_cluster_size);
            RCLCPP_INFO(get_logger(), " *   pcl_max_cluster_size: %d", clustering_params_.pcl_max_cluster_size);
            RCLCPP_INFO(get_logger(), " *   min_radius: %.2f", circle_fitting_params_.min_radius);
            RCLCPP_INFO(get_logger(), " *   max_radius: %.2f", circle_fitting_params_.max_radius);
            RCLCPP_INFO(get_logger(), " *   max_center_distance: %.2f", circle_fitting_params_.max_center_distance);
            RCLCPP_INFO(get_logger(), " *   ransac_distance_threshold: %.2f", circle_fitting_params_.ransac_distance_threshold);
            RCLCPP_INFO(get_logger(), " *   association_threshold: %.2f", tracking_params_.association_threshold);
            RCLCPP_INFO(get_logger(), " *   distance_threshold: %.2f", tracking_params_.distance_threshold);
            RCLCPP_INFO(get_logger(), " *   gating_threshold: %.2f", tracking_params_.gating_threshold);
            RCLCPP_INFO(get_logger(), " *   min_hit_rate_for_confirmation: %.2f", tracking_params_.min_hit_rate_for_confirmation);
            RCLCPP_INFO(get_logger(), " *   min_observations_for_confirmation: %ld", tracking_params_.min_observations_for_confirmation);
            RCLCPP_INFO(get_logger(), " *   max_miss_rate_for_lost: %.2f", tracking_params_.max_miss_rate_for_lost);
            RCLCPP_INFO(get_logger(), " *   min_observations_for_lost: %ld", tracking_params_.min_observations_for_lost);
            RCLCPP_INFO(get_logger(), " *   detection_window_size: %ld", tracking_params_.detection_window_size);
            RCLCPP_INFO(get_logger(), " *   process_noise_std: %.2f", tracking_params_.process_noise_std);
            RCLCPP_INFO(get_logger(), " *   measurement_noise_std: %.2f", tracking_params_.measurement_noise_std);
            RCLCPP_INFO(get_logger(), " *   radius_tolerance: %.2f", tracking_params_.radius_tolerance);
            RCLCPP_INFO(get_logger(), " *   radius_penalty_weight: %.2f", tracking_params_.radius_penalty_weight);
            RCLCPP_INFO(get_logger(), " *   max_lost_tracks: %d", tracking_params_.max_lost_tracks);
            RCLCPP_INFO(get_logger(), " *   max_lost_track_age_seconds: %.2f", tracking_params_.max_lost_track_age_seconds);
            RCLCPP_INFO(get_logger(), " *   lost_track_association_threshold: %.2f", tracking_params_.lost_track_association_threshold);
            RCLCPP_INFO(get_logger(), " *   covariance_inflation_factor: %.2f", tracking_params_.covariance_inflation_factor);
            RCLCPP_INFO(get_logger(), " *   enable_diagnostics: %d", enable_diagnostics_);
            RCLCPP_INFO(get_logger(), " *****************************************");

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rclcpp::Clock::make_shared());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            clustering_ = std::make_shared<ClusteringAlgorithms>(clustering_params_);
            circle_fitting_ = std::make_shared<CircleFitting>(circle_fitting_params_);
            trunk_tracking_ = std::make_shared<TrunkTracking>(tracking_params_);

            // Create subscribers
            laser_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
                "trunk_seg/scan", 10,
                std::bind(&TrunkSegmentationComponent::laserScanCallback, this, std::placeholders::_1));

            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                "trunk_seg/odom", 10,
                std::bind(&TrunkSegmentationComponent::odometryCallback, this, std::placeholders::_1));

            // Create publishers for visualization
            cluster_marker_pub_ = create_publisher<VizMarkerArr>(
                "trunk_clusters", 10);
            circle_marker_pub_ = create_publisher<VizMarkerArr>(
                "trunk_circles", 10);
            active_track_marker_pub_ = create_publisher<VizMarkerArr>(
                "active_tracks", 10);
            lost_track_marker_pub_ = create_publisher<VizMarkerArr>(
                "lost_tracks", 10);

            // Create publishers for trunk observations
            trunk_observations_pub_ = create_publisher<uosm_uav_interface::msg::TrunkObservationArray>(
                "trunk_observations", 10);

            RCLCPP_INFO(get_logger(), "TrunkSegmentationComponent initialized successfully");

            if (enable_diagnostics_)
            {
                updater_.setHardwareID("TrunkSegmentation");
                updater_.add("Segmentation Update Time", [this](diagnostic_updater::DiagnosticStatusWrapper &stat)
                             {
                    for (const auto &pair : section_durations_) {
                            stat.add(pair.first, uosm::util::formatDuration(pair.second));
                    }
                    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Execution timing diagnostics"); });
            }
        }

        void TrunkSegmentationComponent::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            current_pose_ = msg->pose.pose;
            pose_received_ = true;
        }

        void TrunkSegmentationComponent::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            if (enable_diagnostics_)
                section_durations_.clear();

            if (shouldIgnoreScan(msg))
            {
                // Still publish existing tracks even if skipping scan
                auto tracks = trunk_tracking_->getTrackedTrunks();
                if (trunk_observations_pub_->get_subscription_count() > 0)
                    publishTrunkObservations(tracks);
                if (enable_diagnostics_)
                {
                    auto lost_tracks = trunk_tracking_->getLostTracks();

                    if (active_track_marker_pub_->get_subscription_count() > 0 && !tracks.empty())
                        active_track_marker_pub_->publish(getTrackingMarkers(tracks, "active"));
                    if (lost_track_marker_pub_->get_subscription_count() > 0 && !lost_tracks.empty())
                        lost_track_marker_pub_->publish(getTrackingMarkers(lost_tracks, "lost"));
                }
                return;
            }

            try
            {
                geometry_msgs::msg::TransformStamped laser_to_base_msg;
                try
                {
                    laser_to_base_msg = tf_buffer_->lookupTransform(
                        base_frame_,
                        msg->header.frame_id,
                        msg->header.stamp,
                        rclcpp::Duration::from_seconds(0.05));
                }
                catch (const tf2::TransformException &ex)
                {
                    std::cerr << "TF transform error: " << ex.what() << std::endl;
                    return;
                }

                const Eigen::Isometry3d T_base_laser = tf2::transformToEigen(laser_to_base_msg.transform);
                std::vector<Circle> fitted_circles;

                if (seg_params_.clustering_method == static_cast<int>(ClusteringMethod::PCL_EUCLIDEAN) &&
                    seg_params_.circle_fit_method == static_cast<int>(CircleFittingMethod::PCL_CONSENSUS))
                {
                    if (enable_diagnostics_)
                        PROFILE_SECTION("PCL Clustering & Circle Fitting Pipeline");

                    fullPCLPipeline(msg, T_base_laser, fitted_circles);
                }
                else
                {
                    if (enable_diagnostics_)
                        PROFILE_SECTION("Clustering");

                    auto points = clustering_->laserScanToPoints(*msg, T_base_laser);
                    auto clusters = clustering_->clusterPoints(points, static_cast<ClusteringMethod>(seg_params_.clustering_method));

                    if (enable_diagnostics_)
                        publishClusterMarkers(clusters, current_pose_);

                    if (enable_diagnostics_)
                        PROFILE_SECTION("Circle Fitting");

                    fitted_circles.reserve(clusters.size());
                    auto circle_fit_method = static_cast<CircleFittingMethod>(seg_params_.circle_fit_method);
                    for (const auto &cluster : clusters)
                    {
                        auto circle = circle_fitting_->fitCircle(cluster, circle_fit_method);
                        if (circle.quality_score > 0.8)
                            fitted_circles.emplace_back(circle);
                    }

                    if (enable_diagnostics_)
                        publishCircleMarkers(fitted_circles, current_pose_);
                }

                if (enable_diagnostics_)
                    PROFILE_SECTION("Trunk Tracking");

                trunk_tracking_->updateTracks(fitted_circles, current_pose_, msg->header.stamp);
                auto tracks = trunk_tracking_->getTrackedTrunks();
                if (trunk_observations_pub_->get_subscription_count() > 0)
                    publishTrunkObservations(tracks);

                if (enable_diagnostics_)
                {
                    auto lost_tracks = trunk_tracking_->getLostTracks();
                    if (active_track_marker_pub_->get_subscription_count() > 0 && !tracks.empty())
                        active_track_marker_pub_->publish(getTrackingMarkers(tracks, "active"));
                    if (lost_track_marker_pub_->get_subscription_count() > 0 && !lost_tracks.empty())
                        lost_track_marker_pub_->publish(getTrackingMarkers(lost_tracks, "lost"));

                    updater_.force_update();
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_logger(), "Exception in laserScanCallback: %s", e.what());
            }
        }

        void TrunkSegmentationComponent::publishTrunkObservations(const std::unordered_map<uint16_t, TrackedTrunk> &tracks)
        {
            uosm_uav_interface::msg::TrunkObservationArray observation_array_msg;
            observation_array_msg.header.stamp = get_clock()->now();
            observation_array_msg.header.frame_id = odom_frame_;
            observation_array_msg.observations.reserve(tracks.size());
            for (const auto &track : tracks)
            {
                uosm_uav_interface::msg::TrunkObservation observation_msg;
                observation_msg.header = observation_array_msg.header;
                observation_msg.position.x = track.second.position_map(0);
                observation_msg.position.y = track.second.position_map(1);
                observation_msg.position.z = 0.0; // ignore height
                observation_msg.landmark_id = track.first;
                observation_msg.covariance = {static_cast<float>(track.second.covariance(0, 0)),
                                              static_cast<float>(track.second.covariance(0, 1)),
                                              static_cast<float>(track.second.covariance(1, 0)),
                                              static_cast<float>(track.second.covariance(1, 1))};
                observation_array_msg.observations.emplace_back(std::move(observation_msg));
            }
            trunk_observations_pub_->publish(observation_array_msg);
        }

        void TrunkSegmentationComponent::publishClusterMarkers(const std::vector<Cluster> &clusters,
                                                               const geometry_msgs::msg::Pose &robot_pose)
        {
            if (cluster_marker_pub_->get_subscription_count() == 0)
                return;

            VizMarkerArr marker_array;
            // Clear previous markers
            clearMarkers(marker_array, "cluster_lines", odom_frame_, get_clock());
            clearMarkers(marker_array, "cluster_labels", odom_frame_, get_clock());

            // Transform setup
            const Eigen::Quaterniond robot_q(robot_pose.orientation.w, robot_pose.orientation.x,
                                             robot_pose.orientation.y, robot_pose.orientation.z);
            const Eigen::Vector3d robot_t(robot_pose.position.x, robot_pose.position.y, robot_pose.position.z);

            // Pre-allocate memory for markers (2x clusters + 2x clear markers)
            marker_array.markers.reserve(2 + 2 * clusters.size());

            for (size_t cluster_idx = 0; cluster_idx < clusters.size(); ++cluster_idx)
            {
                const auto &cluster = clusters[cluster_idx];
                if (cluster.empty())
                    continue;

                const auto color = generateRandomColor(cluster_idx);
                auto line_marker = createLineMarker(static_cast<int>(cluster_idx) + 1, 0, 0, 0,
                                                    color, 0.1, "cluster_lines",
                                                    odom_frame_, get_clock());

                line_marker.points.reserve(cluster.size());
                for (const auto &point : cluster)
                {
                    Eigen::Vector3d local_point(point.first, point.second, 0);
                    Eigen::Vector3d global_point = robot_q * local_point + robot_t;

                    geometry_msgs::msg::Point pt;
                    pt.x = global_point.x();
                    pt.y = global_point.y();
                    pt.z = global_point.z() + 0.15;
                    line_marker.points.emplace_back(std::move(pt));
                }
                marker_array.markers.push_back(std::move(line_marker));

                Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
                for (const auto &point : cluster)
                {
                    centroid += Eigen::Vector3d(point.first, point.second, 0);
                }
                centroid /= cluster.size();
                const Eigen::Vector3d global_centroid = robot_q * centroid + robot_t;

                const std::string text = "Group" + std::to_string(cluster_idx) + " (" + std::to_string(cluster.size()) + "pts)";
                auto text_marker = createTextMarker(static_cast<int>(cluster_idx) + 1,
                                                    global_centroid.x(), global_centroid.y(), global_centroid.z(),
                                                    0.0, 0.0, 0.0, 1.0, // RGBA
                                                    text, 1.0, 0.2, "cluster_labels",
                                                    odom_frame_, get_clock());
                marker_array.markers.push_back(std::move(text_marker));
            }
            cluster_marker_pub_->publish(marker_array);
        }

        void TrunkSegmentationComponent::publishCircleMarkers(const std::vector<Circle> &circles,
                                                              const geometry_msgs::msg::Pose &robot_pose)
        {
            if (circle_marker_pub_->get_subscription_count() == 0)
                return;

            VizMarkerArr marker_array;
            const std::string ns = "fitted_circles";
            clearMarkers(marker_array, ns, odom_frame_, get_clock());
            marker_array.markers.reserve(1 + circles.size());
            const Eigen::Quaterniond robot_q(robot_pose.orientation.w, robot_pose.orientation.x,
                                             robot_pose.orientation.y, robot_pose.orientation.z);
            const Eigen::Vector3d robot_t(robot_pose.position.x, robot_pose.position.y, robot_pose.position.z);

            for (size_t circle_idx = 0; circle_idx < circles.size(); ++circle_idx)
            {
                // Transform circle center from robot frame to odom frame
                Eigen::Vector3d local_point(circles[circle_idx].x, circles[circle_idx].y, 0);
                Eigen::Vector3d global_point = robot_q * local_point + robot_t;
                auto cylinder_marker = createCylinderMarker(circle_idx + 1, global_point.x(), global_point.y(), global_point.z(),
                                                            circles[circle_idx].r, 0.1,
                                                            0.8, 0.2, 0.4, 0.6, // RGBA
                                                            ns, odom_frame_, get_clock());
                marker_array.markers.push_back(std::move(cylinder_marker));
            }
            circle_marker_pub_->publish(marker_array);
        }

        VizMarkerArr TrunkSegmentationComponent::getTrackingMarkers(const std::unordered_map<uint16_t, TrackedTrunk> &tracks,
                                                                    const std::string ns)
        {
            VizMarkerArr marker_array;
            const std::string trunk_ns = ns + "_trunks";
            const std::string label_ns = ns + "_labels";
            const std::string cov_ns = ns + "_covariance";

            clearMarkers(marker_array, trunk_ns, odom_frame_, get_clock());
            clearMarkers(marker_array, label_ns, odom_frame_, get_clock());
            clearMarkers(marker_array, cov_ns, odom_frame_, get_clock());

            marker_array.markers.reserve(3 * tracks.size());

            for (const auto &[id, track] : tracks)
            {
                std_msgs::msg::ColorRGBA color;
                std::string state_str;
                color.a = 0.5;

                switch (track.state)
                {
                case TrackState::CONFIRMED:
                    color.r = 0.0;
                    color.g = 0.9;
                    color.b = 0.0;
                    state_str = "CONFIRMED";
                    break;
                case TrackState::TENTATIVE:
                    color.r = 0.9;
                    color.g = 0.9;
                    color.b = 0.0;
                    state_str = "TENTATIVE";
                    break;
                case TrackState::LOST:
                    color.r = 0.9;
                    color.g = 0.0;
                    color.b = 0.0;
                    state_str = "LOST";
                    break;
                }

                auto cylinder_marker = createCylinderMarker(track.id,
                                                            track.position_map.x(), track.position_map.y(), seg_params_.map_filter_z_max / 2.0,
                                                            track.getAverageRadius(), seg_params_.map_filter_z_max,
                                                            color.r, color.g, color.b, color.a,
                                                            trunk_ns, odom_frame_, get_clock());
                marker_array.markers.push_back(std::move(cylinder_marker));

                std::stringstream ss;
                ss << "ID: " << track.id << " [" << state_str << "]\n"
                   << "Quality:" << track.calculateQualityScore()
                   << "\nObs: " << track.total_observations
                   << "\n[x: " << std::fixed << std::setprecision(2) << track.position_map.x()
                   << ", y: " << std::fixed << std::setprecision(2) << track.position_map.y() << "]"
                   << "\nR: " << std::fixed << std::setprecision(2) << track.getAverageRadius() << "m";
                auto text_marker = createTextMarker(track.id,
                                                    track.position_map.x(), track.position_map.y(), seg_params_.map_filter_z_max + 0.2,
                                                    0.0, 0.0, 0.0, 1.0, // RGBA
                                                    ss.str(), 0.2, 0.2,
                                                    label_ns, odom_frame_, get_clock());
                marker_array.markers.push_back(std::move(text_marker));

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(track.covariance);
                if (eigensolver.info() == Eigen::Success)
                {
                    const auto &eigenvalues = eigensolver.eigenvalues();
                    const auto &eigenvectors = eigensolver.eigenvectors();

                    // The angle of the major axis
                    double yaw = std::atan2(eigenvectors.col(1).y(), eigenvectors.col(1).x());
                    tf2::Quaternion q;
                    q.setRPY(0, 0, yaw);
                    auto orientation = tf2::toMsg(q);

                    auto cov_marker = createSphereMarker(track.id,
                                                         track.position_map.x(), track.position_map.y(), 0.05,
                                                         2 * std::sqrt(eigenvalues[0]), 2 * std::sqrt(eigenvalues[1]), 0.01,
                                                         orientation,
                                                         0.3, 0.3, 0.3, 0.8, // RGBA
                                                         cov_ns, odom_frame_, get_clock());
                    marker_array.markers.push_back(std::move(cov_marker));
                }
            }
            return marker_array;
        }

        bool TrunkSegmentationComponent::shouldIgnoreScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            if (!pose_received_ || msg->ranges.empty())
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "No pose or scan received yet, skipping scan processing");
                return true;
            }

            if (current_pose_.position.z < seg_params_.map_filter_z_min ||
                current_pose_.position.z > seg_params_.map_filter_z_max)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Robot height is outside the map height range [%.2f, %.2f], skipping scan processing",
                                     seg_params_.map_filter_z_min, seg_params_.map_filter_z_max);
                return true;
            }

            tf2::Quaternion q(current_pose_.orientation.x, current_pose_.orientation.y,
                              current_pose_.orientation.z, current_pose_.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            if (std::abs(roll) > seg_params_.max_tilt_rad || std::abs(pitch) > seg_params_.max_tilt_rad)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Robot tilt exceeds %.1f degrees, skipping scan processing",
                                     seg_params_.max_tilt_deg);
                return true;
            }
            return false;
        }

        void TrunkSegmentationComponent::fullPCLPipeline(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                                                         const Eigen::Isometry3d &transform,
                                                         std::vector<Circle> &fitted_circles)
        {
            std::vector<Eigen::Vector2d> precomputed_unit_vectors;
            if (precomputed_unit_vectors.size() != msg->ranges.size())
            {
                precomputed_unit_vectors.resize(msg->ranges.size());
                const double angle_min = static_cast<double>(msg->angle_min);
                const double angle_increment = static_cast<double>(msg->angle_increment);
                const Eigen::Matrix3d R = transform.rotation();
                for (size_t i = 0; i < msg->ranges.size(); ++i)
                {
                    const double angle = angle_min + i * angle_increment;
                    const double cos_angle = std::cos(angle);
                    const double sin_angle = std::sin(angle);
                    precomputed_unit_vectors[i] = {
                        R(0, 0) * cos_angle + R(0, 1) * sin_angle,
                        R(1, 0) * cos_angle + R(1, 1) * sin_angle};
                }
            }

            const Eigen::Vector3d translation = transform.translation();
            const double tx = translation.x();
            const double ty = translation.y();
            const float range_thresh_min = msg->range_min + 0.5f;
            const float max_range = clustering_params_.submap_search_radius;
            const float *ranges_ptr = msg->ranges.data();
            const size_t ranges_size = msg->ranges.size();

            // Count valid points for reservation
            size_t valid_count = 0;
            for (size_t i = 0; i < ranges_size; ++i)
            {
                const float range = ranges_ptr[i];
                if (range >= range_thresh_min && range <= max_range)
                {
                    ++valid_count;
                }
            }

            if (valid_count == 0)
                return;

            // Create PCL point cloud directly from laser scan
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            cloud->points.reserve(valid_count);

            for (size_t i = 0; i < ranges_size; ++i)
            {
                const float range = ranges_ptr[i];
                if (range >= range_thresh_min && range <= max_range)
                {
                    const auto &unit_vec = precomputed_unit_vectors[i];
                    cloud->points.emplace_back(
                        static_cast<float>(range * unit_vec.x() + tx),
                        static_cast<float>(range * unit_vec.y() + ty),
                        0.0f);
                }
            }

            // Create KdTree for search
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
            tree->setInputCloud(cloud);

            // Perform Euclidean clustering
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(clustering_params_.pcl_cluster_tolerance);
            ec.setMinClusterSize(clustering_params_.pcl_min_cluster_size);
            ec.setMaxClusterSize(clustering_params_.pcl_max_cluster_size);
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud);
            ec.extract(cluster_indices);

            // Direct circle fitting from PCL clusters
            fitted_circles.clear();
            fitted_circles.reserve(cluster_indices.size());
            for (const auto &indices : cluster_indices)
            {
                if (indices.indices.empty())
                    continue;

                // Create a sub-cloud for this cluster
                pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                cluster_cloud->points.reserve(indices.indices.size());

                for (const auto &idx : indices.indices)
                {
                    cluster_cloud->points.push_back(cloud->points[idx]);
                }

                // Fit circle directly to PCL cluster
                pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr model =
                    std::make_shared<pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>>(cluster_cloud);
                model->setRadiusLimits(circle_fitting_params_.min_radius, circle_fitting_params_.max_radius);
                pcl::RandomSampleConsensus<pcl::PointXYZ> sac(model, circle_fitting_params_.ransac_distance_threshold);

                if (sac.computeModel())
                {
                    Eigen::VectorXf coefficients;
                    sac.getModelCoefficients(coefficients);
                    if (coefficients.size() == 3 && coefficients[2] >= circle_fitting_params_.min_radius &&
                        coefficients[2] <= circle_fitting_params_.max_radius)
                    {
                        Circle circle;
                        circle.x = coefficients[0];
                        circle.y = coefficients[1];
                        circle.r = coefficients[2];
                        fitted_circles.emplace_back(circle);
                    }
                }

                // Only convert to Cluster format if needed for publishing
                std::vector<Cluster> clusters;
                if (enable_diagnostics_)
                {
                    Cluster cluster;
                    cluster.reserve(indices.indices.size());
                    for (const auto &idx : indices.indices)
                    {
                        const auto &point = cloud->points[idx];
                        cluster.emplace_back(static_cast<double>(point.x), static_cast<double>(point.y));
                    }
                    clusters.push_back(std::move(cluster));
                    publishClusterMarkers(clusters, current_pose_);
                    publishCircleMarkers(fitted_circles, current_pose_);
                }
            }
        }

    } // namespace perception
} // namespace uosm
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uosm::perception::TrunkSegmentationComponent)