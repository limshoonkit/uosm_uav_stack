#include "../include/map_alignment.hpp"

namespace uosm
{
    namespace perception
    {
        MapAlignmentComponent::MapAlignmentComponent(const rclcpp::NodeOptions &options)
            : Node("map_alignment_node", options), is_alignment_done_(false)
        {
            RCLCPP_INFO(get_logger(), "********************************");
            RCLCPP_INFO(get_logger(), "   Map Alignment Component ");
            RCLCPP_INFO(get_logger(), "********************************");
            RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
            RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
            RCLCPP_INFO(get_logger(), "********************************");

            // Declare parameters
            pcd_file_path_ = declare_parameter<std::string>("pcd_file_path", "");
            map_frame_ = declare_parameter<std::string>("map_frame", "map");
            odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");

            params_.solve_for_scale = declare_parameter<bool>("solve_for_scale", false);
            params_.max_initial_guess_xy_deviation = declare_parameter<double>("max_initial_guess_xy_deviation", 2.0);
            params_.max_scale_deviation = declare_parameter<double>("max_scale_deviation", 0.2);
            params_.association_radius = declare_parameter<double>("association_radius", 1.0);
            params_.min_correspondences = declare_parameter<int>("min_correspondences", 3);
            params_.max_iterations = declare_parameter<int>("max_iterations", 20);
            params_.convergence_threshold = declare_parameter<double>("convergence_threshold", 0.001);
            params_.voxel_leaf_size = declare_parameter<double>("voxel_leaf_size", 0.0);
            params_.initial_yaw_rad = declare_parameter<double>("initial_yaw_rad", 0.0);
            params_.starting_point_x = declare_parameter<double>("starting_point_x", 0.0);
            params_.starting_point_y = declare_parameter<double>("starting_point_y", 0.0);
            params_.max_alignment_attempts = declare_parameter<int>("max_alignment_attempts", 3);
            container_name_ = declare_parameter<std::string>("container_name", "autonomy_stack_container");

            RCLCPP_INFO(get_logger(), " * Map Alignment parameters:");
            RCLCPP_INFO(get_logger(), " *   solve_for_scale: %s", params_.solve_for_scale ? "true" : "false");
            RCLCPP_INFO(get_logger(), " *   max_initial_guess_xy_deviation: %f", params_.max_initial_guess_xy_deviation);
            RCLCPP_INFO(get_logger(), " *   max_scale_deviation: %f", params_.max_scale_deviation);
            RCLCPP_INFO(get_logger(), " *   association_radius: %f", params_.association_radius);
            RCLCPP_INFO(get_logger(), " *   min_correspondences: %d", params_.min_correspondences);
            RCLCPP_INFO(get_logger(), " *   max_iterations: %d", params_.max_iterations);
            RCLCPP_INFO(get_logger(), " *   convergence_threshold: %f", params_.convergence_threshold);
            RCLCPP_INFO(get_logger(), " *   voxel_leaf_size: %f", params_.voxel_leaf_size);
            RCLCPP_INFO(get_logger(), " *   initial_yaw_rad: %f", params_.initial_yaw_rad);
            RCLCPP_INFO(get_logger(), " *   starting_point_x: %f", params_.starting_point_x);
            RCLCPP_INFO(get_logger(), " *   starting_point_y: %f", params_.starting_point_y);
            RCLCPP_INFO(get_logger(), " *   max_alignment_attempts: %d", params_.max_alignment_attempts);
            RCLCPP_INFO(get_logger(), " *****************************************");

            // Initialize state
            map_landmarks_pcl_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            map_kdtree_ = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();

            T_map_odom_ = Eigen::Matrix4f::Identity();
            // Apply initial yaw rotation
            if (std::abs(params_.initial_yaw_rad) > 1e-6)
            {
                float cos_yaw = std::cos(params_.initial_yaw_rad);
                float sin_yaw = std::sin(params_.initial_yaw_rad);

                Eigen::Matrix3f rotation;
                rotation << cos_yaw, -sin_yaw, 0.0f,
                    sin_yaw, cos_yaw, 0.0f,
                    0.0f, 0.0f, 1.0f;

                T_map_odom_.block<3, 3>(0, 0) = rotation;
            }

            // Setup ROS interfaces
            landmark_sub_ = create_subscription<uosm_uav_interface::msg::TrunkObservationArray>(
                "observed_landmarks", 10,
                std::bind(&MapAlignmentComponent::observationCallback, this, std::placeholders::_1));

            alignment_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("map_alignment_markers", 10);
            alignment_status_pub_ = create_publisher<std_msgs::msg::Bool>("alignment_done", rclcpp::QoS(1));
            alignment_transform_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(
                "alignment_transform", rclcpp::QoS(1));

            // Service clients for container lifecycle management
            list_nodes_client_ = create_client<composition_interfaces::srv::ListNodes>(
                "/" + container_name_ + "/_container/list_nodes");
            unload_node_client_ = create_client<composition_interfaces::srv::UnloadNode>(
                "/" + container_name_ + "/_container/unload_node");
            
            alignment_status_srv_ = create_service<std_srvs::srv::Trigger>(
                "get_alignment_status",
                std::bind(&MapAlignmentComponent::handleAlignmentStatusRequest, this,
                          std::placeholders::_1, std::placeholders::_2));

            // Load map and start alignment process
            if (loadMap())
            {
                alignment_timer_ = create_wall_timer(
                    std::chrono::seconds(1),
                    std::bind(&MapAlignmentComponent::attemptAlignment, this));
            }

            // Publish alignment status periodically at 1Hz
            status_pub_timer_ = create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&MapAlignmentComponent::publishAlignmentStatus, this));

            RCLCPP_INFO(get_logger(), "MapAlignmentComponent initialized successfully");
        }

        bool MapAlignmentComponent::loadMap()
        {
            if (pcd_file_path_.empty())
            {
                RCLCPP_FATAL(get_logger(), "Parameter 'pcd_file_path' is not set. Cannot load map.");
                return false;
            }

            auto raw_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_, *raw_cloud) == -1)
            {
                RCLCPP_FATAL(get_logger(), "Couldn't read file %s", pcd_file_path_.c_str());
                return false;
            }

            RCLCPP_INFO(get_logger(), "Loaded %zu points from %s", raw_cloud->size(), pcd_file_path_.c_str());
            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(*raw_cloud, min_pt, max_pt);
            RCLCPP_INFO(this->get_logger(), "Original bounds: X[%.2f,%.2f] Y[%.2f,%.2f] Z[%.2f,%.2f]",
                        min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z);

            auto working_cloud = raw_cloud;
            if (params_.voxel_leaf_size > 0.0)
            {
                pcl::VoxelGrid<pcl::PointXYZ> vg;
                vg.setInputCloud(raw_cloud);
                vg.setLeafSize(params_.voxel_leaf_size, params_.voxel_leaf_size, params_.voxel_leaf_size);
                vg.filter(*working_cloud);
                RCLCPP_INFO(get_logger(), "Downsampled to %zu points", working_cloud->size());
                pcl::getMinMax3D(*working_cloud, min_pt, max_pt);
            }

            // Calculate bounding box for centering
            Eigen::Vector3f translation_offset = Eigen::Vector3f::Zero();
            translation_offset.x() = -(min_pt.x + max_pt.x) * 0.5f;
            translation_offset.y() = -(min_pt.y + max_pt.y) * 0.5f;
            translation_offset.z() = 0.0f;

            RCLCPP_INFO(this->get_logger(), "Translation offset: [%.3f,%.3f,%.3f]",
                        translation_offset.x(), translation_offset.y(), translation_offset.z());

            // Apply translation and bounds filtering
            map_landmarks_pcl_->clear();
            map_landmarks_pcl_->reserve(working_cloud->size());

            for (const auto &pt : working_cloud->points)
            {
                pcl::PointXYZ recentered_pt;
                recentered_pt.x = pt.x + translation_offset.x() - static_cast<float>(params_.starting_point_x);
                recentered_pt.y = pt.y + translation_offset.y() - static_cast<float>(params_.starting_point_y);
                recentered_pt.z = pt.z + translation_offset.z();
                map_landmarks_pcl_->push_back(recentered_pt);
            }

            map_kdtree_->setInputCloud(map_landmarks_pcl_);

            pcl::PointXYZ processed_min_pt, processed_max_pt;
            pcl::getMinMax3D(*map_landmarks_pcl_, processed_min_pt, processed_max_pt);
            RCLCPP_INFO(get_logger(), "Processed map bounds: X[%.2f,%.2f] Y[%.2f,%.2f] Z[%.2f,%.2f]",
                        processed_min_pt.x, processed_max_pt.x, processed_min_pt.y, processed_max_pt.y, processed_min_pt.z, processed_max_pt.z);
            return true;
        }

        void MapAlignmentComponent::observationCallback(
            const uosm_uav_interface::msg::TrunkObservationArray::SharedPtr msg)
        {
            if (is_alignment_done_)
            {
                landmark_sub_.reset();
                return;
            }

            for (const auto &obs : msg->observations)
            {
                observed_landmarks_[obs.landmark_id] = obs;
            }
        }

        void MapAlignmentComponent::attemptAlignment()
        {
            if (is_alignment_done_ || observed_landmarks_.size() < static_cast<size_t>(params_.min_correspondences))
            {
                RCLCPP_DEBUG(get_logger(), "Skipping alignment: done=%s, landmarks=%zu, min_required=%d",
                             is_alignment_done_ ? "true" : "false",
                             observed_landmarks_.size(),
                             params_.min_correspondences);
                return;
            }

            ++alignment_attempt_count_;
            RCLCPP_INFO(get_logger(), "Attempting alignment (%d/%d) with %zu unique landmarks",
                        alignment_attempt_count_, params_.max_alignment_attempts,
                        observed_landmarks_.size());

            // Debug output for initial transform
            RCLCPP_DEBUG(get_logger(), "Initial T_map_odom_:");
            for (int i = 0; i < 4; ++i)
            {
                RCLCPP_DEBUG(get_logger(), "[%.3f, %.3f, %.3f, %.3f]",
                             T_map_odom_(i, 0), T_map_odom_(i, 1), T_map_odom_(i, 2), T_map_odom_(i, 3));
            }

            std::vector<Eigen::Vector3d> odom_points;
            std::vector<Eigen::Matrix2d> covariances_inv;
            prepareObservationData(odom_points, covariances_inv);

            // Debug observation data
            RCLCPP_DEBUG(get_logger(), "Observation points in odom frame:");
            for (size_t i = 0; i < odom_points.size() && i < 5; ++i)
            {
                RCLCPP_DEBUG(get_logger(), "  Point %zu: [%.3f, %.3f, %.3f]",
                             i, odom_points[i].x(), odom_points[i].y(), odom_points[i].z());
            }

            bool solver_succeeded = false;
            Eigen::Matrix4f final_transform = performIterativeAlignment(
                odom_points, covariances_inv, solver_succeeded);

            // Debug final transform
            RCLCPP_DEBUG(get_logger(), "Final transform result:");
            for (int i = 0; i < 4; ++i)
            {
                RCLCPP_DEBUG(get_logger(), "[%.3f, %.3f, %.3f, %.3f]",
                             final_transform(i, 0), final_transform(i, 1), final_transform(i, 2), final_transform(i, 3));
            }

            if (!solver_succeeded)
            {
                RCLCPP_WARN(get_logger(),
                            "Alignment solver could not run (insufficient correspondences). "
                            "Attempt %d/%d failed.",
                            alignment_attempt_count_, params_.max_alignment_attempts);
            }
            else if (validateAlignment(final_transform))
            {
                finalizeAlignment(final_transform, odom_points);
                return;
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Alignment validation failed on attempt %d/%d",
                             alignment_attempt_count_, params_.max_alignment_attempts);
            }

            // Check if we've exhausted all attempts — full cleanup
            if (alignment_attempt_count_ >= params_.max_alignment_attempts)
            {
                RCLCPP_FATAL(get_logger(),
                             "ALIGNMENT FAILED after %d attempts — killing component. "
                             "Flight controller alignment timeout will trigger landing.",
                             alignment_attempt_count_);

                // Full cleanup: cancel all timers, destroy subscription, unload from container
                alignment_timer_->cancel();
                status_pub_timer_->cancel();
                landmark_sub_.reset();
                unloadPerceptionComponents();
            }
        }

        void MapAlignmentComponent::prepareObservationData(
            std::vector<Eigen::Vector3d> &odom_points,
            std::vector<Eigen::Matrix2d> &covariances_inv)
        {
            odom_points.reserve(observed_landmarks_.size());
            covariances_inv.reserve(observed_landmarks_.size());

            for (const auto &[id, obs] : observed_landmarks_)
            {
                odom_points.emplace_back(obs.position.x, obs.position.y, 0.0);

                Eigen::Matrix2d cov;
                cov << obs.covariance[0], obs.covariance[1],
                    obs.covariance[2], obs.covariance[3];

                // Add regularization and invert for weighting
                covariances_inv.emplace_back((cov + Eigen::Matrix2d::Identity() * 1e-6).inverse());
            }
        }

        Eigen::Matrix4f MapAlignmentComponent::performIterativeAlignment(
            const std::vector<Eigen::Vector3d> &odom_points,
            const std::vector<Eigen::Matrix2d> &covariances_inv,
            bool &succeeded)
        {
            succeeded = false;
            Eigen::Matrix4f current_transform = T_map_odom_;

            for (int iter = 0; iter < params_.max_iterations; ++iter)
            {
                // Find correspondences
                auto correspondences = findCorrespondences(odom_points, current_transform);

                if (correspondences.size() < static_cast<size_t>(params_.min_correspondences))
                {
                    RCLCPP_WARN(get_logger(), "Insufficient correspondences (%zu) in iteration %d",
                                correspondences.size(), iter);
                    break;
                }

                // Solve for optimal transform
                Eigen::Matrix4f delta_transform = solveWeightedAlignment(correspondences, covariances_inv);

                // Update and check convergence
                Eigen::Matrix4f new_transform = delta_transform.inverse();
                float change = (new_transform - current_transform).norm();
                current_transform = new_transform;
                succeeded = true; // At least one iteration completed successfully

                if (change < params_.convergence_threshold)
                {
                    RCLCPP_INFO(get_logger(), "Alignment converged after %d iterations", iter + 1);
                    break;
                }
            }

            return current_transform;
        }

        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        MapAlignmentComponent::findCorrespondences(
            const std::vector<Eigen::Vector3d> &odom_points,
            const Eigen::Matrix4f &transform)
        {
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> correspondences;
            correspondences.reserve(odom_points.size());

            const Eigen::Matrix4f inverse_transform = transform.inverse();

            RCLCPP_DEBUG(get_logger(), "Finding correspondences with %zu odom points", odom_points.size());
            RCLCPP_DEBUG(get_logger(), "Association radius: %.3f", params_.association_radius);

            for (size_t idx = 0; idx < odom_points.size(); ++idx)
            {
                const auto &odom_pt = odom_points[idx];
                const Eigen::Vector4f odom_pt_h(odom_pt.x(), odom_pt.y(), 0.0, 1.0);
                const Eigen::Vector4f map_pt_h = inverse_transform * odom_pt_h;

                const pcl::PointXYZ search_point(map_pt_h.x(), map_pt_h.y(), map_pt_h.z());
                std::vector<int> indices(1);
                std::vector<float> distances(1);

                if (map_kdtree_->nearestKSearch(search_point, 1, indices, distances) > 0)
                {
                    float distance = std::sqrt(distances[0]);
                    RCLCPP_DEBUG(get_logger(), "Point %zu: odom[%.3f,%.3f] -> map_search[%.3f,%.3f] -> nearest_dist=%.3f",
                                 idx, odom_pt.x(), odom_pt.y(), map_pt_h.x(), map_pt_h.y(), distance);

                    if (distance < params_.association_radius)
                    {
                        const auto &map_pt = map_landmarks_pcl_->points[indices[0]];
                        correspondences.emplace_back(
                            Eigen::Vector3d(map_pt.x, map_pt.y, 0.0),
                            odom_pt);
                        RCLCPP_DEBUG(get_logger(), "  -> Correspondence added: map[%.3f,%.3f] <-> odom[%.3f,%.3f]",
                                     map_pt.x, map_pt.y, odom_pt.x(), odom_pt.y());
                    }
                    else
                    {
                        RCLCPP_WARN(get_logger(), "  -> Distance %.3f exceeds threshold %.3f",
                                    distance, params_.association_radius);
                    }
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Point %zu: No nearest neighbor found", idx);
                }
            }

            RCLCPP_DEBUG(get_logger(), "Found %zu correspondences from %zu odom points",
                         correspondences.size(), odom_points.size());
            return correspondences;
        }

        Eigen::Matrix4f MapAlignmentComponent::solveWeightedAlignment(
            const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &correspondences,
            const std::vector<Eigen::Matrix2d> &covariances_inv)
        {
            // Calculate weighted centroids
            Eigen::Vector3d map_centroid = Eigen::Vector3d::Zero();
            Eigen::Vector3d odom_centroid = Eigen::Vector3d::Zero();
            double total_weight = 0.0;

            for (size_t i = 0; i < correspondences.size(); ++i)
            {
                const double weight = covariances_inv[i].trace();
                map_centroid += weight * correspondences[i].first;
                odom_centroid += weight * correspondences[i].second;
                total_weight += weight;
            }

            map_centroid /= total_weight;
            odom_centroid /= total_weight;

            RCLCPP_DEBUG(get_logger(), "Weighted centroids: map[%.3f,%.3f], odom[%.3f,%.3f]",
                         map_centroid.x(), map_centroid.y(), odom_centroid.x(), odom_centroid.y());

            // Calculate scale first (if enabled) before rotation estimation
            double scale = 1.0;
            if (params_.solve_for_scale)
            {
                // Use the fixed scale calculation
                double numerator = 0.0;
                double denominator = 0.0;

                for (size_t i = 0; i < correspondences.size(); ++i)
                {
                    const double weight = covariances_inv[i].trace();
                    const Eigen::Vector3d map_diff = correspondences[i].first - map_centroid;
                    const Eigen::Vector3d odom_diff = correspondences[i].second - odom_centroid;

                    numerator += weight * (map_diff.head<2>().squaredNorm());
                    denominator += weight * (odom_diff.head<2>().squaredNorm());
                }

                if (denominator > 1e-9)
                {
                    scale = std::sqrt(numerator / denominator);

                    // Clamp to prevent numerical issues
                    const double min_scale = 1.0 - params_.max_scale_deviation;
                    const double max_scale = 1.0 + params_.max_scale_deviation;
                    scale = std::max(min_scale, std::min(max_scale, scale));
                }

                RCLCPP_DEBUG(get_logger(), "Calculated scale: %.4f", scale);
            }

            // Calculate weighted covariance matrix H with scale consideration
            Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
            for (size_t i = 0; i < correspondences.size(); ++i)
            {
                const double weight = covariances_inv[i].trace();
                const Eigen::Vector3d odom_centered = correspondences[i].second - odom_centroid;
                const Eigen::Vector3d map_centered = correspondences[i].first - map_centroid;

                // Scale the odom points when building the cross-covariance
                H += weight * (scale * odom_centered) * map_centered.transpose();
            }

            // SVD-based rotation estimation
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

            // Handle reflection case
            if (R.determinant() < 0)
            {
                Eigen::Matrix3d V_prime = svd.matrixV();
                V_prime.col(2) *= -1;
                R = V_prime * svd.matrixU().transpose();
            }

            // Calculate translation: t = map_centroid - scale * R * odom_centroid
            const Eigen::Vector3d t = map_centroid - scale * R * odom_centroid;

            RCLCPP_DEBUG(get_logger(), "Transform: scale=%.4f, translation=[%.3f,%.3f,%.3f]",
                         scale, t.x(), t.y(), t.z());

            // Construct transformation matrix
            Eigen::Matrix4f delta_transform = Eigen::Matrix4f::Identity();
            delta_transform.block<3, 3>(0, 0) = (scale * R).cast<float>();
            delta_transform.block<3, 1>(0, 3) = t.cast<float>();

            return delta_transform;
        }

        bool MapAlignmentComponent::validateAlignment(const Eigen::Matrix4f &transform)
        {
            const Eigen::Vector3f translation = transform.block<3, 1>(0, 3);
            const double deviation = std::hypot(translation.x(), translation.y());

            RCLCPP_DEBUG(get_logger(), "Validation - Translation: [%.3f, %.3f, %.3f], Deviation: %.3f",
                         translation.x(), translation.y(), translation.z(), deviation);

            if (deviation > params_.max_initial_guess_xy_deviation)
            {
                RCLCPP_ERROR(get_logger(),
                             "Alignment failed: Solution deviated by %.2f meters (threshold: %.2f)",
                             deviation, params_.max_initial_guess_xy_deviation);
                return false;
            }

            if (params_.solve_for_scale)
            {
                const Eigen::Matrix3f rot = transform.block<3, 3>(0, 0);
                const double scale = std::cbrt(rot.determinant()); // Fixed calculation
                RCLCPP_DEBUG(get_logger(), "Scale factor: %.4f", scale);

                if (scale < 1.0 - params_.max_scale_deviation || scale > 1.0 + params_.max_scale_deviation)
                {
                    RCLCPP_ERROR(get_logger(),
                                 "Alignment failed: Scale factor %.2f deviates beyond threshold [%.2f, %.2f]",
                                 scale, 1.0 - params_.max_scale_deviation, 1.0 + params_.max_scale_deviation);
                    return false;
                }
            }

            RCLCPP_INFO(get_logger(), "Alignment validation passed");
            return true;
        }

        void MapAlignmentComponent::finalizeAlignment(
            const Eigen::Matrix4f &transform,
            const std::vector<Eigen::Vector3d> &odom_points)
        {
            T_map_odom_ = transform.inverse();
            is_alignment_done_ = true;

            // Extract position, rotation, and scale
            Eigen::Vector3f pos = T_map_odom_.block<3, 1>(0, 3);
            Eigen::Matrix3f rot = T_map_odom_.block<3, 3>(0, 0);
            float scale = std::cbrt(rot.determinant());
            Eigen::Quaternionf quat(rot / scale);

            RCLCPP_INFO(get_logger(), "Alignment T_map_odom completed successfully!");
            RCLCPP_INFO(get_logger(), "Pos offset: [%.3f, %.3f, %.3f]", pos.x(), pos.y(), pos.z());
            RCLCPP_INFO(get_logger(), "Rotation (quaternion): [x=%.4f, y=%.4f, z=%.4f, w=%.4f]", quat.x(), quat.y(), quat.z(), quat.w());
            RCLCPP_INFO(get_logger(), "Scale factor: %.4f", scale);

            // Publish final static TF (map -> odom) and alignment transform topic
            publishTransform();

            // Publish alignment_done = true
            std_msgs::msg::Bool done_msg;
            done_msg.data = true;
            alignment_status_pub_->publish(done_msg);

            // Cancel timers — nothing left to do
            alignment_timer_->cancel();
            status_pub_timer_->cancel();
            landmark_sub_.reset();

            // Publish visualization markers
            publishAlignmentVisualization(odom_points);

            // Unload trunk segmentation from the container (no longer needed)
            unloadPerceptionComponents();
        }

        void MapAlignmentComponent::publishTransform()
        {
            const Eigen::Matrix3f R = T_map_odom_.block<3, 3>(0, 0);
            const Eigen::Vector3f t = T_map_odom_.block<3, 1>(0, 3);
            const Eigen::Quaternionf q(R);

            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = get_clock()->now();
            transform_stamped.header.frame_id = map_frame_;
            transform_stamped.child_frame_id = odom_frame_;
            transform_stamped.transform.translation.x = t.x();
            transform_stamped.transform.translation.y = t.y();
            transform_stamped.transform.translation.z = t.z();
            transform_stamped.transform.rotation.x = q.x();
            transform_stamped.transform.rotation.y = q.y();
            transform_stamped.transform.rotation.z = q.z();
            transform_stamped.transform.rotation.w = q.w();

            // Static TF — valid for all time, no need to re-publish
            static tf2_ros::StaticTransformBroadcaster static_broadcaster(*this);
            static_broadcaster.sendTransform(transform_stamped);

            // Also publish on the alignment_transform topic for downstream nodes
            alignment_transform_pub_->publish(transform_stamped);

            RCLCPP_INFO(get_logger(), "Published static TF and alignment transform: '%s' -> '%s'",
                        map_frame_.c_str(), odom_frame_.c_str());
        }

        void MapAlignmentComponent::publishAlignmentVisualization(
            const std::vector<Eigen::Vector3d> &odom_points)
        {
            if (alignment_marker_pub_->get_subscription_count() == 0)
            {
                return;
            }

            visualization_msgs::msg::MarkerArray markers;
            visualization_msgs::msg::Marker line_list;

            line_list.header.frame_id = map_frame_;
            line_list.header.stamp = get_clock()->now();
            line_list.ns = "correspondences";
            line_list.action = visualization_msgs::msg::Marker::ADD;
            line_list.pose.orientation.w = 1.0;
            line_list.id = 0;
            line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
            line_list.scale.x = 0.05;
            line_list.color.g = 1.0;
            line_list.color.a = 0.5;

            // Transform odom points to map frame for visualization
            const Eigen::Matrix4f transform_inv = T_map_odom_.inverse();

            for (const auto &odom_pt : odom_points)
            {
                const Eigen::Vector4f odom_pt_h(odom_pt.x(), odom_pt.y(), 0.0, 1.0);
                const Eigen::Vector4f map_pt_h = transform_inv * odom_pt_h;

                const pcl::PointXYZ search_point(map_pt_h.x(), map_pt_h.y(), map_pt_h.z());
                std::vector<int> indices(1);
                std::vector<float> distances(1);

                if (map_kdtree_->nearestKSearch(search_point, 1, indices, distances) > 0)
                {
                    const auto &map_pt = map_landmarks_pcl_->points[indices[0]];

                    geometry_msgs::msg::Point p_map, p_odom_transformed;
                    p_map.x = map_pt.x;
                    p_map.y = map_pt.y;
                    p_map.z = map_pt.z;
                    p_odom_transformed.x = search_point.x;
                    p_odom_transformed.y = search_point.y;
                    p_odom_transformed.z = search_point.z;

                    line_list.points.push_back(p_map);
                    line_list.points.push_back(p_odom_transformed);
                }
            }

            markers.markers.push_back(line_list);
            alignment_marker_pub_->publish(markers);
        }

        void MapAlignmentComponent::publishAlignmentStatus()
        {
            // Publish alignment_done = false while still trying
            std_msgs::msg::Bool status_msg;
            status_msg.data = is_alignment_done_.load();
            alignment_status_pub_->publish(status_msg);

            // Broadcast the current initial-guess TF so the TF tree stays connected
            static tf2_ros::StaticTransformBroadcaster static_br(*this);

            const Eigen::Matrix3f R = T_map_odom_.block<3, 3>(0, 0);
            const Eigen::Vector3f t = T_map_odom_.block<3, 1>(0, 3);
            const Eigen::Quaternionf q(R);

            geometry_msgs::msg::TransformStamped transform_msg;
            transform_msg.header.stamp = get_clock()->now();
            transform_msg.header.frame_id = map_frame_;
            transform_msg.child_frame_id = odom_frame_;
            transform_msg.transform.translation.x = t.x();
            transform_msg.transform.translation.y = t.y();
            transform_msg.transform.translation.z = t.z();
            transform_msg.transform.rotation.x = q.x();
            transform_msg.transform.rotation.y = q.y();
            transform_msg.transform.rotation.z = q.z();
            transform_msg.transform.rotation.w = q.w();

            static_br.sendTransform(transform_msg);
        }

        void MapAlignmentComponent::handleAlignmentStatusRequest(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            response->success = is_alignment_done_.load();
            if (response->success)
            {
                const Eigen::Vector3f t = T_map_odom_.block<3, 1>(0, 3);
                response->message = "Alignment complete. Offset: [" +
                                    std::to_string(t.x()) + ", " +
                                    std::to_string(t.y()) + ", " +
                                    std::to_string(t.z()) + "]";
            }
            else
            {
                response->message = "Alignment not yet complete. Observed landmarks: " +
                                    std::to_string(observed_landmarks_.size());
            }
        }

        void MapAlignmentComponent::unloadPerceptionComponents()
        {
            RCLCPP_INFO(get_logger(),
                        "Scheduling perception component unload from '%s'...",
                        container_name_.c_str());

            // Deferred — let the current callback finish and final messages flush
            cleanup_timer_ = create_wall_timer(
                std::chrono::seconds(2),
                std::bind(&MapAlignmentComponent::performUnload, this));
        }

        void MapAlignmentComponent::performUnload()
        {
            cleanup_timer_->cancel();

            if (!list_nodes_client_->service_is_ready())
            {
                RCLCPP_WARN(get_logger(),
                            "Container ListNodes service not ready — "
                            "cannot unload components (container: '%s')",
                            container_name_.c_str());
                return;
            }

            auto request =
                std::make_shared<composition_interfaces::srv::ListNodes::Request>();

            list_nodes_client_->async_send_request(
                request,
                [this](rclcpp::Client<composition_interfaces::srv::ListNodes>::SharedFuture future)
                {
                    try
                    {
                        auto response = future.get();

                        for (size_t i = 0; i < response->full_node_names.size(); ++i)
                        {
                            const auto &name = response->full_node_names[i];

                            // Unload trunk_segmentation — the heavy LiDAR processing node
                            if (name.find("trunk_segmentation") != std::string::npos)
                            {
                                RCLCPP_INFO(get_logger(),
                                            "Unloading '%s' (uid=%lu) from container",
                                            name.c_str(),
                                            static_cast<unsigned long>(response->unique_ids[i]));

                                auto unload_req =
                                    std::make_shared<composition_interfaces::srv::UnloadNode::Request>();
                                unload_req->unique_id = response->unique_ids[i];

                                unload_node_client_->async_send_request(
                                    unload_req,
                                    [this, name](rclcpp::Client<composition_interfaces::srv::UnloadNode>::SharedFuture f)
                                    {
                                        try
                                        {
                                            auto resp = f.get();
                                            if (resp->success)
                                                RCLCPP_INFO(get_logger(), "Unloaded: %s", name.c_str());
                                            else
                                                RCLCPP_ERROR(get_logger(), "Failed to unload %s: %s",
                                                             name.c_str(), resp->error_message.c_str());
                                        }
                                        catch (const std::exception &e)
                                        {
                                            RCLCPP_ERROR(get_logger(), "Unload call failed for %s: %s",
                                                         name.c_str(), e.what());
                                        }
                                    });
                                break;
                            }
                        }
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_ERROR(get_logger(), "ListNodes call failed: %s", e.what());
                    }
                });
        }

    } // namespace perception
} // namespace uosm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uosm::perception::MapAlignmentComponent)