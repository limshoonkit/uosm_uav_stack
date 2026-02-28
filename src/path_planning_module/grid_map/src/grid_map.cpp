/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Portions adapted from EGO-Planner-v2: https://github.com/ZJU-FAST-Lab/EGO-Planner-v2
 */

#include "../include/grid_map.hpp"
#include <node_params.hpp>

namespace ego_planner
{
    GridMap::GridMap(const rclcpp::Node::SharedPtr &node)
    {
        node_ = node;
        using uosm::util::getParam;

        params_.resolution_ = getParam<float>(node_, "grid_map.resolution", 0.1f);
        params_.min_clearance_ = getParam<float>(node_, "grid_map.min_clearance", 0.3f);
        params_.local_update_range3d_(0) = getParam<float>(node_, "grid_map.local_update_range_x", 8.0f);
        params_.local_update_range3d_(1) = getParam<float>(node_, "grid_map.local_update_range_y", 8.0f);
        params_.local_update_range3d_(2) = getParam<float>(node_, "grid_map.local_update_range_z", 3.0f);
        params_.obstacles_inflation_ = getParam<float>(node_, "grid_map.obstacles_inflation", 1.0f);
        params_.enable_virtual_wall_ = getParam<bool>(node_, "grid_map.enable_virtual_wall", false);
        params_.virtual_ceil_ = getParam<float>(node_, "grid_map.virtual_ceil", 1.0f);
        params_.virtual_ground_ = getParam<float>(node_, "grid_map.virtual_ground", -0.1f);
        params_.p_hit_ = getParam<double>(node_, "grid_map.p_hit", 0.65);
        params_.p_miss_ = getParam<double>(node_, "grid_map.p_miss", 0.35);
        params_.p_min_ = getParam<double>(node_, "grid_map.p_min", 0.12);
        params_.p_max_ = getParam<double>(node_, "grid_map.p_max", 0.97);
        params_.p_occ_ = getParam<double>(node_, "grid_map.p_occ", 0.80);
        params_.fading_time_ = getParam<float>(node_, "grid_map.fading_time", 0.0f);
        params_.frame_id_ = getParam<std::string>(node_, "grid_map.frame_id", "map");
        double visualize_rate_ = getParam<float>(node_, "grid_map.visualize_rate", 1.0f);
        double fading_rate_ = getParam<float>(node_, "grid_map.fading_rate", 1.0f);
        bool is_sim_ = getParam<bool>(node_, "grid_map.is_sim", false);
        params_.max_cloud_range_ = getParam<double>(node_, "grid_map.max_cloud_range", 5.5);
        params_.confidence_near_range_ = getParam<double>(node_, "grid_map.confidence_near_range", 2.0);
        params_.confidence_far_range_ = getParam<double>(node_, "grid_map.confidence_far_range", 5.5);
        params_.min_confidence_weight_ = getParam<double>(node_, "grid_map.min_confidence_weight", 0.3);
        if (params_.max_cloud_range_ <= 0.0)
        {
            RCLCPP_WARN(node_->get_logger(), "max_cloud_range <= 0 (%.2f), defaulting to 5.5m", params_.max_cloud_range_);
            params_.max_cloud_range_ = 5.5;
        }
        params_.max_cloud_range_sq_ = params_.max_cloud_range_ * params_.max_cloud_range_;
        if (params_.min_confidence_weight_ <= 0.0)
        {
            RCLCPP_WARN(node_->get_logger(), "min_confidence_weight <= 0 (%.2f), clamping to 0.01", params_.min_confidence_weight_);
            params_.min_confidence_weight_ = 0.01;
        }
        params_.max_lidar_range_ = getParam<double>(node_, "grid_map.max_lidar_range", 15.0);
        params_.trunk_extrude_height_ = getParam<double>(node_, "grid_map.trunk_extrude_height", 3.0);
        params_.laser_frame_ = getParam<std::string>(node_, "grid_map.laser_frame", "laser");
        if (params_.max_lidar_range_ <= 0.0)
        {
            RCLCPP_WARN(node_->get_logger(), "max_lidar_range <= 0 (%.2f), defaulting to 15.0m", params_.max_lidar_range_);
            params_.max_lidar_range_ = 15.0;
        }
        params_.max_lidar_range_sq_ = params_.max_lidar_range_ * params_.max_lidar_range_;

        params_.inf_grid_ = std::ceil((params_.obstacles_inflation_ - 1e-5) / params_.resolution_);
        if (params_.inf_grid_ > 4)
        {
            params_.inf_grid_ = 4U;
            params_.resolution_ = params_.obstacles_inflation_ / params_.inf_grid_;
            RCLCPP_WARN(node_->get_logger(), "Inflation grid size too large, set resolution to %.4f", params_.resolution_);
        }
        params_.resolution_inv_ = 1.0f / params_.resolution_;
        params_.min_clearance_sq_ = params_.min_clearance_ * params_.min_clearance_;
        params_.local_update_range3i_ = (params_.local_update_range3d_ * params_.resolution_inv_).array().ceil().cast<int>();
        params_.local_update_range3d_ = params_.local_update_range3i_.array().cast<double>() * params_.resolution_;
        data_.ringbuffer_size3i_ = 2U * params_.local_update_range3i_;
        data_.ringbuffer_inf_size3i_ = data_.ringbuffer_size3i_ + Eigen::Vector3i(2U * params_.inf_grid_, 2U * params_.inf_grid_, 2U * params_.inf_grid_);

        params_.prob_hit_log_ = logit(params_.p_hit_);
        params_.prob_miss_log_ = logit(params_.p_miss_);
        params_.clamp_min_log_ = logit(params_.p_min_);
        params_.clamp_max_log_ = logit(params_.p_max_);
        params_.min_occupancy_log_ = logit(params_.p_occ_);

        // initialize data buffers
        Eigen::Vector3i map_voxel_num3i = 2U * params_.local_update_range3i_;
        int buffer_size = map_voxel_num3i(0) * map_voxel_num3i(1) * map_voxel_num3i(2);
        int buffer_inf_size = (map_voxel_num3i(0) + 2U * params_.inf_grid_) * (map_voxel_num3i(1) + 2 * params_.inf_grid_) * (map_voxel_num3i(2) + 2 * params_.inf_grid_);
        data_.ringbuffer_origin3i_ = Eigen::Vector3i(0, 0, 0);
        data_.ringbuffer_inf_origin3i_ = Eigen::Vector3i(0, 0, 0);

        data_.occupancy_buffer_ = std::vector<double>(buffer_size, params_.clamp_min_log_ - 1.0f); // Start with a value lower than the minimum occupancy log
        data_.occupancy_buffer_inflate_ = std::vector<uint16_t>(buffer_inf_size, 0);
        data_.count_hit_and_miss_ = std::vector<short>(buffer_size, 0);
        data_.count_hit_ = std::vector<short>(buffer_size, 0);
        data_.count_weight_ = std::vector<float>(buffer_size, 0.0f);
        data_.flag_rayend_ = std::vector<char>(buffer_size, -1);
        data_.flag_traverse_ = std::vector<char>(buffer_size, -1);

        data_.cache_voxel_ = std::vector<Eigen::Vector3i>(buffer_size, Eigen::Vector3i(0, 0, 0));
        data_.raycast_num_ = 0;
        data_.cache_voxel_cnt_ = 0;

        RCLCPP_INFO(node_->get_logger(), "Grid Map Parameters:");
        RCLCPP_INFO(node_->get_logger(), " * resolution: %.2f", params_.resolution_);
        RCLCPP_INFO(node_->get_logger(), " * min clearance: %.2f", params_.min_clearance_);
        RCLCPP_INFO(node_->get_logger(), " * local update range: [%.2f, %.2f, %.2f]", params_.local_update_range3d_(0), params_.local_update_range3d_(1), params_.local_update_range3d_(2));
        RCLCPP_INFO(node_->get_logger(), " * obstacles inflation: %.2f", params_.obstacles_inflation_);
        RCLCPP_INFO(node_->get_logger(), " * enable virtual wall: %s", params_.enable_virtual_wall_ ? "true" : "false");
        RCLCPP_INFO(node_->get_logger(), " * virtual ground: %.2f", params_.virtual_ground_);
        RCLCPP_INFO(node_->get_logger(), " * virtual ceil: %.2f", params_.virtual_ceil_);
        RCLCPP_INFO(node_->get_logger(), " * prob_hit_log_: %.2f", params_.prob_hit_log_);
        RCLCPP_INFO(node_->get_logger(), " * prob_miss_log_: %.2f", params_.prob_miss_log_);
        RCLCPP_INFO(node_->get_logger(), " * clamp_min_log_: %.2f", params_.clamp_min_log_);
        RCLCPP_INFO(node_->get_logger(), " * clamp_max_log_: %.2f", params_.clamp_max_log_);
        RCLCPP_INFO(node_->get_logger(), " * min_occupancy_log_: %.2f", params_.min_occupancy_log_);
        RCLCPP_INFO(node_->get_logger(), " * frame ID: %s", params_.frame_id_.c_str());
        RCLCPP_INFO(node_->get_logger(), " * visualize rate: %.2f Hz", visualize_rate_);
        RCLCPP_INFO(node_->get_logger(), " * fading rate: %.2f Hz", fading_rate_);
        RCLCPP_INFO(node_->get_logger(), " * max cloud range: %.2f m", params_.max_cloud_range_);
        RCLCPP_INFO(node_->get_logger(), " * confidence near/far: %.2f / %.2f m", params_.confidence_near_range_, params_.confidence_far_range_);
        RCLCPP_INFO(node_->get_logger(), " * min confidence weight: %.2f", params_.min_confidence_weight_);
        RCLCPP_INFO(node_->get_logger(), " * max lidar range: %.2f m", params_.max_lidar_range_);
        RCLCPP_INFO(node_->get_logger(), " * trunk extrude height: %.2f m", params_.trunk_extrude_height_);
        RCLCPP_INFO(node_->get_logger(), " * laser frame: %s", params_.laser_frame_.c_str());
        RCLCPP_INFO(node_->get_logger(), "********************************");

        data_.is_initialized_ = false;
        data_.has_odom_ = false;

        data_.body_pos_ = Eigen::Vector3d::Zero();
        data_.body_r_m_ = Eigen::Quaterniond::Identity();

        // Init Pub/Sub
        map_inf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy_inflate", 10);

        cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            "grid_map/cloud", rclcpp::QoS(10),
            is_sim_ ? std::bind(&GridMap::cloudCallbackSim, this, std::placeholders::_1)
                    : std::bind(&GridMap::cloudCallback, this, std::placeholders::_1));

        lidar_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
            "grid_map/scan", rclcpp::QoS(10),
            std::bind(&GridMap::lidarCallback, this, std::placeholders::_1));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Init Timer
        visualize_timer_ = node->create_wall_timer(
            std::chrono::duration<double>(1.0 / visualize_rate_),
            std::bind(&GridMap::visualizeMapCallback, this));

        if (params_.fading_time_ > 0)
            fading_timer_ = node_->create_wall_timer(
                std::chrono::duration<double>(1.0 / fading_rate_),
                std::bind(&GridMap::fadingCallback, this));
    }

    void GridMap::fadingCallback()
    {
        const double reduce = (params_.clamp_max_log_ - params_.min_occupancy_log_) / (params_.fading_time_ * 2);
        const double low_thres = params_.clamp_min_log_ + reduce;

        for (size_t i = 0; i < data_.occupancy_buffer_.size(); ++i)
        {
            if (data_.occupancy_buffer_[i] > low_thres)
            {
                bool obs_flag = data_.occupancy_buffer_[i] >= params_.min_occupancy_log_;
                data_.occupancy_buffer_[i] -= reduce;
                if (obs_flag && data_.occupancy_buffer_[i] < params_.min_occupancy_log_)
                {
                    Eigen::Vector3i idx = BufIdx2GlobalIdx(i);
                    int inf_buf_idx = globalIdx2InfBufIdx(idx);
                    changeInfBuf(false, inf_buf_idx, idx);
                }
            }
        }
    }

    void GridMap::setOdom(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        /* get pose */
        Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                       odom->pose.pose.orientation.x,
                                                       odom->pose.pose.orientation.y,
                                                       odom->pose.pose.orientation.z);
        Eigen::Matrix4d body2world;
        body2world.block<3, 3>(0, 0) = body_q.toRotationMatrix();
        body2world(0, 3) = odom->pose.pose.position.x;
        body2world(1, 3) = odom->pose.pose.position.y;
        body2world(2, 3) = odom->pose.pose.position.z;
        body2world(3, 0) = 0.0;
        body2world(3, 1) = 0.0;
        body2world(3, 2) = 0.0;
        body2world(3, 3) = 1.0;

        Eigen::Matrix4d body_T = body2world;
        data_.body_pos_(0) = body_T(0, 3);
        data_.body_pos_(1) = body_T(1, 3);
        data_.body_pos_(2) = body_T(2, 3);
        data_.body_r_m_ = body_T.block<3, 3>(0, 0);

        data_.has_odom_ = true;
    }

    void GridMap::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstPtr &msg)
    {
        RCLCPP_DEBUG(node_->get_logger(), "Received point cloud data.");

        if (!data_.has_odom_)
        {
            RCLCPP_WARN(node_->get_logger(), "No odom!");
            return;
        }

        if (data_.body_pos_.hasNaN())
        {
            RCLCPP_WARN(node_->get_logger(), "No valid position data!");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *latest_cloud);

        if (latest_cloud->points.empty())
        {
            // RCLCPP_WARN(node_->get_logger(), "No point cloud data!");
            return;
        }

        // Transform from point cloud frame (e.g. zedm_camera_link) to base_link at cloud time
        // so we fuse in map frame correctly; avoids misalignment when cloud is not in base_link
        Eigen::Isometry3d cloud_to_base = Eigen::Isometry3d::Identity();
        if (msg->header.frame_id != "base_link")
        {
            try
            {
                rclcpp::Time stamp(msg->header.stamp.sec, msg->header.stamp.nanosec);
                auto tf_msg = tf_buffer_->lookupTransform(
                    "base_link", msg->header.frame_id, stamp,
                    rclcpp::Duration::from_seconds(0.05));
                cloud_to_base.translation() = Eigen::Vector3d(
                    tf_msg.transform.translation.x,
                    tf_msg.transform.translation.y,
                    tf_msg.transform.translation.z);
                Eigen::Quaterniond q(
                    tf_msg.transform.rotation.w,
                    tf_msg.transform.rotation.x,
                    tf_msg.transform.rotation.y,
                    tf_msg.transform.rotation.z);
                cloud_to_base.linear() = q.toRotationMatrix();
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                     "grid_map: cloud frame '%s' -> base_link failed (%s), assuming base_link",
                                     msg->header.frame_id.c_str(), ex.what());
            }
        }

        // Use pose at cloud time so 30 Hz odom vs 10 Hz cloud doesn't cause temporal mismatch
        rclcpp::Time cloud_stamp(msg->header.stamp.sec, msg->header.stamp.nanosec);
        Eigen::Matrix3d R_pose = data_.body_r_m_;
        Eigen::Vector3d p_pose = data_.body_pos_;
        if (!getBasePoseAtTime(cloud_stamp, R_pose, p_pose))
        {
            /* fallback: use latest odom (R_pose, p_pose already set above) */
        }

        if (data_.pnts.size() < latest_cloud->points.size())
        {
            size_t new_size = static_cast<size_t>(latest_cloud->points.size() * 1.05); // extra 5% buffer
            data_.pnts.resize(new_size);
            data_.pnt_weights_.resize(new_size, 1.0f);
            RCLCPP_INFO(node_->get_logger(), "Resized data points buffer to %zu", data_.pnts.size());
        }

        data_.pnts_cnt_ = 0;

        const double conf_range_inv = (params_.confidence_far_range_ > params_.confidence_near_range_)
                                          ? 1.0 / (params_.confidence_far_range_ - params_.confidence_near_range_)
                                          : 0.0;

        for (const auto &point : latest_cloud->points)
        {
            if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
                continue;

            Eigen::Vector3d pt_cloud(point.x, point.y, point.z);
            Eigen::Vector3d pt_base = cloud_to_base * pt_cloud;
            double dist_sq = pt_base.squaredNorm();

            // Filter out points too close to sensor
            if (dist_sq < params_.min_clearance_sq_)
                continue;

            // Filter out points beyond max cloud range
            if (dist_sq > params_.max_cloud_range_sq_)
                continue;

            // Compute distance-dependent confidence weight
            float weight = 1.0f;
            double dist = std::sqrt(dist_sq);
            if (dist > params_.confidence_near_range_)
            {
                float t = static_cast<float>((dist - params_.confidence_near_range_) * conf_range_inv);
                t = std::min(t, 1.0f);
                weight = 1.0f - (1.0f - static_cast<float>(params_.min_confidence_weight_)) * t;
            }

            // Transform base_link -> map using pose at cloud time (time-sync with sensor)
            Eigen::Vector3d pt_world = R_pose * pt_base + p_pose;

            data_.pnts[data_.pnts_cnt_] = pt_world;
            data_.pnt_weights_[data_.pnts_cnt_] = weight;
            data_.pnts_cnt_++;
            if (data_.pnts_cnt_ >= static_cast<int>(data_.pnts.size()))
            {
                RCLCPP_WARN(node_->get_logger(), "Reached proj_points buffer limit");
                break;
            }
        }
        if (data_.pnts_cnt_ > 0)
        {
            /* update occupancy */
            moveRingBuffer();
            raycastProcess();
            clearAndInflateLocalMap();
        }
    }

    void GridMap::cloudCallbackSim(const sensor_msgs::msg::PointCloud2::ConstPtr &msg)
    {
        /* Note: no obstacle elimination in this function! */
        if (!data_.has_odom_)
            return;

        pcl::PointCloud<pcl::PointXYZ> latest_cloud;
        pcl::fromROSMsg(*msg, latest_cloud);

        if (latest_cloud.points.size() == 0)
            return;

        if (data_.body_pos_.hasNaN())
            return;

        moveRingBuffer();

        pcl::PointXYZ pt;
        Eigen::Vector3d p3d, p3d_inf;
        for (size_t i = 0; i < latest_cloud.points.size(); ++i)
        {
            pt = latest_cloud.points[i];
            p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;
            if (p3d.array().isNaN().sum())
                continue;

            if (isInBuf(p3d))
            {
                /* inflate the point */
                Eigen::Vector3i idx = pos2GlobalIdx(p3d);
                int buf_id = globalIdx2BufIdx(idx);
                int inf_buf_id = globalIdx2InfBufIdx(idx);
                data_.occupancy_buffer_[buf_id] = params_.clamp_max_log_;

                if (data_.occupancy_buffer_inflate_[inf_buf_id] < GRID_MAP_OBS_FLAG && data_.occupancy_buffer_[buf_id] >= params_.min_occupancy_log_)
                {
                    changeInfBuf(true, inf_buf_id, idx);
                }
            }
        }
    }

    bool GridMap::getBasePoseAtTime(const rclcpp::Time &time,
                                   Eigen::Matrix3d &R_out, Eigen::Vector3d &p_out,
                                   const rclcpp::Duration &timeout)
    {
        try
        {
            auto tf_msg = tf_buffer_->lookupTransform(
                params_.frame_id_, "base_link", time, timeout);
            p_out(0) = tf_msg.transform.translation.x;
            p_out(1) = tf_msg.transform.translation.y;
            p_out(2) = tf_msg.transform.translation.z;
            Eigen::Quaterniond q(
                tf_msg.transform.rotation.w,
                tf_msg.transform.rotation.x,
                tf_msg.transform.rotation.y,
                tf_msg.transform.rotation.z);
            R_out = q.toRotationMatrix();
            return true;
        }
        catch (tf2::TransformException &)
        {
            return false;
        }
    }

    void GridMap::cacheLaserTransform()
    {
        try
        {
            auto tf_msg = tf_buffer_->lookupTransform(
                "base_link", params_.laser_frame_,
                tf2::TimePointZero);

            laser_to_body_ = Eigen::Isometry3d::Identity();
            laser_to_body_.translation() = Eigen::Vector3d(
                tf_msg.transform.translation.x,
                tf_msg.transform.translation.y,
                tf_msg.transform.translation.z);
            Eigen::Quaterniond q(
                tf_msg.transform.rotation.w,
                tf_msg.transform.rotation.x,
                tf_msg.transform.rotation.y,
                tf_msg.transform.rotation.z);
            laser_to_body_.linear() = q.toRotationMatrix();

            laser_transform_cached_ = true;
            RCLCPP_INFO(node_->get_logger(), "Cached laser -> base_link transform");
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "Failed to lookup laser transform: %s", ex.what());
        }
    }

    void GridMap::extrudeTrunkColumns(const std::vector<Eigen::Vector3d> &hits, int hit_cnt)
    {
        if (params_.trunk_extrude_height_ <= 0.0 || hit_cnt <= 0)
            return;

        for (int i = 0; i < hit_cnt; ++i)
        {
            const Eigen::Vector3d &hit = hits[i];
            double z_bottom = hit(2) - params_.trunk_extrude_height_;
            if (params_.enable_virtual_wall_)
                z_bottom = std::max(z_bottom, params_.virtual_ground_);

            for (double z = hit(2) - params_.resolution_; z >= z_bottom; z -= params_.resolution_)
            {
                Eigen::Vector3d col_pt(hit(0), hit(1), z);
                if (!isInBuf(col_pt))
                    continue;

                Eigen::Vector3i idx = pos2GlobalIdx(col_pt);
                int buf_id = globalIdx2BufIdx(idx);
                int inf_buf_id = globalIdx2InfBufIdx(idx);

                data_.occupancy_buffer_[buf_id] = params_.clamp_max_log_;
                if (data_.occupancy_buffer_inflate_[inf_buf_id] < GRID_MAP_OBS_FLAG)
                    changeInfBuf(true, inf_buf_id, idx);
            }
        }
    }

    void GridMap::lidarCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg)
    {
        if (!data_.has_odom_ || data_.body_pos_.hasNaN())
            return;

        // Cache the laser -> base_link transform on first call
        if (!laser_transform_cached_)
        {
            cacheLaserTransform();
            if (!laser_transform_cached_)
                return;
        }

        // Use pose at scan time so odom rate vs scan rate mismatch doesn't cause temporal error
        rclcpp::Time scan_stamp(msg->header.stamp.sec, msg->header.stamp.nanosec);
        Eigen::Matrix3d R_pose = data_.body_r_m_;
        Eigen::Vector3d p_pose = data_.body_pos_;
        getBasePoseAtTime(scan_stamp, R_pose, p_pose);

        // Ensure point buffer has capacity for scan points
        const size_t scan_size = msg->ranges.size();
        if (data_.pnts.size() < scan_size)
        {
            data_.pnts.resize(scan_size * 1.05);
            data_.pnt_weights_.resize(scan_size * 1.05, 1.0f);
        }

        data_.pnts_cnt_ = 0;

        // Store LiDAR hit positions for trunk extrusion
        static std::vector<Eigen::Vector3d> lidar_hits;
        if (lidar_hits.size() < scan_size)
            lidar_hits.resize(scan_size);
        int lidar_hit_cnt = 0;

        float angle = msg->angle_min;
        for (size_t i = 0; i < scan_size; ++i, angle += msg->angle_increment)
        {
            float range = msg->ranges[i];

            // Skip invalid ranges
            if (range < msg->range_min || range > msg->range_max ||
                std::isnan(range) || std::isinf(range))
                continue;

            // Skip beyond max lidar range
            if (range * range > params_.max_lidar_range_sq_)
                continue;

            // Convert polar to Cartesian in laser frame
            Eigen::Vector3d pt_laser(
                static_cast<double>(range) * std::cos(angle),
                static_cast<double>(range) * std::sin(angle),
                0.0);

            // Transform to body frame using cached static TF
            Eigen::Vector3d pt_body = laser_to_body_ * pt_laser;

            // Transform to world frame using pose at scan time (time-sync with sensor)
            Eigen::Vector3d pt_world = R_pose * pt_body + p_pose;

            data_.pnts[data_.pnts_cnt_] = pt_world;
            data_.pnt_weights_[data_.pnts_cnt_] = 1.0f; // LiDAR: full confidence
            data_.pnts_cnt_++;

            // Record hit for trunk extrusion
            lidar_hits[lidar_hit_cnt++] = pt_world;

            if (data_.pnts_cnt_ >= static_cast<int>(data_.pnts.size()))
                break;
        }

        if (data_.pnts_cnt_ > 0)
        {
            moveRingBuffer();
            raycastProcess();
            clearAndInflateLocalMap();
            extrudeTrunkColumns(lidar_hits, lidar_hit_cnt);
        }
    }

    void GridMap::moveRingBuffer()
    {
        if (!data_.is_initialized_)
            initMapBoundary();

        Eigen::Vector3i center_new = pos2GlobalIdx(data_.body_pos_);
        Eigen::Vector3i ringbuffer_lowbound3i_new = center_new - params_.local_update_range3i_;
        Eigen::Vector3d ringbuffer_lowbound3d_new = ringbuffer_lowbound3i_new.cast<double>() * params_.resolution_;
        Eigen::Vector3i ringbuffer_upbound3i_new = center_new + params_.local_update_range3i_;
        Eigen::Vector3d ringbuffer_upbound3d_new = ringbuffer_upbound3i_new.cast<double>() * params_.resolution_;
        ringbuffer_upbound3i_new -= Eigen::Vector3i(1, 1, 1);

        const Eigen::Vector3i inf_grid3i(params_.inf_grid_, params_.inf_grid_, params_.inf_grid_);
        const Eigen::Vector3d inf_grid3d = inf_grid3i.array().cast<double>() * params_.resolution_;
        Eigen::Vector3i ringbuffer_inf_lowbound3i_new = ringbuffer_lowbound3i_new - inf_grid3i;
        Eigen::Vector3d ringbuffer_inf_lowbound3d_new = ringbuffer_lowbound3d_new - inf_grid3d;
        Eigen::Vector3i ringbuffer_inf_upbound3i_new = ringbuffer_upbound3i_new + inf_grid3i;
        Eigen::Vector3d ringbuffer_inf_upbound3d_new = ringbuffer_upbound3d_new + inf_grid3d;

        if (center_new(0) < data_.center_last3i_(0))
            clearBuffer(0, ringbuffer_upbound3i_new(0));
        if (center_new(0) > data_.center_last3i_(0))
            clearBuffer(1, ringbuffer_lowbound3i_new(0));
        if (center_new(1) < data_.center_last3i_(1))
            clearBuffer(2, ringbuffer_upbound3i_new(1));
        if (center_new(1) > data_.center_last3i_(1))
            clearBuffer(3, ringbuffer_lowbound3i_new(1));
        if (center_new(2) < data_.center_last3i_(2))
            clearBuffer(4, ringbuffer_upbound3i_new(2));
        if (center_new(2) > data_.center_last3i_(2))
            clearBuffer(5, ringbuffer_lowbound3i_new(2));

        for (int i = 0; i < 3; ++i)
        {
            while (data_.ringbuffer_origin3i_(i) < data_.ringbuffer_lowbound3i_(i))
            {
                data_.ringbuffer_origin3i_(i) += data_.ringbuffer_size3i_(i);
            }
            while (data_.ringbuffer_origin3i_(i) > data_.ringbuffer_upbound3i_(i))
            {
                data_.ringbuffer_origin3i_(i) -= data_.ringbuffer_size3i_(i);
            }

            while (data_.ringbuffer_inf_origin3i_(i) < data_.ringbuffer_inf_lowbound3i_(i))
            {
                data_.ringbuffer_inf_origin3i_(i) += data_.ringbuffer_inf_size3i_(i);
            }
            while (data_.ringbuffer_inf_origin3i_(i) > data_.ringbuffer_inf_upbound3i_(i))
            {
                data_.ringbuffer_inf_origin3i_(i) -= data_.ringbuffer_inf_size3i_(i);
            }
        }

        data_.center_last3i_ = center_new;
        data_.ringbuffer_lowbound3i_ = ringbuffer_lowbound3i_new;
        data_.ringbuffer_lowbound3d_ = ringbuffer_lowbound3d_new;
        data_.ringbuffer_upbound3i_ = ringbuffer_upbound3i_new;
        data_.ringbuffer_upbound3d_ = ringbuffer_upbound3d_new;
        data_.ringbuffer_inf_lowbound3i_ = ringbuffer_inf_lowbound3i_new;
        data_.ringbuffer_inf_lowbound3d_ = ringbuffer_inf_lowbound3d_new;
        data_.ringbuffer_inf_upbound3i_ = ringbuffer_inf_upbound3i_new;
        data_.ringbuffer_inf_upbound3d_ = ringbuffer_inf_upbound3d_new;
    }

    void GridMap::raycastProcess()
    {
        data_.cache_voxel_cnt_ = 0;
        data_.raycast_num_ += 1;

        RayCaster raycaster;
        Eigen::Vector3d ray_pt, pt_w;
        int pts_num = 0;

        for (int i = 0; i < data_.pnts_cnt_; ++i)
        {
            int vox_idx;
            float weight = data_.pnt_weights_[i];
            pt_w = data_.pnts[i];
            if (!isInBuf(pt_w))
            {
                pt_w = closestPointInMap(pt_w, data_.body_pos_);
                pts_num++;
                vox_idx = setCacheOccupancy(pt_w, 0);
            }
            else
            {
                pts_num++;
                vox_idx = setCacheOccupancy(pt_w, 1, weight);
            }

            if (vox_idx != INVALID_IDX)
            {
                if (data_.flag_rayend_[vox_idx] == data_.raycast_num_)
                    continue;
                else
                    data_.flag_rayend_[vox_idx] = data_.raycast_num_;
            }

            raycaster.setInput(pt_w / params_.resolution_, data_.body_pos_ / params_.resolution_);
            while (raycaster.step(ray_pt))
            {
                Eigen::Vector3d tmp = (ray_pt + Eigen::Vector3d(0.5, 0.5, 0.5)) * params_.resolution_;
                pts_num++;
                vox_idx = setCacheOccupancy(tmp, 0);
                if (vox_idx != INVALID_IDX)
                {
                    if (data_.flag_traverse_[vox_idx] == data_.raycast_num_)
                        break;
                    else
                        data_.flag_traverse_[vox_idx] = data_.raycast_num_;
                }
            }
        }

        for (int i = 0; i < data_.cache_voxel_cnt_; ++i)
        {
            int idx_ctns = globalIdx2BufIdx(data_.cache_voxel_[i]);
            short hits = data_.count_hit_[idx_ctns];
            short misses = data_.count_hit_and_miss_[idx_ctns] - hits;

            double log_odds_update;
            if (hits >= misses)
            {
                // Occupied: scale by average confidence weight of hit points
                float avg_weight = (hits > 0) ? data_.count_weight_[idx_ctns] / hits : 1.0f;
                log_odds_update = params_.prob_hit_log_ * avg_weight;
            }
            else
            {
                // Free space
                log_odds_update = params_.prob_miss_log_;
            }

            data_.count_hit_[idx_ctns] = data_.count_hit_and_miss_[idx_ctns] = 0;
            data_.count_weight_[idx_ctns] = 0.0f;

            if (log_odds_update >= 0 && data_.occupancy_buffer_[idx_ctns] >= params_.clamp_max_log_)
                continue;
            else if (log_odds_update <= 0 && data_.occupancy_buffer_[idx_ctns] <= params_.clamp_min_log_)
                continue;

            data_.occupancy_buffer_[idx_ctns] = std::min(std::max(data_.occupancy_buffer_[idx_ctns] + log_odds_update, params_.clamp_min_log_),
                                                         params_.clamp_max_log_);
        }
    }

    void GridMap::clearAndInflateLocalMap()
    {
        // Update the occupancy buffer for inflation after sufficient sensor warm-up
        if (data_.inflate_update_cnt_ < MIN_UPDATES_BEFORE_INFLATION)
        {
            data_.inflate_update_cnt_++;
            return;
        }

        for (int i = 0; i < data_.cache_voxel_cnt_; ++i)
        {
            Eigen::Vector3i idx = data_.cache_voxel_[i];
            int buf_id = globalIdx2BufIdx(idx);
            int inf_buf_id = globalIdx2InfBufIdx(idx);

            if (data_.occupancy_buffer_inflate_[inf_buf_id] < GRID_MAP_OBS_FLAG && data_.occupancy_buffer_[buf_id] >= params_.min_occupancy_log_)
            {
                changeInfBuf(true, inf_buf_id, idx);
            }
            if (data_.occupancy_buffer_inflate_[inf_buf_id] >= GRID_MAP_OBS_FLAG && data_.occupancy_buffer_[buf_id] < params_.min_occupancy_log_)
            {
                changeInfBuf(false, inf_buf_id, idx);
            }
        }
    }

    void GridMap::initMapBoundary()
    {
        data_.is_initialized_ = true;

        data_.center_last3i_ = pos2GlobalIdx(data_.body_pos_);
        data_.ringbuffer_lowbound3i_ = data_.center_last3i_ - params_.local_update_range3i_;
        data_.ringbuffer_lowbound3d_ = data_.ringbuffer_lowbound3i_.cast<double>() * params_.resolution_;
        data_.ringbuffer_upbound3i_ = data_.center_last3i_ + params_.local_update_range3i_;
        data_.ringbuffer_upbound3d_ = data_.ringbuffer_upbound3i_.cast<double>() * params_.resolution_;
        data_.ringbuffer_upbound3i_ -= Eigen::Vector3i(1, 1, 1);

        const Eigen::Vector3i inf_grid3i(params_.inf_grid_, params_.inf_grid_, params_.inf_grid_);
        const Eigen::Vector3d inf_grid3d = inf_grid3i.array().cast<double>() * params_.resolution_;
        data_.ringbuffer_inf_lowbound3i_ = data_.ringbuffer_lowbound3i_ - inf_grid3i;
        data_.ringbuffer_inf_lowbound3d_ = data_.ringbuffer_lowbound3d_ - inf_grid3d;
        data_.ringbuffer_inf_upbound3i_ = data_.ringbuffer_upbound3i_ + inf_grid3i;
        data_.ringbuffer_inf_upbound3d_ = data_.ringbuffer_upbound3d_ + inf_grid3d;

        for (int i = 0; i < 3; ++i)
        {
            while (data_.ringbuffer_origin3i_(i) < data_.ringbuffer_lowbound3i_(i))
                data_.ringbuffer_origin3i_(i) += data_.ringbuffer_size3i_(i);
            while (data_.ringbuffer_origin3i_(i) > data_.ringbuffer_upbound3i_(i))
                data_.ringbuffer_origin3i_(i) -= data_.ringbuffer_size3i_(i);
            while (data_.ringbuffer_inf_origin3i_(i) < data_.ringbuffer_inf_lowbound3i_(i))
                data_.ringbuffer_inf_origin3i_(i) += data_.ringbuffer_inf_size3i_(i);
            while (data_.ringbuffer_inf_origin3i_(i) > data_.ringbuffer_inf_upbound3i_(i))
                data_.ringbuffer_inf_origin3i_(i) -= data_.ringbuffer_inf_size3i_(i);
        }
    }

    void GridMap::clearBuffer(char casein, int bound)
    {
        for (int x = (casein == 0 ? bound : data_.ringbuffer_lowbound3i_(0)); x <= (casein == 1 ? bound : data_.ringbuffer_upbound3i_(0)); ++x)
            for (int y = (casein == 2 ? bound : data_.ringbuffer_lowbound3i_(1)); y <= (casein == 3 ? bound : data_.ringbuffer_upbound3i_(1)); ++y)
                for (int z = (casein == 4 ? bound : data_.ringbuffer_lowbound3i_(2)); z <= (casein == 5 ? bound : data_.ringbuffer_upbound3i_(2)); ++z)
                {
                    Eigen::Vector3i id_global(x, y, z);
                    int id_buf = globalIdx2BufIdx(id_global);
                    int id_buf_inf = globalIdx2InfBufIdx(id_global);
                    Eigen::Vector3i id_global_inf_clr((casein == 0 ? x + params_.inf_grid_ : (casein == 1 ? x - params_.inf_grid_ : x)),
                                                      (casein == 2 ? y + params_.inf_grid_ : (casein == 3 ? y - params_.inf_grid_ : y)),
                                                      (casein == 4 ? z + params_.inf_grid_ : (casein == 5 ? z - params_.inf_grid_ : z)));

                    data_.count_hit_[id_buf] = 0;
                    data_.count_hit_and_miss_[id_buf] = 0;
                    data_.count_weight_[id_buf] = 0.0f;
                    data_.flag_traverse_[id_buf] = data_.raycast_num_;
                    data_.flag_rayend_[id_buf] = data_.raycast_num_;
                    data_.occupancy_buffer_[id_buf] = params_.clamp_min_log_;

                    if (data_.occupancy_buffer_inflate_[id_buf_inf] > GRID_MAP_OBS_FLAG)
                    {
                        changeInfBuf(false, id_buf_inf, id_global);
                    }
                }
    }

    Eigen::Vector3d GridMap::closestPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &body_pt)
    {
        Eigen::Vector3d diff = pt - body_pt;
        Eigen::Vector3d max_tc = data_.ringbuffer_upbound3d_ - body_pt;
        Eigen::Vector3d min_tc = data_.ringbuffer_lowbound3d_ - body_pt;
        double min_t = 1000000;

        for (int i = 0; i < 3; ++i)
        {
            if (std::fabs(diff[i]) > 0)
            {
                double t1 = max_tc[i] / diff[i];
                if (t1 > 0 && t1 < min_t)
                    min_t = t1;
                double t2 = min_tc[i] / diff[i];
                if (t2 > 0 && t2 < min_t)
                    min_t = t2;
            }
        }

        return body_pt + (min_t - 1e-3) * diff;
    }

    void GridMap::visualizeMapCallback()
    {
        if (!data_.is_initialized_)
            return;

        publishInfMap();
    }

    void GridMap::publishInfMap()
    {
        if (map_inf_pub_->get_subscription_count() <= 0)
            return;

        Eigen::Vector3d heading = (data_.body_r_m_ * Eigen::Matrix3d::Identity().block<3, 3>(0, 0).transpose()).block<3, 1>(0, 0);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        double lbz = params_.enable_virtual_wall_ ? std::max(data_.ringbuffer_inf_lowbound3d_(2), params_.virtual_ground_) : data_.ringbuffer_inf_lowbound3d_(2);
        double ubz = params_.enable_virtual_wall_ ? std::min(data_.ringbuffer_inf_upbound3d_(2), params_.virtual_ceil_) : data_.ringbuffer_inf_upbound3d_(2);
        if (data_.ringbuffer_inf_upbound3d_(0) - data_.ringbuffer_inf_lowbound3d_(0) > params_.resolution_ &&
            (data_.ringbuffer_inf_upbound3d_(1) - data_.ringbuffer_inf_lowbound3d_(1)) > params_.resolution_ && (ubz - lbz) > params_.resolution_)
            for (double xd = data_.ringbuffer_inf_lowbound3d_(0) + params_.resolution_ / 2; xd < data_.ringbuffer_inf_upbound3d_(0); xd += params_.resolution_)
                for (double yd = data_.ringbuffer_inf_lowbound3d_(1) + params_.resolution_ / 2; yd < data_.ringbuffer_inf_upbound3d_(1); yd += params_.resolution_)
                    for (double zd = lbz + params_.resolution_ / 2; zd < ubz; zd += params_.resolution_)
                    {
                        Eigen::Vector3d relative_dir = (Eigen::Vector3d(xd, yd, zd) - data_.body_pos_);
                        if (heading.dot(relative_dir.normalized()) > 0.5)
                        {
                            if (data_.occupancy_buffer_inflate_[globalIdx2InfBufIdx(pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)))])
                                cloud.push_back(pcl::PointXYZ(xd, yd, zd));
                        }
                    }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = params_.frame_id_;
        sensor_msgs::msg::PointCloud2 cloud_msg;

        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.stamp = node_->get_clock()->now();
        map_inf_pub_->publish(cloud_msg);
    }

} // namespace ego_planner