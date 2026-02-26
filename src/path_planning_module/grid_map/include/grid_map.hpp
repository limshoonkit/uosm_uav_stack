#ifndef _GRID_MAP_HPP_
#define _GRID_MAP_HPP_

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_ros/transforms.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "raycast.hpp"

#define logit(x) (log((x) / (1 - (x))))
#define GRID_MAP_OBS_FLAG 32767
#define INVALID_IDX -10000
#define MIN_UPDATES_BEFORE_INFLATION 5

namespace ego_planner
{
    class GridMap
    {
    private:
        struct GridMapParams
        {
            Eigen::Vector3d local_update_range3d_;
            Eigen::Vector3i local_update_range3i_;
            bool enable_virtual_wall_;

            double resolution_, resolution_inv_;
            double virtual_ceil_, virtual_ground_;
            double obstacles_inflation_;
            double min_clearance_, min_clearance_sq_;

            double p_hit_, p_miss_, p_min_, p_max_, p_occ_;
            double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_;
            double fading_time_;

            double max_cloud_range_, max_cloud_range_sq_;
            double confidence_near_range_, confidence_far_range_;
            double min_confidence_weight_;

            double max_lidar_range_, max_lidar_range_sq_;
            double trunk_extrude_height_;
            std::string laser_frame_;

            int inf_grid_;
            std::string frame_id_;
        };

        struct GridMapData
        {
            Eigen::Vector3i center_last3i_;
            Eigen::Vector3i ringbuffer_origin3i_;
            Eigen::Vector3d ringbuffer_lowbound3d_;
            Eigen::Vector3i ringbuffer_lowbound3i_;
            Eigen::Vector3d ringbuffer_upbound3d_;
            Eigen::Vector3i ringbuffer_upbound3i_;
            Eigen::Vector3i ringbuffer_size3i_;
            Eigen::Vector3i ringbuffer_inf_origin3i_;
            Eigen::Vector3d ringbuffer_inf_lowbound3d_;
            Eigen::Vector3i ringbuffer_inf_lowbound3i_;
            Eigen::Vector3d ringbuffer_inf_upbound3d_;
            Eigen::Vector3i ringbuffer_inf_upbound3i_;
            Eigen::Vector3i ringbuffer_inf_size3i_;

            // main map data, occupancy of each voxel
            std::vector<double> occupancy_buffer_;
            std::vector<uint16_t> occupancy_buffer_inflate_;
            std::vector<Eigen::Vector3i> cache_voxel_;
            int cache_voxel_cnt_;

            // flag
            bool is_initialized_;
            bool has_odom_;
            int inflate_update_cnt_ = 0;

            // body pose & rotation data
            Eigen::Vector3d body_pos_;
            Eigen::Matrix3d body_r_m_;

            // raycast
            std::vector<Eigen::Vector3d> pnts;
            std::vector<float> pnt_weights_;
            int pnts_cnt_;
            std::vector<short> count_hit_, count_hit_and_miss_;
            std::vector<float> count_weight_;
            std::vector<char> flag_traverse_, flag_rayend_;
            char raycast_num_;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        rclcpp::Node::SharedPtr node_;
        GridMapParams params_;
        GridMapData data_;

        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_inf_pub_;
        void publishInfMap();

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

        // TF for LiDAR transform
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        Eigen::Isometry3d laser_to_body_;
        bool laser_transform_cached_ = false;

        // Timers
        rclcpp::TimerBase::SharedPtr visualize_timer_;
        rclcpp::TimerBase::SharedPtr fading_timer_;

        // Callbacks
        void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstPtr &msg);
        void cloudCallbackSim(const sensor_msgs::msg::PointCloud2::ConstPtr &msg);
        void lidarCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg);
        void visualizeMapCallback();
        void fadingCallback();

        /** Get base_link pose in map frame at given time (from TF). Returns false if lookup fails. */
        bool getBasePoseAtTime(const rclcpp::Time &time,
                              Eigen::Matrix3d &R_out, Eigen::Vector3d &p_out,
                              const rclcpp::Duration &timeout = rclcpp::Duration::from_seconds(0.05));

        // Core functions
        void moveRingBuffer();
        void raycastProcess();
        void clearAndInflateLocalMap();
        void clearBuffer(char casein, int bound);
        void initMapBoundary();
        inline void changeInfBuf(const bool dir, const int inf_buf_idx, const Eigen::Vector3i global_idx);
        inline int setCacheOccupancy(Eigen::Vector3d pos, int occ, float weight = 1.0f);
        Eigen::Vector3d closestPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt);
        void cacheLaserTransform();
        void extrudeTrunkColumns(const std::vector<Eigen::Vector3d> &hits, int hit_cnt);

        // Helper functions
        inline Eigen::Vector3d globalIdx2Pos(const Eigen::Vector3i &id);
        inline Eigen::Vector3i pos2GlobalIdx(const Eigen::Vector3d &pos);
        inline int globalIdx2BufIdx(const Eigen::Vector3i &id);
        inline int globalIdx2InfBufIdx(const Eigen::Vector3i &id);
        inline Eigen::Vector3i BufIdx2GlobalIdx(size_t address);
        inline Eigen::Vector3i infBufIdx2GlobalIdx(size_t address);
        inline bool isInBuf(const Eigen::Vector3d &pos);
        inline bool isInBuf(const Eigen::Vector3i &idx);
        inline bool isInInfBuf(const Eigen::Vector3d &pos);
        inline bool isInInfBuf(const Eigen::Vector3i &idx);

    public:
        GridMap(const rclcpp::Node::SharedPtr &node);
        ~GridMap() = default;

        void setOdom(const nav_msgs::msg::Odometry::SharedPtr odom);
        inline double getResolution() { return params_.resolution_; };
        inline int getInflateOccupancy(Eigen::Vector3d pos);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    inline int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ, float weight)
    {
        if (occ != 1 && occ != 0)
            return INVALID_IDX;

        Eigen::Vector3i id = pos2GlobalIdx(pos);
        int idx_ctns = globalIdx2BufIdx(id);

        data_.count_hit_and_miss_[idx_ctns] += 1;

        if (data_.count_hit_and_miss_[idx_ctns] == 1)
        {
            data_.cache_voxel_[data_.cache_voxel_cnt_++] = id;
        }

        if (occ == 1)
        {
            data_.count_hit_[idx_ctns] += 1;
            data_.count_weight_[idx_ctns] += weight;
        }

        return idx_ctns;
    }

    inline void GridMap::changeInfBuf(const bool dir, const int inf_buf_idx, const Eigen::Vector3i global_idx)
    {
        int inf_grid = params_.inf_grid_;
        if (dir)
            data_.occupancy_buffer_inflate_[inf_buf_idx] += GRID_MAP_OBS_FLAG;
        else
            data_.occupancy_buffer_inflate_[inf_buf_idx] -= GRID_MAP_OBS_FLAG;

        for (int x_inf = -inf_grid; x_inf <= inf_grid; ++x_inf)
            for (int y_inf = -inf_grid; y_inf <= inf_grid; ++y_inf)
                for (int z_inf = -inf_grid; z_inf <= inf_grid; ++z_inf)
                {
                    Eigen::Vector3i id_inf(global_idx + Eigen::Vector3i(x_inf, y_inf, z_inf));

                    // Skip neighbors that fall outside the current inflation buffer bounds
                    if (!isInInfBuf(id_inf))
                        continue;

                    int id_inf_buf = globalIdx2InfBufIdx(id_inf);
                    if (dir)
                        ++data_.occupancy_buffer_inflate_[id_inf_buf];
                    else
                    {
                        if (data_.occupancy_buffer_inflate_[id_inf_buf] == 0)
                            continue; // Already zero, skip to prevent underflow
                        --data_.occupancy_buffer_inflate_[id_inf_buf];
                    }
                }
    }

    inline int GridMap::globalIdx2BufIdx(const Eigen::Vector3i &id)
    {
        int x_buffer = (id(0) - data_.ringbuffer_origin3i_(0)) % data_.ringbuffer_size3i_(0);
        int y_buffer = (id(1) - data_.ringbuffer_origin3i_(1)) % data_.ringbuffer_size3i_(1);
        int z_buffer = (id(2) - data_.ringbuffer_origin3i_(2)) % data_.ringbuffer_size3i_(2);
        if (x_buffer < 0)
            x_buffer += data_.ringbuffer_size3i_(0);
        if (y_buffer < 0)
            y_buffer += data_.ringbuffer_size3i_(1);
        if (z_buffer < 0)
            z_buffer += data_.ringbuffer_size3i_(2);

        return data_.ringbuffer_size3i_(0) * data_.ringbuffer_size3i_(1) * z_buffer + data_.ringbuffer_size3i_(0) * y_buffer + x_buffer;
    }

    inline int GridMap::globalIdx2InfBufIdx(const Eigen::Vector3i &id)
    {
        int x_buffer = (id(0) - data_.ringbuffer_inf_origin3i_(0)) % data_.ringbuffer_inf_size3i_(0);
        int y_buffer = (id(1) - data_.ringbuffer_inf_origin3i_(1)) % data_.ringbuffer_inf_size3i_(1);
        int z_buffer = (id(2) - data_.ringbuffer_inf_origin3i_(2)) % data_.ringbuffer_inf_size3i_(2);
        if (x_buffer < 0)
            x_buffer += data_.ringbuffer_inf_size3i_(0);
        if (y_buffer < 0)
            y_buffer += data_.ringbuffer_inf_size3i_(1);
        if (z_buffer < 0)
            z_buffer += data_.ringbuffer_inf_size3i_(2);

        return data_.ringbuffer_inf_size3i_(0) * data_.ringbuffer_inf_size3i_(1) * z_buffer + data_.ringbuffer_inf_size3i_(0) * y_buffer + x_buffer;
    }

    inline Eigen::Vector3i GridMap::BufIdx2GlobalIdx(size_t address)
    {

        const int ringbuffer_xysize = data_.ringbuffer_size3i_(0) * data_.ringbuffer_size3i_(1);
        int zid_in_buffer = address / ringbuffer_xysize;
        address %= ringbuffer_xysize;
        int yid_in_buffer = address / data_.ringbuffer_size3i_(0);
        int xid_in_buffer = address % data_.ringbuffer_size3i_(0);

        int xid_global = xid_in_buffer + data_.ringbuffer_origin3i_(0);
        if (xid_global > data_.ringbuffer_upbound3i_(0))
            xid_global -= data_.ringbuffer_size3i_(0);
        int yid_global = yid_in_buffer + data_.ringbuffer_origin3i_(1);
        if (yid_global > data_.ringbuffer_upbound3i_(1))
            yid_global -= data_.ringbuffer_size3i_(1);
        int zid_global = zid_in_buffer + data_.ringbuffer_origin3i_(2);
        if (zid_global > data_.ringbuffer_upbound3i_(2))
            zid_global -= data_.ringbuffer_size3i_(2);

        return Eigen::Vector3i(xid_global, yid_global, zid_global);
    }

    inline Eigen::Vector3i GridMap::infBufIdx2GlobalIdx(size_t address)
    {

        const int ringbuffer_xysize = data_.ringbuffer_inf_size3i_(0) * data_.ringbuffer_inf_size3i_(1);
        int zid_in_buffer = address / ringbuffer_xysize;
        address %= ringbuffer_xysize;
        int yid_in_buffer = address / data_.ringbuffer_inf_size3i_(0);
        int xid_in_buffer = address % data_.ringbuffer_inf_size3i_(0);

        int xid_global = xid_in_buffer + data_.ringbuffer_inf_origin3i_(0);
        if (xid_global > data_.ringbuffer_inf_upbound3i_(0))
            xid_global -= data_.ringbuffer_inf_size3i_(0);
        int yid_global = yid_in_buffer + data_.ringbuffer_inf_origin3i_(1);
        if (yid_global > data_.ringbuffer_inf_upbound3i_(1))
            yid_global -= data_.ringbuffer_inf_size3i_(1);
        int zid_global = zid_in_buffer + data_.ringbuffer_inf_origin3i_(2);
        if (zid_global > data_.ringbuffer_inf_upbound3i_(2))
            zid_global -= data_.ringbuffer_inf_size3i_(2);

        return Eigen::Vector3i(xid_global, yid_global, zid_global);
    }

    inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos)
    {
        if (!isInInfBuf(pos))
            return 0;

        if (params_.enable_virtual_wall_ && (pos(2) >= params_.virtual_ceil_ || pos(2) <= params_.virtual_ground_))
            return -1;

        return int(data_.occupancy_buffer_inflate_[globalIdx2InfBufIdx(pos2GlobalIdx(pos))]);
    }

    inline bool GridMap::isInBuf(const Eigen::Vector3d &pos)
    {
        if (pos(0) < data_.ringbuffer_lowbound3d_(0) || pos(1) < data_.ringbuffer_lowbound3d_(1) || pos(2) < data_.ringbuffer_lowbound3d_(2))
        {
            return false;
        }
        if (pos(0) > data_.ringbuffer_upbound3d_(0) || pos(1) > data_.ringbuffer_upbound3d_(1) || pos(2) > data_.ringbuffer_upbound3d_(2))
        {
            return false;
        }
        return true;
    }

    inline bool GridMap::isInBuf(const Eigen::Vector3i &idx)
    {
        if (idx(0) < data_.ringbuffer_lowbound3i_(0) || idx(1) < data_.ringbuffer_lowbound3i_(1) || idx(2) < data_.ringbuffer_lowbound3i_(2))
        {
            return false;
        }
        if (idx(0) > data_.ringbuffer_upbound3i_(0) || idx(1) > data_.ringbuffer_upbound3i_(1) || idx(2) > data_.ringbuffer_upbound3i_(2))
        {
            return false;
        }
        return true;
    }

    inline bool GridMap::isInInfBuf(const Eigen::Vector3d &pos)
    {
        if (pos(0) < data_.ringbuffer_inf_lowbound3d_(0) || pos(1) < data_.ringbuffer_inf_lowbound3d_(1) || pos(2) < data_.ringbuffer_inf_lowbound3d_(2))
        {
            return false;
        }
        if (pos(0) > data_.ringbuffer_inf_upbound3d_(0) || pos(1) > data_.ringbuffer_inf_upbound3d_(1) || pos(2) > data_.ringbuffer_inf_upbound3d_(2))
        {
            return false;
        }
        return true;
    }

    inline bool GridMap::isInInfBuf(const Eigen::Vector3i &idx)
    {
        if (idx(0) < data_.ringbuffer_inf_lowbound3i_(0) || idx(1) < data_.ringbuffer_inf_lowbound3i_(1) || idx(2) < data_.ringbuffer_inf_lowbound3i_(2))
        {
            return false;
        }
        if (idx(0) > data_.ringbuffer_inf_upbound3i_(0) || idx(1) > data_.ringbuffer_inf_upbound3i_(1) || idx(2) > data_.ringbuffer_inf_upbound3i_(2))
        {
            return false;
        }
        return true;
    }

    inline Eigen::Vector3d GridMap::globalIdx2Pos(const Eigen::Vector3i &id) // t ~ 0us
    {
        return Eigen::Vector3d((id(0) + 0.5) * params_.resolution_, (id(1) + 0.5) * params_.resolution_, (id(2) + 0.5) * params_.resolution_);
    }

    inline Eigen::Vector3i GridMap::pos2GlobalIdx(const Eigen::Vector3d &pos)
    {
        return (pos * params_.resolution_inv_).array().floor().cast<int>(); // more than twice faster than std::floor()
    }
} // namespace ego_planner

#endif // _GRID_MAP_HPP_