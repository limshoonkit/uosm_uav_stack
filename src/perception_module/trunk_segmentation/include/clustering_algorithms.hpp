/*
 * SPDX-FileCopyrightText: University of Southampton Malaysia
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef CLUSTERING_ALGORITHMS_HPP_
#define CLUSTERING_ALGORITHMS_HPP_

#include <vector>
#include <memory>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>

#include "nanoflann.hpp"

// Type aliases
using Point = std::pair<double, double>;
using Cluster = std::vector<Point>;

struct PointCloudAdaptor
{
    const std::vector<Point> &obj;

    PointCloudAdaptor(const std::vector<Point> &obj_) : obj(obj_) {}

    inline size_t kdtree_get_point_count() const { return obj.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0)
            return obj[idx].first;
        else
            return obj[idx].second;
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX & /* bb */) const { return false; }
};

using PointAdaptor = PointCloudAdaptor;
using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointAdaptor>,
    PointAdaptor,
    2 /* dimensionality */,
    uint16_t /* index type */>;

struct ClusteringParams
{
    double submap_search_radius = 10.0; // Maximum range for considering points

    // DBSCAN++ specific parameters
    double p = 0.1;              // The sample fraction, which determines m, the number of points to sample
    double eps_density = 2.0;    // Radius for determining core points;
    double eps_clustering = 2.0; // Radius for determining neighbors and edges in the density graph
    int dbscan_min_pts = 10;     // Minimum points for DBSCAN core point

    // PCL Euclidean clustering parameters
    double pcl_cluster_tolerance = 0.3; // Cluster tolerance for PCL
    int pcl_min_cluster_size = 3;       // Minimum cluster size for PCL
    int pcl_max_cluster_size = 20;      // Maximum cluster size for PCL
};

enum class ClusteringMethod
{
    PCL_EUCLIDEAN = 0, // PCL Euclidean clustering
    DBSCAN_PLUS = 1,   // DBSCAN++ algorithm
};

class ClusteringAlgorithms
{
public:
    explicit ClusteringAlgorithms(const ClusteringParams &params);
    ~ClusteringAlgorithms() = default;

    // Main clustering interface
    std::vector<Cluster> clusterPoints(const std::vector<Point> &points, ClusteringMethod method);

    std::vector<Cluster> pclEuclideanClustering(const std::vector<Point> &points);
    std::vector<Cluster> dbscanPlusClustering(const std::vector<Point> &points);

    // Convert laser scan to points
    std::vector<Point> laserScanToPoints(const sensor_msgs::msg::LaserScan &scan,
                                         const Eigen::Isometry3d &transform);

private:
    ClusteringParams params_;
    double eps_density_sq;
    double eps_clustering_sq;
    std::vector<Eigen::Vector2d> precomputed_unit_vectors_; 

    // DBSCAN++ implementation helpers
    std::vector<int> dbscanPlusImpl(const std::vector<Point> &points);
    std::vector<size_t> k_centers_sampling(const std::vector<Point>& points, size_t m);


    // PCL conversion helpers
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointsToPointCloud(const std::vector<Point> &points);
    std::vector<Cluster> pointCloudIndicesToClusters(const std::vector<Point> &points,
                                                     const std::vector<pcl::PointIndices> &cluster_indices);
};

#endif // CLUSTERING_ALGORITHMS_HPP_