/*
 * SPDX-FileCopyrightText: University of Southampton Malaysia
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "../include/clustering_algorithms.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <random>
#include <numeric>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <iostream>

ClusteringAlgorithms::ClusteringAlgorithms(const ClusteringParams &params) : params_(params)
{
    eps_density_sq = params_.eps_density * params_.eps_density;
    eps_clustering_sq = params_.eps_clustering * params_.eps_clustering;
}

std::vector<Cluster> ClusteringAlgorithms::clusterPoints(const std::vector<Point> &points, ClusteringMethod method)
{
    switch (method)
    {
    case ClusteringMethod::PCL_EUCLIDEAN:
        return pclEuclideanClustering(points);
    case ClusteringMethod::DBSCAN_PLUS:
        return dbscanPlusClustering(points);
    default:
        return pclEuclideanClustering(points);
    }
}

std::vector<Point> ClusteringAlgorithms::laserScanToPoints(const sensor_msgs::msg::LaserScan &scan,
                                                           const Eigen::Isometry3d &transform)
{
    std::vector<Point> points;
    if (scan.ranges.empty())
    {
        return points;
    }

    // Precompute unit vectors
    if (precomputed_unit_vectors_.size() != scan.ranges.size())
    {
        precomputed_unit_vectors_.resize(scan.ranges.size());
        const double angle_min = static_cast<double>(scan.angle_min);
        const double angle_increment = static_cast<double>(scan.angle_increment);
        const Eigen::Matrix3d R = transform.rotation();

        for (size_t i = 0; i < scan.ranges.size(); ++i)
        {
            const double angle = angle_min + i * angle_increment;
            const double cos_angle = std::cos(angle);
            const double sin_angle = std::sin(angle);

            precomputed_unit_vectors_[i] = {
                R(0, 0) * cos_angle + R(0, 1) * sin_angle,
                R(1, 0) * cos_angle + R(1, 1) * sin_angle};
        }
    }

    const Eigen::Vector3d translation = transform.translation();
    const double tx = translation.x();
    const double ty = translation.y();
    const float range_thresh_min = scan.range_min + 0.5f;
    const float max_range = params_.submap_search_radius;
    const float *ranges_ptr = scan.ranges.data();
    const size_t ranges_size = scan.ranges.size();

    size_t valid_count = 0;
    for (size_t i = 0; i < ranges_size; ++i)
    {
        const float range = ranges_ptr[i];
        if (range >= range_thresh_min && range <= max_range)
        {
            ++valid_count;
        }
    }

    points.reserve(valid_count);
    for (size_t i = 0; i < ranges_size; ++i)
    {
        const float range = ranges_ptr[i];

        if (range >= range_thresh_min && range <= max_range)
        {
            const auto &unit_vec = precomputed_unit_vectors_[i];
            points.emplace_back(
                range * unit_vec.x() + tx,
                range * unit_vec.y() + ty);
        }
    }

    return points;
}

std::vector<size_t> ClusteringAlgorithms::k_centers_sampling(const std::vector<Point> &points, size_t m)
{
    if (m >= points.size())
    {
        std::vector<size_t> all_indices(points.size());
        std::iota(all_indices.begin(), all_indices.end(), 0);
        return all_indices;
    }

    std::vector<size_t> sampled_indices;
    sampled_indices.reserve(m);

    std::vector<double> min_dist_sq(points.size(), std::numeric_limits<double>::max());
    std::vector<bool> is_sampled(points.size(), false);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> distrib(0, points.size() - 1);

    size_t first_idx = distrib(gen);
    sampled_indices.push_back(first_idx);
    is_sampled[first_idx] = true;

    for (size_t i = 0; i < points.size(); ++i)
    {
        double dx = points[i].first - points[first_idx].first;
        double dy = points[i].second - points[first_idx].second;
        min_dist_sq[i] = dx * dx + dy * dy;
    }

    for (size_t i = 1; i < m; ++i)
    {
        size_t next_idx = std::distance(min_dist_sq.begin(), std::max_element(min_dist_sq.begin(), min_dist_sq.end()));
        sampled_indices.push_back(next_idx);
        is_sampled[next_idx] = true;
        min_dist_sq[next_idx] = 0.0; // Mark as selected

        const auto &new_center = points[next_idx];
        for (size_t j = 0; j < points.size(); ++j)
        {
            if (min_dist_sq[j] > 0)
            { // No need to update points already selected
                double dx = points[j].first - new_center.first;
                double dy = points[j].second - new_center.second;
                min_dist_sq[j] = std::min(min_dist_sq[j], dx * dx + dy * dy);
            }
        }
    }
    return sampled_indices;
}

std::vector<Cluster> ClusteringAlgorithms::dbscanPlusClustering(const std::vector<Point> &points)
{
    std::vector<Cluster> clusters;
    if (points.empty())
        return clusters;

    std::vector<int> labels = dbscanPlusImpl(points);

    // Group points by cluster label
    std::map<int, Cluster> cluster_map;
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (labels[i] >= 0) // Ignore noise points (label -1)
        {
            cluster_map[labels[i]].push_back(points[i]);
        }
    }

    // Convert map to vector
    clusters.reserve(cluster_map.size());
    for (auto &pair : cluster_map)
    {
        if (pair.second.size() >= static_cast<size_t>(params_.dbscan_min_pts))
        {
            clusters.push_back(std::move(pair.second));
        }
    }

    return clusters;
}

std::vector<int> ClusteringAlgorithms::dbscanPlusImpl(const std::vector<Point> &points)
{
    const size_t n = points.size();
    if (n == 0)
        return {};

    std::vector<int> labels(n, -1); // default noise

    // compute sample size m
    constexpr size_t dimension = 2U;
    double exponent = static_cast<double>(dimension) / (static_cast<double>(dimension) + 4.0);
    size_t computed_m = static_cast<size_t>(std::round(params_.p * std::pow(static_cast<double>(n), exponent)));

    // ensure at least dbscan_min_pts and at most n
    size_t m = std::min(n, std::max(static_cast<size_t>(params_.dbscan_min_pts), computed_m));
    if (m < static_cast<size_t>(params_.dbscan_min_pts) || m == 0)
        return labels; // can't form core points

    // std::cout << "DBSCAN++: m = " << m << ", n = " << n << std::endl;
    std::vector<size_t> sampled_indices = k_centers_sampling(points, m);
    if (sampled_indices.empty())
        return labels;
    // std::cout << "DBSCAN++: sampled_indices.size() = " << sampled_indices.size() << std::endl;
    // Build KDTree on all points
    PointAdaptor all_points_adaptor(points);
    KDTree all_points_kdtree(dimension, all_points_adaptor, {10});
    all_points_kdtree.buildIndex();
    // std::cout << "DBSCAN++: KDTree built with " << n << " points." << std::endl;
    // Identify sampled core points by checking k-NN distance of sampled point
    std::vector<size_t> core_point_indices;
    for (size_t idx : sampled_indices)
    {
        std::vector<uint16_t> indices(params_.dbscan_min_pts);
        std::vector<double> distances(params_.dbscan_min_pts);

        size_t num_found = all_points_kdtree.knnSearch(
            &points[idx].first,
            params_.dbscan_min_pts,
            indices.data(),
            distances.data());

        if (num_found == static_cast<size_t>(params_.dbscan_min_pts) &&
            distances[params_.dbscan_min_pts - 1] <= eps_density_sq)
        {
            core_point_indices.push_back(idx);
        }
    }

    if (core_point_indices.empty())
        return labels; // no core points found
    // std::cout << "DBSCAN++: core_point_indices.size() = " << core_point_indices.size() << std::endl;
    // Create lookup structures for core points
    std::unordered_set<size_t> core_point_set(core_point_indices.begin(), core_point_indices.end());
    std::unordered_map<size_t, size_t> orig_to_local; // map original index to local core index
    for (size_t i = 0; i < core_point_indices.size(); ++i)
    {
        orig_to_local[core_point_indices[i]] = i;
    }

    // Cluster core points
    std::vector<int> core_labels(core_point_indices.size(), -2); // -2 = unvisited
    int cluster_id = 0;

    for (size_t i = 0; i < core_point_indices.size(); ++i)
    {
        if (core_labels[i] != -2)
            continue;

        core_labels[i] = cluster_id;
        std::queue<size_t> q;
        q.push(core_point_indices[i]); // use original index

        while (!q.empty())
        {
            size_t cur_orig_idx = q.front();
            q.pop();

            // Find neighbors in all points, but only consider core points
            std::vector<nanoflann::ResultItem<uint16_t, double>> neighbors;
            nanoflann::RadiusResultSet<double, uint16_t> result_set(eps_clustering_sq, neighbors);
            all_points_kdtree.findNeighbors(result_set, &points[cur_orig_idx].first, {});

            for (const auto &r : neighbors)
            {
                size_t nbr_orig_idx = static_cast<size_t>(r.first);
                // Only process if it's a core point
                if (core_point_set.count(nbr_orig_idx))
                {
                    size_t nbr_local_idx = orig_to_local[nbr_orig_idx];
                    if (core_labels[nbr_local_idx] == -2)
                    {
                        core_labels[nbr_local_idx] = cluster_id;
                        q.push(nbr_orig_idx);
                    }
                }
            }
        }
        ++cluster_id;
    }
    // std::cout << "DBSCAN++: Found " << cluster_id << " clusters in core points." << std::endl;
    // Assign every point to nearest core point's cluster
    for (size_t i = 0; i < n; ++i)
    {
        double min_dist_sq = std::numeric_limits<double>::max();
        int best_cluster = -1;

        // Check distance to all core points
        for (size_t j = 0; j < core_point_indices.size(); ++j)
        {
            size_t core_orig_idx = core_point_indices[j];
            double dx = points[i].first - points[core_orig_idx].first;
            double dy = points[i].second - points[core_orig_idx].second;
            double dist_sq = dx * dx + dy * dy;

            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                best_cluster = core_labels[j];
            }
        }

        // Assign to cluster if within density radius
        if (min_dist_sq <= eps_density_sq && best_cluster >= 0)
        {
            labels[i] = best_cluster;
        }
        else
        {
            labels[i] = -1; // noise
        }
    }

    return labels;
}

std::vector<Cluster> ClusteringAlgorithms::pclEuclideanClustering(const std::vector<Point> &points)
{
    std::vector<Cluster> clusters;
    if (points.empty())
        return clusters;

    // Convert points to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pointsToPointCloud(points);

    // Create KdTree for search
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    tree->setInputCloud(cloud);

    // Perform Euclidean clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(params_.pcl_cluster_tolerance);
    ec.setMinClusterSize(params_.pcl_min_cluster_size);
    ec.setMaxClusterSize(params_.pcl_max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Convert back to our cluster format
    return pointCloudIndicesToClusters(points, cluster_indices);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ClusteringAlgorithms::pointsToPointCloud(const std::vector<Point> &points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->points.reserve(points.size());

    for (const auto &point : points)
    {
        pcl::PointXYZ pcl_point;
        pcl_point.x = point.first;
        pcl_point.y = point.second;
        pcl_point.z = 0.0; // Assume 2D points at z=0
        cloud->points.push_back(pcl_point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;
}

std::vector<Cluster> ClusteringAlgorithms::pointCloudIndicesToClusters(const std::vector<Point> &points,
                                                                       const std::vector<pcl::PointIndices> &cluster_indices)
{
    std::vector<Cluster> clusters;
    clusters.reserve(cluster_indices.size());

    for (const auto &indices : cluster_indices)
    {
        Cluster cluster;
        cluster.reserve(indices.indices.size());

        for (int idx : indices.indices)
        {
            if (idx >= 0 && idx < static_cast<int>(points.size()))
            {
                cluster.push_back(points[idx]);
            }
        }

        if (!cluster.empty())
        {
            clusters.push_back(std::move(cluster));
        }
    }

    return clusters;
}
