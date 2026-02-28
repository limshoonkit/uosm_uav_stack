/*
 * SPDX-FileCopyrightText: University of Southampton Malaysia
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "../include/circle_fitting.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <limits>
#include <iostream>

CircleFitting::CircleFitting(const CircleFittingParams &params) : params_(params)
{
    max_center_distance_sq_ = params_.max_center_distance * params_.max_center_distance;
}

Circle CircleFitting::fitCircle(const Cluster &cluster, CircleFittingMethod method)
{
    Circle result;
    switch (method)
    {
    case CircleFittingMethod::KASA:
        result = fitCircleKasa(cluster);
        break;
    case CircleFittingMethod::HyperLS:
        result = fitCircleHyperLS(cluster);
        break;
    case CircleFittingMethod::PCL_CONSENSUS:
        result = fitCirclePCLConsensus(cluster);
        break;
    default:
        result = fitCircleHyperLS(cluster);
    }

    // Validate the fitted circle
    if (isValidCircle(result, cluster))
    {
        result.quality_score = calculateFitQuality(result, cluster);
        // std::cout << "Fitted circle: center=(" << result.x << ", " << result.y << "), radius=" << result.r
        //           << ", quality_score=" << result.quality_score << std::endl;
        return result;
    }

    return {};
}

Circle CircleFitting::fitCircleKasa(const Cluster &cluster)
{
    const size_t n = cluster.size();

    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd b(n);

    for (size_t i = 0; i < n; ++i)
    {
        const double x = cluster[i].first;
        const double y = cluster[i].second;
        A(i, 0) = x;
        A(i, 1) = y;
        A(i, 2) = 1.0;
        b(i) = x * x + y * y;
    }

    // Solve the linear system A * p = b for p = [2a, 2b, r^2 - a^2 - b^2]
    const Eigen::Vector3d p = A.colPivHouseholderQr().solve(b);

    Circle circle;
    circle.x = p(0) / 2.0;
    circle.y = p(1) / 2.0;
    const double r_sq = p(2) + circle.x * circle.x + circle.y * circle.y;

    if (r_sq < 0)
        return {};
    circle.r = std::sqrt(r_sq);
    return circle;
}

Circle CircleFitting::fitCircleHyperLS(const Cluster &cluster)
{
    // Calculate the mean of the cluster points
    const size_t n = cluster.size();
    Eigen::Vector2d mean;
    mean << std::accumulate(cluster.begin(), cluster.end(), 0.0, [](double sum, const auto &p)
                            { return sum + p.first; }) /
                n,
        std::accumulate(cluster.begin(), cluster.end(), 0.0, [](double sum, const auto &p)
                        { return sum + p.second; }) /
            n;

    // Calculate the covariance matrix A
    Eigen::Matrix2d A;
    A.setZero();
    for (const auto &p : cluster)
    {
        Eigen::Vector2d centered(p.first - mean(0), p.second - mean(1));
        A.noalias() += centered * centered.transpose(); // A = Σ(xi - μ)(xi - μ)^T
    }

    // Calculate the vector B
    Eigen::Vector2d B;
    B.setZero();
    for (const auto &p : cluster)
    {
        Eigen::Vector2d centered(p.first - mean(0), p.second - mean(1));
        B.noalias() += centered * (centered.squaredNorm() + (A.trace() - A.diagonal().sum()) / n); // B = Σ(xi - μ)(||xi - μ||^2 + (tr(A) - Σ(diag(A))) / n)
    }

    // Solve for centered coordinates of the center (u, v)
    const Eigen::Vector2d uc_vc = A.ldlt().solve(B); // (u, v) = A^-1 * B
    const double uc = uc_vc(0);
    const double vc = uc_vc(1);

    Circle circle;
    circle.x = uc + mean(0);                                                      // x = u + μ_x
    circle.y = vc + mean(1);                                                      // y = v + μ_y
    const double r_sq = uc * uc + vc * vc + (A.trace() - A.diagonal().sum()) / n; // r^2 = u^2 + v^2 + (tr(A) - Σ(diag(A))) / n

    if (r_sq < 0)
        return {};
    circle.r = std::sqrt(r_sq);
    return circle;
}

Circle CircleFitting::fitCirclePCLConsensus(const Cluster &cluster)
{
    const size_t n = cluster.size();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->points.resize(n);
    for (size_t i = 0; i < n; ++i)
    {
        cloud->points[i].x = static_cast<float>(cluster[i].first);
        cloud->points[i].y = static_cast<float>(cluster[i].second);
        cloud->points[i].z = 0; // 2D problem
    }

    // Create the Sample Consensus model and RANSAC objects
    pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr model =
        std::make_shared<pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>>(cloud);
    model->setRadiusLimits(params_.min_radius, params_.max_radius);

    pcl::RandomSampleConsensus<pcl::PointXYZ> sac(model, params_.ransac_distance_threshold);

    // Run the RANSAC computation
    if (!sac.computeModel())
    {
        return {}; // Failed to find a model
    }

    // Extract the circle parameters
    Eigen::VectorXf coefficients;
    sac.getModelCoefficients(coefficients);

    if (coefficients.size() != 3 || coefficients[2] <= params_.min_radius || coefficients[2] >= params_.max_radius)
    {
        return {};
    }

    Circle circle;
    circle.x = coefficients[0];
    circle.y = coefficients[1];
    circle.r = coefficients[2];

    return circle;
}

bool CircleFitting::isValidCircle(const Circle &circle, const Cluster &cluster)
{
    if (circle.r < params_.min_radius || circle.r > params_.max_radius)
        return false;

    if (cluster.empty())
        return false;

    auto sum = std::accumulate(cluster.begin(), cluster.end(), std::make_pair(0.0, 0.0),
                               [](const std::pair<double, double> &acc, const std::pair<double, double> &p) -> std::pair<double, double>
                               {
                                   return {acc.first + p.first, acc.second + p.second};
                               });

    const double dx = circle.x - sum.first / cluster.size();
    const double dy = circle.y - sum.second / cluster.size();

    return (dx * dx + dy * dy) <= max_center_distance_sq_;
}

double CircleFitting::calculateFitQuality(const Circle &circle, const Cluster &cluster)
{
    if (cluster.empty() || circle.r <= 0)
        return 0.0;

    double sum_sq_residuals = 0.0;
    for (const auto &point : cluster)
    {
        const double dx = point.first - circle.x;
        const double dy = point.second - circle.y;
        const double dist_sq = dx * dx + dy * dy;
        const double r_sq = circle.r * circle.r;
        const double residual_sq = dist_sq > r_sq ? dist_sq - r_sq : r_sq - dist_sq;
        sum_sq_residuals += residual_sq;
    }

    // Quality score is inversely proportional to mean squared error. 1 is best, 0 is worst.
    return 1.0 / (1.0 + sum_sq_residuals / cluster.size());
}
