#ifndef CIRCLE_FITTING_HPP_
#define CIRCLE_FITTING_HPP_

#include <vector>
#include <utility>
#include <random>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>

// Type aliases for consistency
using Point = std::pair<double, double>;
using Cluster = std::vector<Point>;

struct Circle
{
    double x = 0.0;
    double y = 0.0;
    double r = 0.0;
    double quality_score = 0.0;
};

struct CircleFittingParams
{
    double min_radius = 0.25;                // minimum radius in meters
    double max_radius = 0.45;                // maximum radius in meters
    double max_center_distance = 0.5;        // Maximum distance from cluster centroid to circle center
    double ransac_distance_threshold = 0.05; // RANSAC inlier distance threshold in meters
};

enum class CircleFittingMethod
{
    KASA = 0,
    HyperLS = 1,
    PCL_CONSENSUS = 2 // RANSAC
};

class CircleFitting
{
public:
    explicit CircleFitting(const CircleFittingParams &params);
    ~CircleFitting() = default;

    // Main fitting interface
    Circle fitCircle(const Cluster &cluster, CircleFittingMethod method);

    Circle fitCircleKasa(const Cluster &cluster);
    Circle fitCircleHyperLS(const Cluster &cluster);
    Circle fitCirclePCLConsensus(const Cluster &cluster);

private:
    CircleFittingParams params_;
    double max_center_distance_sq_;

    // Helper functions
    bool isValidCircle(const Circle &circle, const Cluster &cluster);
    double calculateFitQuality(const Circle &circle, const Cluster &cluster);
};

#endif // CIRCLE_FITTING_HPP_