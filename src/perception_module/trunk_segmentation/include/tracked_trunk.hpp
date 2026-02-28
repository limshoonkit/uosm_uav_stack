/*
 * SPDX-FileCopyrightText: University of Southampton Malaysia
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef TRACKED_TRUNK_HPP_
#define TRACKED_TRUNK_HPP_

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <vector>
#include <array>
#include <deque>
#include <rclcpp/time.hpp>

enum TrackState
{
    TENTATIVE, // Recent detection
    CONFIRMED, // Solid track
    LOST       // Track lost but may recover
};

class TrackedTrunk
{
public:
    TrackedTrunk(uint16_t id_, const Eigen::Vector2d &position_map_, double radius_)
        : id(id_),
          position_map(position_map_),
          covariance(Eigen::Matrix2d::Identity() * 0.25),
          radius(radius_),
          state(TENTATIVE),
          total_observations(1),
          recent_radii_buffer(),
          radii_index(0),
          radii_count(0),
          detection_window_size(10)
    {
        addRadius(radius_);
        // First blood is a hit
        detection_window.push_back(true);
    }

    ~TrackedTrunk() = default;

    void addRadius(double r)
    {
        recent_radii_buffer[radii_index] = r;
        radii_index = (radii_index + 1) % 10;
        if (radii_count < 10)
            radii_count++;
    }

    double getAverageRadius() const
    {
        if (radii_count == 0)
            return radius;
        double sum = 0.0;
        for (size_t i = 0; i < radii_count; ++i)
        {
            sum += recent_radii_buffer[i];
        }
        return sum / radii_count;
    }

    void addDetectionResult(bool detected)
    {
        detection_window.push_back(detected);
        if (detection_window.size() > detection_window_size)
        {
            detection_window.pop_front();
        }
        total_observations++;
    }

    double getHitRate() const
    {
        if (detection_window.empty())
            return 0.0;

        int hits = 0;
        for (bool detected : detection_window)
        {
            if (detected)
                hits++;
        }
        return static_cast<double>(hits) / detection_window.size();
    }

    double getMissRate() const
    {
        return 1.0 - getHitRate();
    }

    bool shouldConfirm(double min_hit_rate, size_t min_observations) const
    {
        return detection_window.size() >= min_observations && getHitRate() >= min_hit_rate;
    }

    bool shouldMarkLost(double max_miss_rate, size_t min_observations) const
    {
        return detection_window.size() >= min_observations && getMissRate() >= max_miss_rate;
    }

    size_t getWindowSize() const
    {
        return detection_window.size();
    }

    void resetDetectionWindow()
    {
        detection_window.clear();
    }

    double calculateQualityScore() const
    {
        double hit_rate = detection_window.empty() ? 0.5 : getHitRate();
        double observation_weight = std::min(1.0, total_observations / 20.0);
        return hit_rate * observation_weight;
    }

    uint16_t id;
    Eigen::Vector2d position_map;
    Eigen::Matrix2d covariance;
    double radius;
    rclcpp::Time last_seen;
    rclcpp::Time created_time;
    rclcpp::Time lost_time;
    TrackState state;
    size_t total_observations;
    std::array<double, 10> recent_radii_buffer;
    size_t radii_index = 0;
    size_t radii_count = 0;

private:
    std::deque<bool> detection_window;
    const size_t detection_window_size;
};

#endif // TRACKED_TRUNK_HPP_