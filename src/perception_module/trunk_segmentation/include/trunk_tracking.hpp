/*
 * SPDX-FileCopyrightText: University of Southampton Malaysia
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef TRUNK_TRACKING_HPP_
#define TRUNK_TRACKING_HPP_

#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "circle_fitting.hpp"
#include "tracked_trunk.hpp"
#include "nanoflann.hpp"

struct TrunkTrackAdapter
{
    const std::vector<TrackedTrunk *> &tracks;

    explicit TrunkTrackAdapter(const std::vector<TrackedTrunk *> &track_vec) : tracks(track_vec) {}

    inline size_t kdtree_get_point_count() const { return tracks.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        return tracks[idx]->position_map[dim];
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX &) const { return false; }
};

struct TrackingParams
{
    double association_threshold = 0.7;
    double distance_threshold = 0.5;
    double gating_threshold = 5.99;
    double min_hit_rate_for_confirmation = 0.3;
    size_t min_observations_for_confirmation = 5;
    double max_miss_rate_for_lost = 0.8;
    size_t min_observations_for_lost = 8;
    size_t detection_window_size = 10;
    double process_noise_std = 0.1; 
    double measurement_noise_std = 0.2;
    double radius_tolerance = 0.1;
    double radius_penalty_weight = 1.0;
    int max_lost_tracks = 50;
    double max_lost_track_age_seconds = 300.0;
    double lost_track_association_threshold = 1.0;
    double covariance_inflation_factor = 2.0;
};

class TrunkTracking
{
public:
    explicit TrunkTracking(const TrackingParams &params);
    ~TrunkTracking() = default;

    // Main tracking update
    void updateTracks(const std::vector<Circle> &detections,
                      const geometry_msgs::msg::Pose &robot_pose,
                      const rclcpp::Time &timestamp);

    // Get current active tracks
    const std::unordered_map<uint16_t, TrackedTrunk> &getTrackedTrunks() const
    {
        return active_tracks_;
    }

    // Get lost tracks
    const std::unordered_map<uint16_t, TrackedTrunk> &getLostTracks() const
    {
        return lost_tracks_;
    }

    // Get total number of tracks managed
    size_t getTotalTrackCount() const
    {
        return active_tracks_.size() + lost_tracks_.size();
    }

private:
    TrackingParams params_;

    std::unordered_map<uint16_t, TrackedTrunk> active_tracks_;
    std::unordered_map<uint16_t, TrackedTrunk> lost_tracks_;

    uint16_t next_trunk_id_ = 1;
    double process_noise_std_sq;
    double search_radius_sq;
    double measurement_noise_std_sq;
    double min_distance_sq;

    std::vector<uint16_t> reference_trunk_ids_;
    std::unordered_map<uint16_t, Eigen::Vector2d> trunk_map_;

    struct AssociationResult
    {
        std::vector<std::pair<uint16_t, size_t>> active_associations;
        std::vector<std::pair<uint16_t, size_t>> lost_associations;
        std::vector<size_t> unassociated_detections;
    };

    // Core tracking steps
    AssociationResult associateDetections(
        const std::vector<Circle> &detections,
        const std::vector<Eigen::Vector2d> &detections_map);

    std::vector<std::pair<uint16_t, size_t>> associateWithTracks(
        const std::vector<TrackedTrunk *> &tracks,
        const std::vector<Circle> &detections,
        const std::vector<Eigen::Vector2d> &detections_map,
        bool is_lost_track);
    void updateTrack(TrackedTrunk &track,
                     const Eigen::Vector2d &measurement,
                     const Eigen::Matrix2d &measurement_cov);

    // Track management
    void moveLostTracksToActive(const std::vector<std::pair<uint16_t, size_t>> &lost_associations,
                                const std::vector<Circle> &detections,
                                const std::vector<Eigen::Vector2d> &detections_map,
                                const rclcpp::Time &current_time);

    void manageLostTracks(const rclcpp::Time &current_time);
    void pruneLostTracks(const rclcpp::Time &current_time);
    void predictTracks(double dt);
    void manageTrackStates(const rclcpp::Time &current_time);
    void createNewTracks(const std::vector<Circle> &unassociated_detections,
                         const geometry_msgs::msg::Pose &robot_pose,
                         const rclcpp::Time &current_time);

    // Helper functions
    double calculateAssociationCost(const Eigen::Vector2d &detection_pos,
                                    double detection_radius,
                                    const TrackedTrunk &track,
                                    bool is_lost_track);
    Eigen::Vector2d transformToMapFrame(const Eigen::Vector2d &point,
                                        const geometry_msgs::msg::Pose &robot_pose);
    double mahalanobisDistance(const Eigen::Vector2d &detection,
                               const TrackedTrunk &track);

    // Lost track utility functions
    bool isLostTrackExpired(const TrackedTrunk &track, const rclcpp::Time &current_time) const;
    void inflateCovarianceForLostTrack(TrackedTrunk &track);
    std::vector<uint16_t> selectTracksToRemove(size_t num_to_remove);
};

#endif // TRUNK_TRACKING_HPP_