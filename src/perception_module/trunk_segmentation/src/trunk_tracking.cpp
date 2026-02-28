/*
 * SPDX-FileCopyrightText: University of Southampton Malaysia
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "../include/trunk_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <vector>

TrunkTracking::TrunkTracking(const TrackingParams &params) : params_(params)
{
    process_noise_std_sq = params_.process_noise_std * params_.process_noise_std;
    measurement_noise_std_sq = params_.measurement_noise_std * params_.measurement_noise_std;
    search_radius_sq = params_.association_threshold * params_.association_threshold;
    min_distance_sq = params_.distance_threshold * params_.distance_threshold;
}

void TrunkTracking::updateTracks(const std::vector<Circle> &detections,
                                 const geometry_msgs::msg::Pose &robot_pose,
                                 const rclcpp::Time &timestamp)
{
    // Predict all existing active tracks
    static rclcpp::Time last_update = timestamp;
    double dt = (timestamp - last_update).seconds();
    if (dt > 0 && dt < 1.0) // TODO: Parameterize dt? According to odom (velocity)?
    {
        predictTracks(dt);
    }
    last_update = timestamp;

    std::vector<Eigen::Vector2d> detections_map;
    detections_map.reserve(detections.size());
    for (const auto &d : detections)
    {
        detections_map.push_back(transformToMapFrame(Eigen::Vector2d(d.x, d.y), robot_pose));
    }

    auto association_result = associateDetections(detections, detections_map);
    std::vector<bool> detection_used(detections.size(), false);

    // Update active tracks with associations
    for (const auto &[track_id, detection_idx] : association_result.active_associations)
    {
        if (detection_idx < detections.size())
        {
            const auto &detection = detections[detection_idx];
            Eigen::Vector2d measurement_map = detections_map[detection_idx];
            Eigen::Matrix2d measurement_cov = Eigen::Matrix2d::Identity() * measurement_noise_std_sq;

            auto it = active_tracks_.find(track_id);
            if (it != active_tracks_.end())
            {
                updateTrack(it->second, measurement_map, measurement_cov);
                it->second.addDetectionResult(true); // Hit
                it->second.last_seen = timestamp;
                it->second.addRadius(detection.r);
                detection_used[detection_idx] = true;
            }
        }
    }

    // Resurrect lost tracks
    moveLostTracksToActive(association_result.lost_associations, detections, detections_map, timestamp);
    for (const auto &[track_id, detection_idx] : association_result.lost_associations)
    {
        detection_used[detection_idx] = true;
    }

    // Handle unassociated active tracks
    for (auto &[id, track] : active_tracks_)
    {
        bool was_updated = std::any_of(association_result.active_associations.begin(),
                                       association_result.active_associations.end(),
                                       [id](const auto &assoc)
                                       { return assoc.first == id; });
        if (!was_updated)
        {
            track.addDetectionResult(false); // Miss
        }
    }

    // Create new tracks from unassociated detections
    std::vector<Circle> unassociated;
    unassociated.reserve(detections.size());
    for (size_t i = 0; i < detections.size(); ++i)
    {
        if (!detection_used[i])
            unassociated.push_back(detections[i]);
    }
    createNewTracks(unassociated, robot_pose, timestamp);
    manageTrackStates(timestamp);
    manageLostTracks(timestamp);
}

TrunkTracking::AssociationResult TrunkTracking::associateDetections(
    const std::vector<Circle> &detections,
    const std::vector<Eigen::Vector2d> &detections_map)
{
    AssociationResult result;

    if (detections.empty())
        return result;

    // First, try to associate with active tracks
    std::vector<TrackedTrunk *> active_track_ptrs;
    active_track_ptrs.reserve(active_tracks_.size());
    for (auto &[id, track] : active_tracks_)
    {
        if (track.state != TrackState::LOST)
        {
            active_track_ptrs.push_back(&track);
        }
    }

    if (!active_track_ptrs.empty())
    {
        result.active_associations = associateWithTracks(active_track_ptrs, detections,
                                                         detections_map, false);
    }

    // Then, try to associate remaining detections with lost tracks
    std::vector<TrackedTrunk *> lost_track_ptrs;
    lost_track_ptrs.reserve(lost_tracks_.size());
    for (auto &[id, track] : lost_tracks_)
    {
        lost_track_ptrs.push_back(&track);
    }

    if (!lost_track_ptrs.empty())
    {
        result.lost_associations = associateWithTracks(lost_track_ptrs, detections,
                                                       detections_map, true);
    }

    // Identify unassociated detections
    std::vector<bool> detection_used(detections.size(), false);
    for (const auto &[track_id, detection_idx] : result.active_associations)
    {
        detection_used[detection_idx] = true;
    }
    for (const auto &[track_id, detection_idx] : result.lost_associations)
    {
        detection_used[detection_idx] = true;
    }
    for (size_t i = 0; i < detection_used.size(); ++i)
    {
        if (!detection_used[i])
        {
            result.unassociated_detections.push_back(i);
        }
    }

    return result;
}

std::vector<std::pair<uint16_t, size_t>> TrunkTracking::associateWithTracks(
    const std::vector<TrackedTrunk *> &tracks,
    const std::vector<Circle> &detections,
    const std::vector<Eigen::Vector2d> &detections_map,
    bool is_lost_track)
{
    std::vector<std::pair<uint16_t, size_t>> associations;

    if (tracks.empty() || detections.empty())
        return associations;

    using TrunkKDTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, TrunkTrackAdapter>,
        TrunkTrackAdapter, 2>;

    TrunkTrackAdapter adapter(tracks);
    TrunkKDTree kdtree(2, adapter, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    kdtree.buildIndex();

    struct Candidate
    {
        double cost;
        uint16_t track_id;
        size_t det_idx;
    };

    std::vector<Candidate> candidates;
    candidates.reserve(detections.size() * tracks.size());

    // Use different thresholds for lost tracks
    double association_threshold = is_lost_track ? params_.lost_track_association_threshold : params_.association_threshold;
    double search_radius = association_threshold * association_threshold;

    std::vector<nanoflann::ResultItem<uint16_t, double>> matches;
    matches.reserve(tracks.size());

    for (size_t di = 0; di < detections.size(); ++di)
    {
        matches.clear();
        nanoflann::RadiusResultSet<double, uint16_t> result_set(search_radius, matches);
        kdtree.findNeighbors(result_set, &detections_map[di][0], {});

        for (const auto &match : matches)
        {
            TrackedTrunk *track = tracks[match.first];
            const auto &detection_circle = detections[di];
            const auto &detection_map_pos = detections_map[di];
            double cost = calculateAssociationCost(detection_map_pos, detection_circle.r, *track, is_lost_track);
            if (cost < std::numeric_limits<double>::infinity())
            {
                candidates.push_back({cost, track->id, di});
            }
        }
    }

    // Sort by cost and perform assignment
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate &a, const Candidate &b)
              { return a.cost < b.cost; });

    std::vector<bool> assigned_detections(detections.size(), false);
    std::unordered_set<uint16_t> assigned_tracks;
    assigned_tracks.reserve(tracks.size());

    for (const auto &c : candidates)
    {
        if (assigned_tracks.count(c.track_id) || assigned_detections[c.det_idx])
            continue;

        associations.emplace_back(c.track_id, c.det_idx);
        assigned_tracks.insert(c.track_id);
        assigned_detections[c.det_idx] = true;
    }

    return associations;
}

double TrunkTracking::calculateAssociationCost(const Eigen::Vector2d &detection_pos,
                                               double detection_radius,
                                               const TrackedTrunk &track,
                                               bool is_lost_track)
{
    // Radius check
    double radius_tolerance = is_lost_track ? params_.radius_tolerance * 2.0 : params_.radius_tolerance;
    double radius_diff = std::abs(detection_radius - track.radius);
    if (radius_diff > radius_tolerance)
        return std::numeric_limits<double>::infinity();

    // Mahalanobis distance gating
    double gating_threshold = is_lost_track ? params_.gating_threshold * 2.0 : params_.gating_threshold;
    double m2 = mahalanobisDistance(detection_pos, track);
    if (m2 > gating_threshold)
        return std::numeric_limits<double>::infinity();

    // Calculate cost
    const double lambda = params_.radius_penalty_weight;
    const double radius_norm = radius_tolerance > 0.0 ? radius_tolerance : 1e-6;
    double rterm = (radius_diff / radius_norm);
    double cost = m2 + lambda * rterm * rterm;

    // Add penalty for lost tracks to prefer active tracks
    if (is_lost_track)
        cost += 1.0;

    return cost;
}

void TrunkTracking::moveLostTracksToActive(const std::vector<std::pair<uint16_t, size_t>> &lost_associations,
                                           const std::vector<Circle> &detections,
                                           const std::vector<Eigen::Vector2d> &detections_map,
                                           const rclcpp::Time &current_time)
{
    for (const auto &[track_id, detection_idx] : lost_associations)
    {
        auto lost_it = lost_tracks_.find(track_id);
        if (lost_it == lost_tracks_.end() || detection_idx >= detections.size())
            continue;

        TrackedTrunk track = std::move(lost_it->second);
        lost_tracks_.erase(lost_it);

        track.state = TrackState::CONFIRMED;
        track.last_seen = current_time;
        track.resetDetectionWindow();   // Clear old history
        track.addDetectionResult(true); // Add current detection as hit

        const auto &detection = detections[detection_idx];
        Eigen::Vector2d measurement_map = detections_map[detection_idx];
        Eigen::Matrix2d measurement_cov = Eigen::Matrix2d::Identity() * measurement_noise_std_sq;

        updateTrack(track, measurement_map, measurement_cov);
        track.addRadius(detection.r);
        active_tracks_.emplace(track_id, std::move(track));

        // std::cout << "Resurrected lost track " << track_id << " back to active" << std::endl;
    }
}

void TrunkTracking::manageLostTracks(const rclcpp::Time &current_time)
{
    pruneLostTracks(current_time);
    if (lost_tracks_.size() > static_cast<size_t>(params_.max_lost_tracks))
    {
        size_t num_to_remove = lost_tracks_.size() - params_.max_lost_tracks;
        auto tracks_to_remove = selectTracksToRemove(num_to_remove);

        for (uint16_t track_id : tracks_to_remove)
        {
            lost_tracks_.erase(track_id);
        }
    }
}

void TrunkTracking::pruneLostTracks(const rclcpp::Time &current_time)
{
    for (auto it = lost_tracks_.begin(); it != lost_tracks_.end();)
    {
        if (isLostTrackExpired(it->second, current_time))
        {
            it = lost_tracks_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void TrunkTracking::updateTrack(TrackedTrunk &track,
                                const Eigen::Vector2d &measurement,
                                const Eigen::Matrix2d &measurement_cov)
{
    const Eigen::Vector2d innovation = measurement - track.position_map; // Innovation
    const Eigen::Matrix2d S = track.covariance + measurement_cov;        // Innovation covariance
    const Eigen::Matrix2d K = track.covariance * S.inverse();            // Kalman gain

    // State update
    track.position_map += K * innovation;

    // Covariance update (NOTE: Joseph form for numerical stability)
    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    track.covariance = (I - K) * track.covariance * (I - K).transpose() + K * measurement_cov * K.transpose();

    track.total_observations++;
}

void TrunkTracking::predictTracks(double dt)
{
    const Eigen::Matrix2d process_noise = Eigen::Matrix2d::Identity() * (process_noise_std_sq * dt);

    // Predict only active tracks
    for (auto &[id, track] : active_tracks_)
    {
        track.covariance += process_noise;
    }
}

void TrunkTracking::manageTrackStates(const rclcpp::Time &current_time)
{
    for (auto it = active_tracks_.begin(); it != active_tracks_.end();)
    {
        bool should_remove = false;

        switch (it->second.state)
        {
        case TrackState::TENTATIVE:
            if (it->second.shouldConfirm(params_.min_hit_rate_for_confirmation,
                                         params_.min_observations_for_confirmation))
            {
                // std::cout << "Track " << it->first << " confirmed with hit rate: "
                //           << std::fixed << std::setprecision(2) << it->second.getHitRate()
                //           << " over " << it->second.getWindowSize() << " observations" << std::endl;
                it->second.state = TrackState::CONFIRMED;
            }
            else if (it->second.getWindowSize() >= params_.min_observations_for_lost &&
                     it->second.getMissRate() >= 0.9) // TODO: Parameterize threshold?
            {
                // std::cout << "Removing tentative track " << it->first
                //           << " with poor hit rate: " << it->second.getHitRate() << std::endl;
                should_remove = true;
            }
            break;

        case TrackState::CONFIRMED:
            if (it->second.shouldMarkLost(params_.max_miss_rate_for_lost,
                                          params_.min_observations_for_lost))
            {
                // std::cout << "Track " << it->first << " marked as lost with miss rate: "
                //           << std::fixed << std::setprecision(2) << it->second.getMissRate()
                //           << " over " << it->second.getWindowSize() << " observations" << std::endl;

                it->second.state = TrackState::LOST;
                it->second.lost_time = current_time;
                inflateCovarianceForLostTrack(it->second);

                lost_tracks_.emplace(it->first, std::move(it->second));
                should_remove = true; // Remove from active track container
            }
            break;

        case TrackState::LOST:
            should_remove = true;
            break;
        }

        if (should_remove)
        {
            it = active_tracks_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void TrunkTracking::createNewTracks(const std::vector<Circle> &unassociated_detections,
                                    const geometry_msgs::msg::Pose &robot_pose,
                                    const rclcpp::Time &current_time)
{
    if (unassociated_detections.empty())
        return;

    // Filter valid detections
    std::vector<std::pair<Circle, Eigen::Vector2d>> valid_detections;
    valid_detections.reserve(unassociated_detections.size());

    for (const auto &detection : unassociated_detections)
    {
        if (detection.r >= 0.2 && detection.r <= 0.5) // TODO: Parameterize radius limits?
        {
            Eigen::Vector2d position_map = transformToMapFrame(
                Eigen::Vector2d(detection.x, detection.y), robot_pose);
            valid_detections.emplace_back(detection, position_map);
        }
    }

    if (valid_detections.empty())
        return;

    // Check against both active and lost tracks to avoid duplicates
    std::vector<TrackedTrunk *> all_existing_tracks;
    all_existing_tracks.reserve(active_tracks_.size() + lost_tracks_.size());

    for (auto &[id, track] : active_tracks_)
    {
        all_existing_tracks.push_back(&track);
    }
    for (auto &[id, track] : lost_tracks_)
    {
        all_existing_tracks.push_back(&track);
    }

    std::vector<Eigen::Vector2d> new_track_positions;
    new_track_positions.reserve(valid_detections.size());

    if (!all_existing_tracks.empty())
    {
        using TrunkKDTree = nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<double, TrunkTrackAdapter>,
            TrunkTrackAdapter, 2>;

        TrunkTrackAdapter adapter(all_existing_tracks);
        TrunkKDTree kdtree(2, adapter, nanoflann::KDTreeSingleIndexAdaptorParams(10));
        kdtree.buildIndex();

        std::vector<nanoflann::ResultItem<uint16_t, double>> matches;
        matches.reserve(all_existing_tracks.size());

        for (const auto &[detection, position_map] : valid_detections)
        {
            matches.clear();
            nanoflann::RadiusResultSet<double, uint16_t> result_set(min_distance_sq, matches);
            kdtree.findNeighbors(result_set, &position_map[0], {});

            // If no existing tracks are close, also check against newly created tracks in this batch
            if (matches.empty())
            {
                bool too_close_to_new = false;
                for (const auto &new_pos : new_track_positions)
                {
                    if ((position_map - new_pos).squaredNorm() < min_distance_sq)
                    {
                        too_close_to_new = true;
                        break;
                    }
                }

                if (!too_close_to_new)
                {
                    TrackedTrunk new_track(next_trunk_id_, position_map, detection.r);
                    new_track.created_time = current_time;
                    new_track.last_seen = current_time;

                    new_track_positions.push_back(position_map);
                    active_tracks_.emplace(next_trunk_id_, std::move(new_track));
                    next_trunk_id_++;
                }
            }
        }
    }
    else
    {
        // No existing tracks — check detections against each other to avoid duplicates
        for (const auto &[detection, position_map] : valid_detections)
        {
            bool too_close = false;
            for (const auto &new_pos : new_track_positions)
            {
                if ((position_map - new_pos).squaredNorm() < min_distance_sq)
                {
                    too_close = true;
                    break;
                }
            }

            if (!too_close)
            {
                TrackedTrunk new_track(next_trunk_id_, position_map, detection.r);
                new_track.created_time = current_time;
                new_track.last_seen = current_time;

                new_track_positions.push_back(position_map);
                active_tracks_.emplace(next_trunk_id_, std::move(new_track));
                next_trunk_id_++;
            }
        }
    }
}

Eigen::Vector2d TrunkTracking::transformToMapFrame(
    const Eigen::Vector2d &point,
    const geometry_msgs::msg::Pose &robot_pose)
{
    // Project pose to 2D plane (map)
    Eigen::Quaterniond q(robot_pose.orientation.w, robot_pose.orientation.x,
                         robot_pose.orientation.y, robot_pose.orientation.z);

    // Compute yaw
    double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                            1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

    double c = std::cos(yaw);
    double s = std::sin(yaw);

    Eigen::Matrix2d R;
    R << c, -s,
        s, c;

    Eigen::Vector2d t(robot_pose.position.x, robot_pose.position.y);
    return R * point + t;
}

double TrunkTracking::mahalanobisDistance(const Eigen::Vector2d &detection,
                                          const TrackedTrunk &track)
{
    Eigen::Vector2d diff = detection - track.position_map;
    Eigen::LLT<Eigen::Matrix2d> llt(track.covariance);
    if (llt.info() == Eigen::Success)
    {
        Eigen::Vector2d y = llt.matrixL().solve(diff);
        return y.squaredNorm();
    }
    else
    {
        return diff.transpose() * track.covariance.inverse() * diff;
    }
}

inline bool TrunkTracking::isLostTrackExpired(const TrackedTrunk &track, const rclcpp::Time &current_time) const
{
    double age_seconds = (current_time - track.lost_time).seconds();
    return age_seconds > params_.max_lost_track_age_seconds;
}

inline void TrunkTracking::inflateCovarianceForLostTrack(TrackedTrunk &track)
{
    track.covariance *= params_.covariance_inflation_factor;
    const double max_variance = 4.0; // 2m standard deviation
    track.covariance = track.covariance.cwiseMin(Eigen::Matrix2d::Identity() * max_variance);
}

inline std::vector<uint16_t> TrunkTracking::selectTracksToRemove(size_t num_to_remove)
{
    std::vector<std::pair<double, uint16_t>> track_scores;
    track_scores.reserve(lost_tracks_.size());

    for (const auto &[id, track] : lost_tracks_)
    {
        track_scores.emplace_back(track.calculateQualityScore(), id);
    }

    // Sort by quality score in ascending order (worst first)
    std::sort(track_scores.begin(), track_scores.end());

    std::vector<uint16_t> tracks_to_remove;
    tracks_to_remove.reserve(num_to_remove);

    for (size_t i = 0; i < std::min(num_to_remove, track_scores.size()); ++i)
    {
        tracks_to_remove.push_back(track_scores[i].second);
    }

    return tracks_to_remove;
}