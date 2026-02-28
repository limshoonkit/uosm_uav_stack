/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Portions adapted from EGO-Planner-v2: https://github.com/ZJU-FAST-Lab/EGO-Planner-v2
 */

#include "dyn_a_star.hpp"

namespace ego_planner
{
    void AStar::init_grid_map(std::shared_ptr<GridMap> occ_map, const Eigen::Vector3i &pool_size)
    {
        pool_size_ = pool_size;
        center_idx_ = pool_size / 2;

        // Initialize 3D vector with proper dimensions
        grid_node_map_.resize(pool_size_(0));
        for (int i = 0; i < pool_size_(0); ++i)
        {
            grid_node_map_[i].resize(pool_size_(1));
            for (int j = 0; j < pool_size_(1); ++j)
            {
                grid_node_map_[i][j].resize(pool_size_(2));
                for (int k = 0; k < pool_size_(2); ++k)
                {
                    grid_node_map_[i][j][k] = std::make_shared<GridNode>(Eigen::Vector3i(i, j, k));
                }
            }
        }

        grid_map_ = occ_map;
    }

    double AStar::get_diagonal_heuristic(const GridNodePtr &node1, const GridNodePtr &node2) const
    {
        const double dx = std::abs(node1->index(0) - node2->index(0));
        const double dy = std::abs(node1->index(1) - node2->index(1));
        const double dz = std::abs(node1->index(2) - node2->index(2));

        const int diag = std::min({dx, dy, dz});
        const double remaining_x = dx - diag;
        const double remaining_y = dy - diag;
        const double remaining_z = dz - diag;

        double h = 0.0;

        if (remaining_x == 0)
        {
            h = std::sqrt(3.0) * diag + std::sqrt(2.0) * std::min(remaining_y, remaining_z) +
                std::abs(remaining_y - remaining_z);
        }
        else if (remaining_y == 0)
        {
            h = std::sqrt(3.0) * diag + std::sqrt(2.0) * std::min(remaining_x, remaining_z) +
                std::abs(remaining_x - remaining_z);
        }
        else if (remaining_z == 0)
        {
            h = std::sqrt(3.0) * diag + std::sqrt(2.0) * std::min(remaining_x, remaining_y) +
                std::abs(remaining_x - remaining_y);
        }

        return h;
    }

    double AStar::get_manhattan_heuristic(const GridNodePtr &node1, const GridNodePtr &node2) const
    {
        const double dx = std::abs(node1->index(0) - node2->index(0));
        const double dy = std::abs(node1->index(1) - node2->index(1));
        const double dz = std::abs(node1->index(2) - node2->index(2));

        return dx + dy + dz;
    }

    double AStar::get_euclidean_heuristic(const GridNodePtr &node1, const GridNodePtr &node2) const
    {
        return (node2->index - node1->index).cast<double>().norm();
    }

    std::vector<GridNodePtr> AStar::retrieve_path(const GridNodePtr &current) const
    {
        std::vector<GridNodePtr> path;
        auto node = current;

        while (node != nullptr)
        {
            path.push_back(node);
            node = node->came_from;
        }

        return path;
    }

    bool AStar::convert_to_index_and_adjust_points(Eigen::Vector3d &start_pt, Eigen::Vector3d &end_pt,
                                                   Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx) const
    {
        auto start_opt = coord_to_index(start_pt, start_idx);
        auto end_opt = coord_to_index(end_pt, end_idx);

        if (!start_opt || !end_opt)
        {
            return false;
        }

        // Adjust start point if it's in an obstacle
        if (check_occupancy(index_to_coord(start_idx)))
        {
            RCLCPP_WARN(rclcpp::get_logger("ego_planner"), "Start point is inside an obstacle.");

            const Eigen::Vector3d direction = (start_pt - end_pt).normalized();
            int occ = 0;

            do
            {
                start_pt = direction * step_size_ + start_pt;
                if (!coord_to_index(start_pt, start_idx))
                {
                    return false;
                }
                occ = check_occupancy(index_to_coord(start_idx));

                if (occ == -1)
                {
                    RCLCPP_WARN(rclcpp::get_logger("ego_planner"),
                                "Start point outside the map region.");
                    return false;
                }
            } while (occ > 0);
        }

        // Adjust end point if it's in an obstacle
        if (check_occupancy(index_to_coord(end_idx)))
        {
            RCLCPP_WARN(rclcpp::get_logger("ego_planner"), "End point is inside an obstacle.");

            const Eigen::Vector3d direction = (end_pt - start_pt).normalized();
            int occ = 0;

            do
            {
                end_pt = direction * step_size_ + end_pt;
                if (!coord_to_index(end_pt, end_idx))
                {
                    return false;
                }
                occ = check_occupancy(index_to_coord(end_idx));

                if (occ == -1)
                {
                    RCLCPP_WARN(rclcpp::get_logger("ego_planner"),
                                "End point outside the map region.");
                    return false;
                }
            } while (occ > 0);
        }

        return true;
    }

    AStarResult AStar::search(double step_size, Eigen::Vector3d &start_pt, Eigen::Vector3d &end_pt)
    {
        const auto start_time = std::chrono::steady_clock::now();
        ++rounds_;

        step_size_ = step_size;
        inv_step_size_ = 1.0f / step_size;
        center_ = (start_pt + end_pt) / 2.0f;

        Eigen::Vector3i start_idx, end_idx;
        if (!convert_to_index_and_adjust_points(start_pt, end_pt, start_idx, end_idx))
        {
            RCLCPP_ERROR(rclcpp::get_logger("ego_planner"),
                         "Unable to handle the initial or end point, force return!");
            return AStarResult::INIT_ERROR;
        }

        auto start_ptr = grid_node_map_[start_idx(0)][start_idx(1)][start_idx(2)];
        auto end_ptr = grid_node_map_[end_idx(0)][end_idx(1)][end_idx(2)];

        if (!start_ptr || !end_ptr)
        {
            return AStarResult::INIT_ERROR;
        }

        // Clear the open set
        std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
        open_set_.swap(empty);

        // Initialize
        end_ptr->index = end_idx;
        start_ptr->index = start_idx;
        start_ptr->rounds = rounds_;
        start_ptr->g_score = 0.0;
        start_ptr->f_score = get_heuristic(start_ptr, end_ptr);
        start_ptr->state = GridNode::State::OPEN_SET;
        start_ptr->came_from = nullptr;
        open_set_.push(start_ptr);

        int num_iter = 0;
        constexpr double TIME_LIMIT_SEC = 0.2;

        while (!open_set_.empty())
        {
            ++num_iter;
            auto current = open_set_.top();
            open_set_.pop();

            // Check if we reached the goal
            if (current->index == end_ptr->index)
            {
                grid_path_ = retrieve_path(current);
                return AStarResult::SUCCESS;
            }

            current->state = GridNode::State::CLOSED_SET;

            // Explore all 26 neighbors (3D grid)
            for (int dx = -1; dx <= 1; ++dx)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    for (int dz = -1; dz <= 1; ++dz)
                    {
                        if (dx == 0 && dy == 0 && dz == 0)
                        {
                            continue;
                        }

                        const Eigen::Vector3i neighbor_idx = current->index + Eigen::Vector3i(dx, dy, dz);

                        // Check bounds with 1-cell margin
                        if (neighbor_idx(0) < 1 || neighbor_idx(0) >= pool_size_(0) - 1 ||
                            neighbor_idx(1) < 1 || neighbor_idx(1) >= pool_size_(1) - 1 ||
                            neighbor_idx(2) < 1 || neighbor_idx(2) >= pool_size_(2) - 1)
                        {
                            continue;
                        }

                        auto neighbor_ptr = grid_node_map_[neighbor_idx(0)][neighbor_idx(1)][neighbor_idx(2)];
                        if (!neighbor_ptr)
                        {
                            continue;
                        }

                        const bool already_explored = (neighbor_ptr->rounds == rounds_);

                        if (already_explored && neighbor_ptr->state == GridNode::State::CLOSED_SET)
                        {
                            continue;
                        }

                        neighbor_ptr->rounds = rounds_;

                        // Check occupancy
                        if (check_occupancy(index_to_coord(neighbor_ptr->index)))
                        {
                            continue;
                        }

                        const double movement_cost = std::sqrt(dx * dx + dy * dy + dz * dz);
                        const double tentative_g_score = current->g_score + movement_cost;

                        if (!already_explored)
                        {
                            // Discover new node
                            neighbor_ptr->state = GridNode::State::OPEN_SET;
                            neighbor_ptr->came_from = current;
                            neighbor_ptr->g_score = tentative_g_score;
                            neighbor_ptr->f_score = tentative_g_score + get_heuristic(neighbor_ptr, end_ptr);
                            open_set_.push(neighbor_ptr);
                        }
                        else if (tentative_g_score < neighbor_ptr->g_score)
                        {
                            // Update existing node with better path
                            neighbor_ptr->came_from = current;
                            neighbor_ptr->g_score = tentative_g_score;
                            neighbor_ptr->f_score = tentative_g_score + get_heuristic(neighbor_ptr, end_ptr);
                        }
                    }
                }
            }

            // Check time limit
            const auto current_time = std::chrono::steady_clock::now();
            const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
                                     current_time - start_time)
                                     .count();

            if (elapsed > TIME_LIMIT_SEC)
            {
                RCLCPP_WARN(rclcpp::get_logger("ego_planner"),
                            "A* search exceeded time limit of %.1f seconds", TIME_LIMIT_SEC);
                return AStarResult::SEARCH_ERROR;
            }
        }

        const auto end_time = std::chrono::steady_clock::now();
        const auto total_time = std::chrono::duration_cast<std::chrono::duration<double>>(
                                    end_time - start_time)
                                    .count();

        if (total_time > 0.1)
        {
            RCLCPP_WARN(rclcpp::get_logger("ego_planner"),
                        "A* search took %.3fs with %d iterations", total_time, num_iter);
        }

        return AStarResult::SEARCH_ERROR;
    }

    std::vector<Eigen::Vector3d> AStar::get_path() const
    {
        std::vector<Eigen::Vector3d> path;
        path.reserve(grid_path_.size());

        for (const auto &node_ptr : grid_path_)
        {
            path.push_back(index_to_coord(node_ptr->index));
        }

        std::reverse(path.begin(), path.end());
        return path;
    }
} // namespace ego_planner