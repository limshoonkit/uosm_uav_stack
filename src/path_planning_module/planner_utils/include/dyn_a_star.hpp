#ifndef DYN_A_STAR_HPP_
#define DYN_A_STAR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <queue>
#include <iostream>
#include <memory>
#include <vector>
#include <limits>
#include <chrono>
#include "grid_map.hpp"

namespace ego_planner
{

    constexpr double INF = std::numeric_limits<double>::infinity();

    enum class AStarResult
    {
        SUCCESS = 0,
        INIT_ERROR,
        SEARCH_ERROR
    };

    struct GridNode
    {
        enum class State
        {
            OPEN_SET = 1,
            CLOSED_SET = 2,
            UNDEFINED = 3
        };

        int rounds{0};
        State state{State::UNDEFINED};
        Eigen::Vector3i index{Eigen::Vector3i::Zero()};
        double g_score{INF};
        double f_score{INF};
        std::shared_ptr<GridNode> came_from{nullptr};

        GridNode() = default;
        explicit GridNode(const Eigen::Vector3i &idx) : index(idx) {}
    };

    using GridNodePtr = std::shared_ptr<GridNode>;

    class NodeComparator
    {
    public:
        bool operator()(const GridNodePtr &node1, const GridNodePtr &node2) const
        {
            return node1->f_score > node2->f_score;
        }
    };

    class AStar
    {
    private:
        std::shared_ptr<GridMap> grid_map_;
        double step_size_{0.0};
        double inv_step_size_{0.0};
        Eigen::Vector3d center_{Eigen::Vector3d::Zero()};
        Eigen::Vector3i center_idx_{Eigen::Vector3i::Zero()};
        Eigen::Vector3i pool_size_{Eigen::Vector3i::Zero()};

        static constexpr double TIE_BREAKER = 1.0 + 1.0 / 10000.0;

        std::vector<GridNodePtr> grid_path_;
        std::vector<std::vector<std::vector<GridNodePtr>>> grid_node_map_;
        std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> open_set_;
        int rounds_{0};

        // Helper methods
        inline void coord_to_grid_index(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);
        double get_diagonal_heuristic(const GridNodePtr &node1, const GridNodePtr &node2) const;
        double get_manhattan_heuristic(const GridNodePtr &node1, const GridNodePtr &node2) const;
        double get_euclidean_heuristic(const GridNodePtr &node1, const GridNodePtr &node2) const;
        inline double get_heuristic(const GridNodePtr &node1, const GridNodePtr &node2) const;
        inline Eigen::Vector3d index_to_coord(const Eigen::Vector3i &index) const;
        inline bool coord_to_index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;
        inline int check_occupancy(const Eigen::Vector3d &pos) const { return grid_map_->getInflateOccupancy(pos); }
        std::vector<GridNodePtr> retrieve_path(const GridNodePtr &current) const;
        bool convert_to_index_and_adjust_points(
            Eigen::Vector3d &start_pt, Eigen::Vector3d &end_pt,
            Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx) const;

    public:
        AStar() = default;
        ~AStar() = default;

        void init_grid_map(std::shared_ptr<GridMap> occ_map, const Eigen::Vector3i &pool_size);
        AStarResult search(double step_size, Eigen::Vector3d &start_pt, Eigen::Vector3d &end_pt);
        std::vector<Eigen::Vector3d> get_path() const;
    };

    // Inline implementations
    inline double AStar::get_heuristic(const GridNodePtr &node1, const GridNodePtr &node2) const
    {
        return TIE_BREAKER * get_diagonal_heuristic(node1, node2);
    }

    inline Eigen::Vector3d AStar::index_to_coord(const Eigen::Vector3i &index) const
    {
        return ((index - center_idx_).cast<double>() * step_size_) + center_;
    }

    inline bool AStar::coord_to_index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
    {
        idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + center_idx_;

        if (idx(0) < 0 || idx(0) >= pool_size_(0) ||
            idx(1) < 0 || idx(1) >= pool_size_(1) ||
            idx(2) < 0 || idx(2) >= pool_size_(2))
        {
            RCLCPP_ERROR(rclcpp::get_logger("ego_planner"),
                         "Index out of bounds: [%d, %d, %d], pool size: [%d, %d, %d]",
                         idx(0), idx(1), idx(2),
                         pool_size_(0), pool_size_(1), pool_size_(2));
            return false;
        }

        return true;
    }

} // namespace ego_planner

#endif // DYN_A_STAR_HPP_