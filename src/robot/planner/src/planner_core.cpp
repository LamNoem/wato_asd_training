#include "planner_core.hpp"
#include "planner_structs.hpp"

#include <vector>
#include <cmath>
#include <queue>
#include <unordered_map>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{
PlannerCore::PlannerCore() 
: logger_(rclcpp::get_logger("planner_default")) {} // Use a default logger


PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

nav_msgs::msg::Path PlannerCore::computePath(
    const nav_msgs::msg::OccupancyGrid &map,
    const geometry_msgs::msg::Pose &start,
    const geometry_msgs::msg::Point &goal)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = map.header.frame_id;
    path.header.stamp = rclcpp::Clock().now();

   

    // Convert start and goal to CellIndex
    CellIndex start_index(
        static_cast<int>((start.position.x - map.info.origin.position.x) / map.info.resolution),
        static_cast<int>((start.position.y - map.info.origin.position.y) / map.info.resolution));

    CellIndex goal_index(
        static_cast<int>((goal.x - map.info.origin.position.x) / map.info.resolution),
        static_cast<int>((goal.y - map.info.origin.position.y) / map.info.resolution));

    // Perform A* search
    std::vector<CellIndex> cell_path = computeAStarPath(map, start_index, goal_index);

    // Convert CellIndex path to ROS2 Path message
    for (const auto &cell : cell_path)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = map.info.origin.position.x + cell.x * map.info.resolution;
        pose.pose.position.y = map.info.origin.position.y + cell.y * map.info.resolution;
        pose.pose.position.z = 0.0; // Assuming 2D navigation
        path.poses.push_back(pose);
    }

    return path;
}

double PlannerCore::heuristic(const CellIndex &a, const CellIndex &b){
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

std::vector<CellIndex> PlannerCore::getNeighbors(const nav_msgs::msg::OccupancyGrid &grid, const CellIndex &current)
{
    std::vector<CellIndex> neighbors;
    int width = grid.info.width;
    int height = grid.info.height;

    const std::vector<std::pair<int, int>> directions = {
        {0, 1}, {0, -1}, {1, 0}, {-1, 0}};

    for (const auto &dir : directions)
    {
        int nx = current.x + dir.first;
        int ny = current.y + dir.second;

        if (nx >= 0 && nx < width && ny >= 0 && ny < height)
        {
            int index = ny * width + nx;
            if (grid.data[index] < 50) // Free cell
            {
                neighbors.emplace_back(nx, ny);
            }
        }
    }

    return neighbors;
}

std::vector<CellIndex> PlannerCore::reconstructPath(
    const std::unordered_map<CellIndex, CellIndex, CellIndexHash> &came_from,
    CellIndex current)
{
    std::vector<CellIndex> path;

    while (came_from.find(current) != came_from.end())
    {
        path.push_back(current);
        current = came_from.at(current);
    }

    path.push_back(current);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<CellIndex> PlannerCore::computeAStarPath(
    const nav_msgs::msg::OccupancyGrid &grid,
    const CellIndex &start,
    const CellIndex &goal)
{
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;

    g_score[start] = 0;
    double h_start = heuristic(start, goal);
    open_set.emplace(start, h_start);

    while (!open_set.empty())
    {
        AStarNode current = open_set.top();
        open_set.pop();

        if (current.index == goal)
        {
            return reconstructPath(came_from, current.index);
        }

        for (const CellIndex &neighbor : getNeighbors(grid, current.index))
        {
            double tentative_g = g_score[current.index] + heuristic(current.index, neighbor);

            if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor])
            {
                came_from[neighbor] = current.index;
                g_score[neighbor] = tentative_g;
                double f_score = tentative_g + heuristic(neighbor, goal);
                open_set.emplace(neighbor, f_score);
            }
        }
    }

    return {};
}

} 





