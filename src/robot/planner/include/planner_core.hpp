#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <vector>
#include <unordered_map>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "planner_structs.hpp" 


namespace robot
{

class PlannerCore {
  public:
    PlannerCore();
    explicit PlannerCore(const rclcpp::Logger& logger);

    nav_msgs::msg::Path computePath(
    const nav_msgs::msg::OccupancyGrid &map,
    const geometry_msgs::msg::Pose &start,
    const geometry_msgs::msg::Point &goal);

    double heuristic(const CellIndex &a, const CellIndex &b);

    std::vector<CellIndex> getNeighbors(const nav_msgs::msg::OccupancyGrid &grid, const CellIndex &current);

    std::vector<CellIndex> reconstructPath(
    const std::unordered_map<CellIndex, CellIndex, CellIndexHash> &came_from,
    CellIndex current);

    std::vector<CellIndex> computeAStarPath(
    const nav_msgs::msg::OccupancyGrid &grid,
    const CellIndex &start,
    const CellIndex &goal);


    
  private:
    rclcpp::Logger logger_;
};

}  

#endif  
