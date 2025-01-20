#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

    // Timer callback to publish a test message
    void publishMessage();

    // Callback function to handle LaserScan messages
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // Initialize the costmap
    void initializeCostmap();

    // Convert LaserScan range and angle to grid coordinates
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);

    // Mark an obstacle in the costmap
    void markObstacle(int x_grid, int y_grid);

    // Inflate obstacles in the costmap
    void inflateObstacles();

    // Publish the costmap as an OccupancyGrid
    void publishCostmap();


  private:

    // Core costmap object
    robot::CostmapCore costmap_;
    // Publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    // Timer for periodic test message publishing
    rclcpp::TimerBase::SharedPtr timer_;

    // Costmap parameters
    double resolution_; // Grid resolution (meters per cell)
    int width_;         // Number of cells in x-direction
    int height_;        // Number of cells in y-direction
    double origin_x_;   // Origin of the costmap in x (meters)
    double origin_y_;   // Origin of the costmap in y (meters)

    // Costmap data
    std::vector<std::vector<int>> costmap_grid; // 2D array for storing grid costs
    //was costmap_ before
};

#endif 