#include "planner_node.hpp"
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

PlannerNode::PlannerNode() : Node("planner"), state_(State::WAITING_FOR_GOAL), goal_received_(false), planner_(robot::PlannerCore(this->get_logger())) {
  // Initialize subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    // Initialize publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    // Initialize timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = *msg;
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        planPath();
    }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    goal_ = *msg;
    goal_received_ = true;
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        if (goalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            state_ = State::WAITING_FOR_GOAL;
        } else {
            RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
            planPath();
        }
    }
}

bool PlannerNode::goalReached() {
    double dx = goal_.point.x - robot_pose_.position.x;
    double dy = goal_.point.y - robot_pose_.position.y;
    return std::sqrt(dx * dx + dy * dy) < 0.5;  // Distance threshold
}

void PlannerNode::planPath() {
    if (!goal_received_ || current_map_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
        return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";

    // Use planner_ to compute A* path
    path = planner_.computePath(current_map_, robot_pose_, goal_.point);

    path_pub_->publish(path);
    RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints", path.poses.size());
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
