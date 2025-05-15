#include "planner_node.hpp"
#include <cmath>

PlannerNode::PlannerNode() 
  : Node("planner"), 
    planner_(robot::PlannerCore(this->get_logger())),
    state_(State::WAITING_FOR_GOAL) {
  
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
  return std::sqrt(dx * dx + dy * dy) < 0.5;
}

void PlannerNode::planPath() {
  if (!goal_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "map";

  int width = current_map_.info.width;
  int height = current_map_.info.height;
  double resolution = current_map_.info.resolution;
  CellIndex start(
    static_cast<int>((robot_pose_.position.x - current_map_.info.origin.position.x) / resolution),
    static_cast<int>((robot_pose_.position.y - current_map_.info.origin.position.y) / resolution));
  CellIndex goal(
    static_cast<int>((goal_.point.x - current_map_.info.origin.position.x) / resolution),
    static_cast<int>((goal_.point.y - current_map_.info.origin.position.y) / resolution));

  // Check if start and goal are valid
  if (start.x < 0 || start.x >= width || start.y < 0 || start.y >= height ||
      goal.x < 0 || goal.x >= width || goal.y < 0 || goal.y >= height) {
    RCLCPP_ERROR(this->get_logger(), "Start or goal is out of map bounds!");
    return;
  }

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;

  open_set.emplace(start, 0.0);
  g_score[start] = 0.0;

  while (!open_set.empty()) {
    AStarNode current = open_set.top();
    open_set.pop();

    if (current.index == goal) {
      CellIndex idx = goal;
      while (idx != start) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = idx.x * resolution + current_map_.info.origin.position.x;
        pose.pose.position.y = idx.y * resolution + current_map_.info.origin.position.y;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
        idx = came_from[idx];
      }
      std::reverse(path.poses.begin(), path.poses.end());
      path.header.frame_id = "sim_world";
      path_pub_->publish(path);
      RCLCPP_INFO(this->get_logger(), "Path successfully planned!");
      return;
    }

    std::vector<CellIndex> neighbors = {
      CellIndex(current.index.x + 1, current.index.y),
      CellIndex(current.index.x - 1, current.index.y),
      CellIndex(current.index.x, current.index.y + 1),
      CellIndex(current.index.x, current.index.y - 1)
    };

    for (const auto &neighbor : neighbors) {
      if (neighbor.x < 0 || neighbor.x >= width || neighbor.y < 0 || neighbor.y >= height) {
        continue;
      }

      int map_index = neighbor.y * width + neighbor.x;
      if (current_map_.data[map_index] > 50) {
        continue;
      }

      double tentative_g_score = g_score[current.index] + 1.0;
      if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
        g_score[neighbor] = tentative_g_score;
        double h_score = std::hypot(goal.x - neighbor.x, goal.y - neighbor.y);
        open_set.emplace(neighbor, tentative_g_score + h_score);
        came_from[neighbor] = current.index;
      }
    }
  }

  RCLCPP_WARN(this->get_logger(), "No valid path found!");
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
