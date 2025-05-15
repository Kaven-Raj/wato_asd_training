#include "map_memory_node.hpp"
#include <cmath>

MapMemoryNode::MapMemoryNode() 
  : Node("map_memory"), 
    map_memory_(robot::MapMemoryCore(this->get_logger())),
    last_x_(0.0), last_y_(0.0) {
  
  // Initialize subscribers
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Initialize publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Initialize timer
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));

  global_map_.info.resolution = 0.1;
  global_map_.info.origin.position.x = -15.0;
  global_map_.info.origin.position.y = -15.0;
  global_map_.info.width = 400;
  global_map_.info.height = 400;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.resize(400 * 400, 0);
  global_map_.header.frame_id = "sim_world";

}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  double distance = std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));
  if (distance >= distance_threshold_) {
    last_x_ = x;
    last_y_ = y;
    should_update_map_ = true;
  }
}

void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_updated_) {
    integrateCostmap();
    global_map_.header.stamp = this->now();
    map_pub_->publish(global_map_);
    should_update_map_ = false;
    costmap_updated_ = false;
  }
}

void MapMemoryNode::integrateCostmap() {
  for (size_t y = 0; y < latest_costmap_.info.height; ++y) {
    for (size_t x = 0; x < latest_costmap_.info.width; ++x) {
      int costmap_index = y * latest_costmap_.info.width + x;
      int costmap_value = latest_costmap_.data[costmap_index];

      if (costmap_value == -1) {
        continue;
      }

      // Transform costmap coordinates
      double world_x = latest_costmap_.info.origin.position.x + x * latest_costmap_.info.resolution;
      double world_y = latest_costmap_.info.origin.position.y + y * latest_costmap_.info.resolution;

      int global_x = static_cast<int>((world_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
      int global_y = static_cast<int>((world_y - global_map_.info.origin.position.y) / global_map_.info.resolution);

      if (global_x >= 0 && global_x < static_cast<int>(global_map_.info.width) &&
          global_y >= 0 && global_y < static_cast<int>(global_map_.info.height)) {
        int global_index = global_y * global_map_.info.width + global_x;

        if (global_map_.data[global_index] == -1 || costmap_value > global_map_.data[global_index]) {
          global_map_.data[global_index] = costmap_value;
        }
      }
    }
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
