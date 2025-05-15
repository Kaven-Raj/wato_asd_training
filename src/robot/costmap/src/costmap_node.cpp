#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap_node") {
    // Initialize parameters
    resolution_ = 0.1;
    width_ = 100;
    height_ = 100;
    origin_x_ = -5.0;
    origin_y_ = -5.0;
    inflation_radius_ = 1.0; 
    max_cost_ = 100;

    // Initialize publisher and subscriber
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::initializeCostmap() {
    costmap_ = std::vector<std::vector<int>>(height_, std::vector<int>(width_, 0));
}

void CostmapNode::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    x_grid = static_cast<int>((x - origin_x_) / resolution_);
    y_grid = static_cast<int>((y - origin_y_) / resolution_);
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
    if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
        costmap_[y_grid][x_grid] = max_cost_;
    }
}

void CostmapNode::inflateObstacles() {
    std::vector<std::vector<int>> inflated_costmap = costmap_;

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (costmap_[y][x] == max_cost_) {
                for (int dy = -inflation_radius_ / resolution_; dy <= inflation_radius_ / resolution_; ++dy) {
                    for (int dx = -inflation_radius_ / resolution_; dx <= inflation_radius_ / resolution_; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                            double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                            if (distance <= inflation_radius_) {
                                int cost = max_cost_ * (1 - distance / inflation_radius_);
                                inflated_costmap[ny][nx] = cost;
                            }
                        }
                    }
                }
            }
        }
    }

    costmap_ = inflated_costmap;
}

void CostmapNode::publishCostmap() {
    auto msg = nav_msgs::msg::OccupancyGrid();
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";

    msg.info.resolution = resolution_;
    msg.info.width = width_;
    msg.info.height = height_;
    msg.info.origin.position.x = origin_x_;
    msg.info.origin.position.y = origin_y_;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.data.resize(width_ * height_);
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            msg.data[y * width_ + x] = costmap_[y][x];
        }
    }

    costmap_pub_->publish(msg);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    initializeCostmap();

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
    }

    inflateObstacles();
    publishCostmap();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}