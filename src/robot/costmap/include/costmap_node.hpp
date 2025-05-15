#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

  private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void initializeCostmap();
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    void publishCostmap();

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    std::vector<std::vector<int>> costmap_;
    double resolution_;
    int width_, height_;
    double origin_x_, origin_y_;
    double inflation_radius_;
    int max_cost_;
};

#endif