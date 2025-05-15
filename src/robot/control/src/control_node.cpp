#include "control_node.hpp"
#include <cmath>
#include <optional>

ControlNode::ControlNode()
  : Node("control"),
    control_(robot::ControlCore(this->get_logger())),
    lookahead_distance_(1.0),
    goal_tolerance_(0.1),
    linear_speed_(0.85) {

  // Initialize subscribers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  // Initialize publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Initialize timer
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::controlLoop() {
  if (!current_path_ || !robot_odom_) {
    return;
  }

  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    RCLCPP_INFO(this->get_logger(), "No valid lookahead point found. Stopping robot.");
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist()); // Stop the robot
    return;
  }

  auto cmd_vel = computeVelocity(*lookahead_point);

  cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  for (const auto &pose : current_path_->poses) {
    double distance = computeDistance(robot_odom_->pose.pose.position, pose.pose.position);
    if (distance >= lookahead_distance_) {
      return pose;
    }
  }
  return std::nullopt;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  geometry_msgs::msg::Twist cmd_vel;

  double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

  double dx = target.pose.position.x - robot_odom_->pose.pose.position.x;
  double dy = target.pose.position.y - robot_odom_->pose.pose.position.y;
  double target_angle = std::atan2(dy, dx);

  double angular_error = target_angle - robot_yaw;

  while (angular_error > M_PI) angular_error -= 2 * M_PI;
  while (angular_error < -M_PI) angular_error += 2 * M_PI;

  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = 2.0 * angular_error;

  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  // Convert quaternion to yaw
  double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
