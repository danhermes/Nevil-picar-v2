#ifndef NEVIL_NAVIGATION_MOTION_CONTROL_HPP_
#define NEVIL_NAVIGATION_MOTION_CONTROL_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace nevil_navigation
{

/**
 * @class MotionControlNode
 * @brief Motion Control Node for Nevil-picar v2.0
 * 
 * This node is responsible for:
 * - Translating high-level commands to motor/servo controls
 * - Implementing safety limits and checks
 * - Providing feedback on motion execution
 * - Managing expressive behaviors
 */
class MotionControlNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for MotionControlNode
   */
  MotionControlNode();

private:
  /**
   * @brief Callback for velocity commands
   * @param msg Twist message containing linear and angular velocity
   */
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  
  /**
   * @brief Callback for system mode changes
   * @param msg String message containing the system mode
   */
  void system_mode_callback(const std_msgs::msg::String::SharedPtr msg);
  
  /**
   * @brief Timer callback to publish motor status
   */
  void timer_callback();

  // ROS2 objects
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_mode_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // State variables
  double current_linear_;
  double current_angular_;
  std::string current_mode_;
};

}  // namespace nevil_navigation

#endif  // NEVIL_NAVIGATION_MOTION_CONTROL_HPP_