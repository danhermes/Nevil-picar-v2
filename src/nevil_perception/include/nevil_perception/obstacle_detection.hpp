#ifndef NEVIL_PERCEPTION_OBSTACLE_DETECTION_HPP_
#define NEVIL_PERCEPTION_OBSTACLE_DETECTION_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace nevil_perception
{

/**
 * @class ObstacleDetectionNode
 * @brief Obstacle Detection Node for Nevil-picar v2.0
 * 
 * This node is responsible for:
 * - Processing ultrasonic sensor data
 * - Detecting obstacles in the robot's path
 * - Providing distance measurements
 * - Triggering obstacle avoidance behaviors
 */
class ObstacleDetectionNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for ObstacleDetectionNode
   */
  ObstacleDetectionNode();

private:
  /**
   * @brief Callback for ultrasonic sensor data
   * @param msg Range message from ultrasonic sensor
   */
  void ultrasonic_callback(const sensor_msgs::msg::Range::SharedPtr msg);
  
  /**
   * @brief Callback for system mode changes
   * @param msg String message containing the system mode
   */
  void system_mode_callback(const std_msgs::msg::String::SharedPtr msg);
  
  /**
   * @brief Timer callback to check for sensor timeout
   */
  void timer_callback();
  
  /**
   * @brief Send stop command to motors
   */
  void send_stop_command();

  // ROS2 objects
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultrasonic_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_mode_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_detected_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // State variables
  double current_range_;
  rclcpp::Time last_range_time_;
  std::string current_mode_;
};

}  // namespace nevil_perception

#endif  // NEVIL_PERCEPTION_OBSTACLE_DETECTION_HPP_