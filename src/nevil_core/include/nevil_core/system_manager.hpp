#ifndef NEVIL_CORE_SYSTEM_MANAGER_HPP_
#define NEVIL_CORE_SYSTEM_MANAGER_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace nevil_core
{

/**
 * @class SystemManagerNode
 * @brief System Manager Node for Nevil-picar v2.0
 * 
 * This node is responsible for:
 * - Managing system modes
 * - Coordinating node lifecycles
 * - Monitoring system health
 */
class SystemManagerNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for SystemManagerNode
   */
  SystemManagerNode();

private:
  /**
   * @brief Timer callback to publish system mode
   */
  void timer_callback();

  // ROS2 objects
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_mode_publisher_;
};

}  // namespace nevil_core

#endif  // NEVIL_CORE_SYSTEM_MANAGER_HPP_