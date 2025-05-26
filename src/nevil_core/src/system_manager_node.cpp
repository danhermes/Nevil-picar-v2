#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SystemManagerNode : public rclcpp::Node
{
public:
  SystemManagerNode()
  : Node("system_manager_node")
  {
    // Initialize parameters
    this->declare_parameter("system_mode", "standby");
    
    // Create publishers
    system_mode_publisher_ = this->create_publisher<std_msgs::msg::String>("/system_mode", 10);
    
    // Create timers
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&SystemManagerNode::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "System Manager Node initialized");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    std::string mode = this->get_parameter("system_mode").as_string();
    message.data = mode;
    system_mode_publisher_->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "Publishing system mode: '%s'", mode.c_str());
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_mode_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemManagerNode>());
  rclcpp::shutdown();
  return 0;
}