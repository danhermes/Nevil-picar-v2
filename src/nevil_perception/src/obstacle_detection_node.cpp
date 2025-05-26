#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class ObstacleDetectionNode : public rclcpp::Node
{
public:
  ObstacleDetectionNode()
  : Node("obstacle_detection_node")
  {
    // Initialize parameters
    this->declare_parameter("min_distance", 0.2);  // meters
    this->declare_parameter("safety_enabled", true);
    this->declare_parameter("sensor_timeout", 0.5); // seconds
    
    // Create subscribers
    ultrasonic_subscription_ = this->create_subscription<sensor_msgs::msg::Range>(
      "/ultrasonic_data", 10, std::bind(&ObstacleDetectionNode::ultrasonic_callback, this, std::placeholders::_1));
    
    system_mode_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/system_mode", 10, std::bind(&ObstacleDetectionNode::system_mode_callback, this, std::placeholders::_1));
    
    // Create publishers
    obstacle_detected_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/obstacle_detected", 10);
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Create timers
    timer_ = this->create_wall_timer(
      100ms, std::bind(&ObstacleDetectionNode::timer_callback, this));
    
    // Initialize state variables
    last_range_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Obstacle Detection Node initialized");
  }

private:
  void ultrasonic_callback(const sensor_msgs::msg::Range::SharedPtr msg)
  {
    // Store the current range reading
    current_range_ = msg->range;
    last_range_time_ = this->now();
    
    // Check if obstacle is detected
    double min_distance = this->get_parameter("min_distance").as_double();
    bool obstacle_detected = current_range_ < min_distance;
    
    // Publish obstacle detection status
    auto obstacle_msg = std::make_unique<std_msgs::msg::Bool>();
    obstacle_msg->data = obstacle_detected;
    obstacle_detected_publisher_->publish(std::move(obstacle_msg));
    
    // If safety is enabled and obstacle is detected, send stop command
    if (this->get_parameter("safety_enabled").as_bool() && obstacle_detected) {
      RCLCPP_WARN(this->get_logger(), "Obstacle detected at %.2f meters! Sending stop command.", current_range_);
      send_stop_command();
    }
  }
  
  void system_mode_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    current_mode_ = msg->data;
    
    // If system is in standby mode, disable safety
    if (current_mode_ == "standby") {
      this->set_parameter(rclcpp::Parameter("safety_enabled", false));
      RCLCPP_INFO(this->get_logger(), "System in standby mode, disabling obstacle safety");
    } else {
      this->set_parameter(rclcpp::Parameter("safety_enabled", true));
      RCLCPP_INFO(this->get_logger(), "System in %s mode, enabling obstacle safety", current_mode_.c_str());
    }
  }
  
  void timer_callback()
  {
    // Check for sensor timeout
    double timeout = this->get_parameter("sensor_timeout").as_double();
    rclcpp::Time now = this->now();
    double elapsed = (now - last_range_time_).seconds();
    
    if (elapsed > timeout) {
      RCLCPP_ERROR(this->get_logger(), "Ultrasonic sensor timeout (%.2f seconds)! Sending stop command.", elapsed);
      send_stop_command();
    }
  }
  
  void send_stop_command()
  {
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    // All velocities set to zero
    cmd_vel_publisher_->publish(std::move(cmd_vel));
  }

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultrasonic_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_mode_subscription_;
  
  // Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_detected_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr timer_;
  
  // State variables
  double current_range_ = 999.0;  // Initialize to a large value
  rclcpp::Time last_range_time_;
  std::string current_mode_ = "standby";
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetectionNode>());
  rclcpp::shutdown();
  return 0;
}