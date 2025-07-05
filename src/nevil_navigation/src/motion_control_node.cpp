#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nevil_interfaces/srv/hardware_command.hpp"

using namespace std::chrono_literals;

class MotionControlNode : public rclcpp::Node
{
public:
  MotionControlNode()
  : Node("motion_control_node")
  {
    // Initialize parameters
    this->declare_parameter("max_linear_speed", 0.5);  // m/s
    this->declare_parameter("max_angular_speed", 1.0); // rad/s
    this->declare_parameter("safety_enabled", true);
    this->declare_parameter("use_hardware_bridge", true);
    
    // Create subscribers
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&MotionControlNode::cmd_vel_callback, this, std::placeholders::_1));
    
    system_mode_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/system_mode", 10, std::bind(&MotionControlNode::system_mode_callback, this, std::placeholders::_1));
    
    // Create publishers
    motor_status_publisher_ = this->create_publisher<std_msgs::msg::String>("/motor_status", 10);
    
    // Create hardware service client
    hardware_client_ = this->create_client<nevil_interfaces::srv::HardwareCommand>("hardware_command");
    
    // Create timers
    //timer_ = this->create_wall_timer(
    //  100ms, std::bind(&MotionControlNode::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Motion Control Node initialized");
    
    // Wait for hardware service
    if (this->get_parameter("use_hardware_bridge").as_bool()) {
      RCLCPP_INFO(this->get_logger(), "Waiting for hardware bridge service...");
      while (!hardware_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for hardware service");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Hardware service not available, waiting...");
      }
      RCLCPP_INFO(this->get_logger(), "Hardware bridge service connected!");
    }
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Apply safety limits
    double max_linear = this->get_parameter("max_linear_speed").as_double();
    double max_angular = this->get_parameter("max_angular_speed").as_double();
    
    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;
    
    // Apply limits if safety is enabled
    if (this->get_parameter("safety_enabled").as_bool()) {
      linear_x = std::min(std::max(linear_x, -max_linear), max_linear);
      angular_z = std::min(std::max(angular_z, -max_angular), max_angular);
    }
    
    // Store current velocity command
    current_linear_ = linear_x;
    current_angular_ = angular_z;
    
    RCLCPP_DEBUG(this->get_logger(), "Received velocity command: linear=%.2f, angular=%.2f",
                linear_x, angular_z);
    
    // Send command to hardware bridge if enabled
    if (this->get_parameter("use_hardware_bridge").as_bool() && hardware_client_->service_is_ready()) {
      send_hardware_command(linear_x, angular_z, false, "velocity");
    } else {
      // Fallback: just log the command (simulation mode)
      RCLCPP_INFO(this->get_logger(), "Setting motor speeds for linear=%.2f, angular=%.2f",
                 linear_x, angular_z);
    }
  }
  
  void system_mode_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    current_mode_ = msg->data;
    
    // If system is in standby mode, stop motors
    if (current_mode_ == "standby") {
      current_linear_ = 0.0;
      current_angular_ = 0.0;
      RCLCPP_INFO(this->get_logger(), "System in standby mode, stopping motors");
    }
  }
  
  void send_hardware_command(double linear_x, double angular_z, bool emergency_stop, const std::string& command_type)
  {
    auto request = std::make_shared<nevil_interfaces::srv::HardwareCommand::Request>();
    request->linear_x = linear_x;
    request->angular_z = angular_z;
    request->emergency_stop = emergency_stop;
    request->command_type = command_type;
    
    // Use async_send_request with callback
    auto response_callback = [this](rclcpp::Client<nevil_interfaces::srv::HardwareCommand>::SharedFuture future) {
      try {
        auto response = future.get();
        if (response->success) {
          RCLCPP_DEBUG(this->get_logger(), "Hardware command executed successfully: %s (backend: %s)",
                      response->status_message.c_str(), response->hardware_backend.c_str());
          hardware_status_ = response->status_message;
          hardware_backend_ = response->hardware_backend;
          hardware_ready_ = response->hardware_ready;
        } else {
          RCLCPP_WARN(this->get_logger(), "Hardware command failed: %s", response->status_message.c_str());
          hardware_status_ = "Hardware command failed: " + response->status_message;
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Hardware service call failed: %s", e.what());
        hardware_status_ = "Service call failed";
      }
    };
    
    // Send request with callback
    hardware_client_->async_send_request(request, response_callback);
  }
  
  void timer_callback()
  {
    // Publish motor status
    auto message = std_msgs::msg::String();
    message.data = "Motors running: linear=" + std::to_string(current_linear_) +
                  ", angular=" + std::to_string(current_angular_) +
                  ", backend=" + hardware_backend_ +
                  ", status=" + hardware_status_;
    motor_status_publisher_->publish(message);
  }

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_mode_subscription_;
  
  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_status_publisher_;
  
  // Service clients
  rclcpp::Client<nevil_interfaces::srv::HardwareCommand>::SharedPtr hardware_client_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr timer_;
  
  // State variables
  double current_linear_ = 0.0;
  double current_angular_ = 0.0;
  std::string current_mode_ = "standby";
  std::string hardware_status_ = "Not connected";
  std::string hardware_backend_ = "unknown";
  bool hardware_ready_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionControlNode>());
  rclcpp::shutdown();
  return 0;
}