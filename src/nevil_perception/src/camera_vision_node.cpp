#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

using namespace std::chrono_literals;

class CameraVisionNode : public rclcpp::Node
{
public:
  CameraVisionNode()
  : Node("camera_vision_node")
  {
    // Initialize parameters
    this->declare_parameter("enable_detection", true);
    this->declare_parameter("detection_threshold", 0.5);
    this->declare_parameter("camera_fps", 15);
    
    // Create subscribers
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&CameraVisionNode::image_callback, this, std::placeholders::_1));
    
    system_mode_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/system_mode", 10, std::bind(&CameraVisionNode::system_mode_callback, this, std::placeholders::_1));
    
    // Create publishers
    detections_publisher_ = this->create_publisher<vision_msgs::msg::Detection2DArray>("/object_detections", 10);
    processed_image_publisher_ = image_transport::create_publisher(this, "/camera/processed_image");
    
    // Create timers
    timer_ = this->create_wall_timer(
      100ms, std::bind(&CameraVisionNode::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Camera Vision Node initialized");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!this->get_parameter("enable_detection").as_bool()) {
      return;
    }
    
    try {
      // Convert ROS Image message to OpenCV image
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      
      // Store the current image
      current_image_ = cv_ptr->image;
      
      // Here we would implement object detection
      // For now, just log that we received an image
      RCLCPP_DEBUG(this->get_logger(), "Received image: %dx%d", 
                  current_image_.cols, current_image_.rows);
      
      // Process the image (in a real implementation, this would detect objects)
      process_image();
      
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }
  
  void system_mode_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    current_mode_ = msg->data;
    
    // If system is in standby mode, disable detection
    if (current_mode_ == "standby") {
      this->set_parameter(rclcpp::Parameter("enable_detection", false));
      RCLCPP_INFO(this->get_logger(), "System in standby mode, disabling object detection");
    } else {
      this->set_parameter(rclcpp::Parameter("enable_detection", true));
      RCLCPP_INFO(this->get_logger(), "System in %s mode, enabling object detection", current_mode_.c_str());
    }
  }
  
  void timer_callback()
  {
    // Publish status or other periodic updates
    if (!current_image_.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "Processing image in timer callback");
    }
  }
  
  void process_image()
  {
    if (current_image_.empty()) {
      return;
    }
    
    // In a real implementation, this would run object detection
    // For now, just create a simple detection message
    auto detections_msg = std::make_unique<vision_msgs::msg::Detection2DArray>();
    detections_msg->header.stamp = this->now();
    detections_msg->header.frame_id = "camera_frame";
    
    // Publish detections
    detections_publisher_->publish(std::move(detections_msg));
    
    // Publish processed image
    sensor_msgs::msg::Image::SharedPtr processed_img_msg = 
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", current_image_).toImageMsg();
    processed_image_publisher_.publish(*processed_img_msg);
  }

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_mode_subscription_;
  
  // Publishers
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_publisher_;
  image_transport::Publisher processed_image_publisher_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr timer_;
  
  // State variables
  cv::Mat current_image_;
  std::string current_mode_ = "standby";
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraVisionNode>());
  rclcpp::shutdown();
  return 0;
}