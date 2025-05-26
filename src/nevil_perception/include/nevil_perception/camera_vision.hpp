#ifndef NEVIL_PERCEPTION_CAMERA_VISION_HPP_
#define NEVIL_PERCEPTION_CAMERA_VISION_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

namespace nevil_perception
{

/**
 * @class CameraVisionNode
 * @brief Camera Vision Node for Nevil-picar v2.0
 * 
 * This node is responsible for:
 * - Processing camera images
 * - Detecting and tracking objects
 * - Extracting visual features
 * - Providing visual data for navigation and AI processing
 */
class CameraVisionNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for CameraVisionNode
   */
  CameraVisionNode();

private:
  /**
   * @brief Callback for image messages
   * @param msg Image message from camera
   */
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  
  /**
   * @brief Callback for system mode changes
   * @param msg String message containing the system mode
   */
  void system_mode_callback(const std_msgs::msg::String::SharedPtr msg);
  
  /**
   * @brief Timer callback for periodic processing
   */
  void timer_callback();
  
  /**
   * @brief Process image and detect objects
   */
  void process_image();

  // ROS2 objects
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_mode_subscription_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_publisher_;
  image_transport::Publisher processed_image_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // State variables
  cv::Mat current_image_;
  std::string current_mode_;
};

}  // namespace nevil_perception

#endif  // NEVIL_PERCEPTION_CAMERA_VISION_HPP_