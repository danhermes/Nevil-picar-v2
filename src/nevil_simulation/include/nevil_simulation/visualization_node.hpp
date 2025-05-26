#ifndef NEVIL_SIMULATION_VISUALIZATION_NODE_HPP
#define NEVIL_SIMULATION_VISUALIZATION_NODE_HPP

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"

namespace nevil_simulation {

/**
 * @brief Visualization node for Nevil-picar simulation
 * 
 * This node provides visualization capabilities for the Nevil-picar simulation,
 * including robot state, sensor data, and environment visualization.
 */
class VisualizationNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor
   */
  VisualizationNode();
  
  /**
   * @brief Destructor
   */
  virtual ~VisualizationNode();

private:
  /**
   * @brief Initialize the visualization
   * 
   * @return bool True if successful, false otherwise
   */
  bool initialize();
  
  /**
   * @brief Load visualization parameters from ROS2 parameters
   */
  void loadParameters();
  
  /**
   * @brief Create publishers and subscribers
   */
  void createPubSub();
  
  /**
   * @brief Timer callback for visualization update
   */
  void timerCallback();
  
  /**
   * @brief Camera image callback
   * 
   * @param msg Camera image message
   */
  void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  
  /**
   * @brief Ultrasonic sensor callback
   * 
   * @param msg Ultrasonic sensor message
   */
  void ultrasonicCallback(const sensor_msgs::msg::Range::SharedPtr msg);
  
  /**
   * @brief Robot pose callback
   * 
   * @param msg Robot pose message
   */
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  
  /**
   * @brief Markers callback
   * 
   * @param msg Markers message
   */
  void markersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
  
  /**
   * @brief Status callback
   * 
   * @param msg Status message
   */
  void statusCallback(const std_msgs::msg::String::SharedPtr msg);
  
  /**
   * @brief Update the visualization
   */
  void updateVisualization();
  
  /**
   * @brief Create robot visualization markers
   * 
   * @return visualization_msgs::msg::MarkerArray Robot markers
   */
  visualization_msgs::msg::MarkerArray createRobotMarkers();
  
  /**
   * @brief Create sensor visualization markers
   * 
   * @return visualization_msgs::msg::MarkerArray Sensor markers
   */
  visualization_msgs::msg::MarkerArray createSensorMarkers();
  
  /**
   * @brief Create debug visualization markers
   * 
   * @return visualization_msgs::msg::MarkerArray Debug markers
   */
  visualization_msgs::msg::MarkerArray createDebugMarkers();
  
  /**
   * @brief Publish visualization markers
   */
  void publishVisualizationMarkers();

  // Visualization parameters
  double update_rate_;
  bool show_robot_;
  bool show_sensors_;
  bool show_debug_;
  bool show_camera_;
  bool show_ultrasonic_;
  bool show_trajectories_;
  
  // Visualization state
  bool initialized_;
  std::chrono::time_point<std::chrono::steady_clock> last_update_time_;
  
  // Robot state
  geometry_msgs::msg::PoseStamped robot_pose_;
  sensor_msgs::msg::Range ultrasonic_data_;
  sensor_msgs::msg::Image camera_data_;
  visualization_msgs::msg::MarkerArray environment_markers_;
  std::string robot_status_;
  
  // Trajectory history
  std::vector<geometry_msgs::msg::Point> trajectory_points_;
  int max_trajectory_points_;
  
  // ROS2 timers
  rclcpp::TimerBase::SharedPtr timer_;
  
  // ROS2 publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_markers_pub_;
  
  // ROS2 subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultrasonic_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr markers_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  
  // TF listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace nevil_simulation

#endif // NEVIL_SIMULATION_VISUALIZATION_NODE_HPP