#ifndef NEVIL_SIMULATION_NODE_HPP
#define NEVIL_SIMULATION_NODE_HPP

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
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

#include "nevil_simulation/hardware_abstraction_layer.hpp"
#include "nevil_simulation/physics_engine.hpp"

namespace nevil_simulation {

/**
 * @brief Main simulation node for Nevil-picar
 * 
 * This node manages the simulation of the Nevil-picar robot,
 * including physics, sensors, and visualization.
 */
class SimulationNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor
   */
  SimulationNode();
  
  /**
   * @brief Destructor
   */
  virtual ~SimulationNode();

private:
  /**
   * @brief Initialize the simulation
   * 
   * @return bool True if successful, false otherwise
   */
  bool initialize();
  
  /**
   * @brief Load simulation parameters from ROS2 parameters
   */
  void loadParameters();
  
  /**
   * @brief Create publishers and subscribers
   */
  void createPubSub();
  
  /**
   * @brief Timer callback for simulation update
   */
  void timerCallback();
  
  /**
   * @brief Command velocity callback
   * 
   * @param msg Command velocity message
   */
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  
  /**
   * @brief System mode callback
   * 
   * @param msg System mode message
   */
  void systemModeCallback(const std_msgs::msg::String::SharedPtr msg);
  
  /**
   * @brief Reset simulation callback
   * 
   * @param msg Reset message
   */
  void resetCallback(const std_msgs::msg::String::SharedPtr msg);
  
  /**
   * @brief Load environment callback
   * 
   * @param msg Environment name message
   */
  void loadEnvironmentCallback(const std_msgs::msg::String::SharedPtr msg);
  
  /**
   * @brief Update the simulation state
   * 
   * @param dt Time step in seconds
   */
  void updateSimulation(double dt);
  
  /**
   * @brief Publish simulation state
   */
  void publishSimulationState();
  
  /**
   * @brief Publish TF transforms
   */
  void publishTransforms();
  
  /**
   * @brief Publish visualization markers
   */
  void publishVisualizationMarkers();
  
  /**
   * @brief Reset the simulation to initial state
   */
  void resetSimulation();
  
  /**
   * @brief Load an environment from a YAML file
   * 
   * @param environment_name Environment name
   * @return bool True if successful, false otherwise
   */
  bool loadEnvironment(const std::string& environment_name);

  // Hardware abstraction layer
  std::unique_ptr<HardwareAbstractionLayer> hardware_;
  
  // Physics engine
  std::unique_ptr<PhysicsEngine> physics_;
  
  // Simulation parameters
  double update_rate_;
  double physics_update_rate_;
  std::string default_environment_;
  bool auto_start_;
  bool visualization_enabled_;
  
  // Robot parameters
  double robot_mass_;
  double robot_width_;
  double robot_length_;
  double robot_wheel_radius_;
  double robot_max_linear_velocity_;
  double robot_max_angular_velocity_;
  
  // Simulation state
  bool initialized_;
  bool running_;
  std::string current_mode_;
  std::chrono::time_point<std::chrono::steady_clock> last_update_time_;
  
  // ROS2 timers
  rclcpp::TimerBase::SharedPtr timer_;
  
  // ROS2 publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  
  // ROS2 subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reset_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr load_environment_sub_;
  
  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

} // namespace nevil_simulation

#endif // NEVIL_SIMULATION_NODE_HPP