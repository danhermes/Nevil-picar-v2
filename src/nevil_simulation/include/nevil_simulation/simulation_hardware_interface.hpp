#ifndef NEVIL_SIMULATION_HARDWARE_INTERFACE_HPP
#define NEVIL_SIMULATION_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <functional>
#include <chrono>
#include <map>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nevil_realtime/rt_thread_utils.hpp"

namespace nevil_simulation {

/**
 * @brief Simulation hardware interface for PiCar-X
 * 
 * This class provides a simulated version of the hardware interface
 * that mimics the behavior of the real PiCar-X hardware.
 */
class SimulationHardwareInterface {
public:
  /**
   * @brief Constructor
   * 
   * @param node ROS2 node for publishing simulated sensor data
   * @param physics_engine_enabled Whether to use the physics engine for simulation
   */
  explicit SimulationHardwareInterface(
    rclcpp::Node::SharedPtr node,
    bool physics_engine_enabled = true);
  
  /**
   * @brief Destructor
   */
  virtual ~SimulationHardwareInterface();
  
  /**
   * @brief Initialize the hardware interface
   * 
   * @return bool True if successful, false otherwise
   */
  bool initialize();
  
  /**
   * @brief Shutdown the hardware interface
   */
  void shutdown();
  
  /**
   * @brief Set motor speeds
   * 
   * @param left Left motor speed (-100 to 100)
   * @param right Right motor speed (-100 to 100)
   * @return bool True if successful, false otherwise
   */
  bool setMotorSpeed(int left, int right);
  
  /**
   * @brief Set servo angle
   * 
   * @param servo_id Servo ID (0 = steering, 1 = camera pan, 2 = camera tilt)
   * @param angle Angle in degrees
   * @return bool True if successful, false otherwise
   */
  bool setServoAngle(int servo_id, float angle);
  
  /**
   * @brief Get ultrasonic distance
   * 
   * @return float Distance in centimeters
   */
  float getUltrasonicDistance();
  
  /**
   * @brief Get infrared sensor readings
   * 
   * @return std::vector<bool> Sensor readings (true = obstacle detected)
   */
  std::vector<bool> getInfraredSensors();
  
  /**
   * @brief Get camera image
   * 
   * @return std::vector<uint8_t> Raw image data
   */
  std::vector<uint8_t> getCameraImage();
  
  /**
   * @brief Register a callback for hardware events
   * 
   * @param event_type Event type
   * @param callback Callback function
   * @return int Callback ID
   */
  int registerCallback(
    const std::string& event_type,
    std::function<void(const std::string&, const std::vector<uint8_t>&)> callback);
  
  /**
   * @brief Unregister a callback
   * 
   * @param callback_id Callback ID
   * @return bool True if successful, false otherwise
   */
  bool unregisterCallback(int callback_id);
  
  /**
   * @brief Update the simulation state
   * 
   * This method should be called periodically to update the simulation state
   * based on the current motor and servo settings.
   * 
   * @param dt Time step in seconds
   */
  void update(double dt);
  
  /**
   * @brief Set the robot position in the simulation
   * 
   * @param x X position in meters
   * @param y Y position in meters
   * @param theta Orientation in radians
   */
  void setRobotPose(double x, double y, double theta);
  
  /**
   * @brief Get the robot position in the simulation
   * 
   * @return std::tuple<double, double, double> X, Y, and orientation
   */
  std::tuple<double, double, double> getRobotPose();
  
  /**
   * @brief Add an obstacle to the simulation
   * 
   * @param type Obstacle type (box, cylinder, sphere)
   * @param x X position in meters
   * @param y Y position in meters
   * @param params Additional parameters (dimensions, radius, etc.)
   * @return int Obstacle ID
   */
  int addObstacle(
    const std::string& type,
    double x,
    double y,
    const std::map<std::string, double>& params);
  
  /**
   * @brief Remove an obstacle from the simulation
   * 
   * @param obstacle_id Obstacle ID
   * @return bool True if successful, false otherwise
   */
  bool removeObstacle(int obstacle_id);
  
  /**
   * @brief Clear all obstacles from the simulation
   */
  void clearObstacles();
  
  /**
   * @brief Load an environment from a YAML file
   * 
   * @param filename Environment file name
   * @return bool True if successful, false otherwise
   */
  bool loadEnvironment(const std::string& filename);

private:
  /**
   * @brief Simulate ultrasonic sensor readings
   * 
   * @return float Simulated distance in centimeters
   */
  float simulateUltrasonicSensor();
  
  /**
   * @brief Simulate infrared sensor readings
   * 
   * @return std::vector<bool> Simulated sensor readings
   */
  std::vector<bool> simulateInfraredSensors();
  
  /**
   * @brief Simulate camera image
   * 
   * @return std::vector<uint8_t> Simulated image data
   */
  std::vector<uint8_t> simulateCameraImage();
  
  /**
   * @brief Update robot position based on motor speeds
   * 
   * @param dt Time step in seconds
   */
  void updateRobotPosition(double dt);
  
  /**
   * @brief Check for collisions with obstacles
   * 
   * @return bool True if collision detected, false otherwise
   */
  bool checkCollisions();
  
  /**
   * @brief Publish sensor data to ROS2 topics
   */
  void publishSensorData();

  // ROS2 node
  rclcpp::Node::SharedPtr node_;
  
  // Publishers for simulated sensor data
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  
  // Simulation state
  bool initialized_;
  bool physics_engine_enabled_;
  int left_motor_speed_;
  int right_motor_speed_;
  std::vector<float> servo_angles_;
  
  // Robot state
  double robot_x_;
  double robot_y_;
  double robot_theta_;
  double robot_linear_velocity_;
  double robot_angular_velocity_;
  
  // Obstacle state
  std::map<int, std::tuple<std::string, double, double, std::map<std::string, double>>> obstacles_;
  int next_obstacle_id_;
  
  // Random number generator for sensor noise
  std::random_device rd_;
  std::mt19937 gen_;
  std::normal_distribution<> noise_dist_;
  
  // Mutex for thread safety
  std::mutex state_mutex_;
  
  // Callback management
  std::map<int, std::pair<std::string, std::function<void(const std::string&, const std::vector<uint8_t>&)>>> callbacks_;
  int next_callback_id_;
  std::mutex callbacks_mutex_;
};

} // namespace nevil_simulation

#endif // NEVIL_SIMULATION_HARDWARE_INTERFACE_HPP