#ifndef NEVIL_SIMULATION_HARDWARE_ABSTRACTION_LAYER_HPP
#define NEVIL_SIMULATION_HARDWARE_ABSTRACTION_LAYER_HPP

#include <memory>
#include <string>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nevil_realtime/rt_hardware_interface.hpp"
#include "nevil_simulation/simulation_hardware_interface.hpp"

namespace nevil_simulation {

/**
 * @brief Hardware abstraction layer for Nevil-picar
 * 
 * This class provides a unified interface for both real and simulated hardware,
 * allowing seamless switching between the two.
 */
class HardwareAbstractionLayer {
public:
  /**
   * @brief Constructor
   * 
   * @param node ROS2 node for publishing simulated sensor data
   * @param simulation_mode Whether to use simulated hardware
   */
  explicit HardwareAbstractionLayer(
    rclcpp::Node::SharedPtr node,
    bool simulation_mode = false);
  
  /**
   * @brief Destructor
   */
  virtual ~HardwareAbstractionLayer();
  
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
   * @brief Check if the hardware interface is in simulation mode
   * 
   * @return bool True if in simulation mode, false otherwise
   */
  bool isSimulationMode() const;
  
  /**
   * @brief Set the simulation mode
   * 
   * @param simulation_mode Whether to use simulated hardware
   * @return bool True if successful, false otherwise
   */
  bool setSimulationMode(bool simulation_mode);
  
  /**
   * @brief Update the simulation state (only in simulation mode)
   * 
   * @param dt Time step in seconds
   */
  void update(double dt);
  
  /**
   * @brief Set the robot position in the simulation (only in simulation mode)
   * 
   * @param x X position in meters
   * @param y Y position in meters
   * @param theta Orientation in radians
   * @return bool True if successful, false otherwise
   */
  bool setRobotPose(double x, double y, double theta);
  
  /**
   * @brief Get the robot position in the simulation (only in simulation mode)
   * 
   * @return std::tuple<double, double, double> X, Y, and orientation
   */
  std::tuple<double, double, double> getRobotPose();
  
  /**
   * @brief Add an obstacle to the simulation (only in simulation mode)
   * 
   * @param type Obstacle type (box, cylinder, sphere)
   * @param x X position in meters
   * @param y Y position in meters
   * @param params Additional parameters (dimensions, radius, etc.)
   * @return int Obstacle ID, or -1 if failed
   */
  int addObstacle(
    const std::string& type,
    double x,
    double y,
    const std::map<std::string, double>& params);
  
  /**
   * @brief Remove an obstacle from the simulation (only in simulation mode)
   * 
   * @param obstacle_id Obstacle ID
   * @return bool True if successful, false otherwise
   */
  bool removeObstacle(int obstacle_id);
  
  /**
   * @brief Clear all obstacles from the simulation (only in simulation mode)
   * 
   * @return bool True if successful, false otherwise
   */
  bool clearObstacles();
  
  /**
   * @brief Load an environment from a YAML file (only in simulation mode)
   * 
   * @param filename Environment file name
   * @return bool True if successful, false otherwise
   */
  bool loadEnvironment(const std::string& filename);

private:
  // ROS2 node
  rclcpp::Node::SharedPtr node_;
  
  // Hardware interfaces
  std::unique_ptr<nevil_realtime::RTHardwareInterface> real_hardware_;
  std::unique_ptr<SimulationHardwareInterface> sim_hardware_;
  
  // Current mode
  bool simulation_mode_;
  
  // Initialization state
  bool initialized_;
};

} // namespace nevil_simulation

#endif // NEVIL_SIMULATION_HARDWARE_ABSTRACTION_LAYER_HPP