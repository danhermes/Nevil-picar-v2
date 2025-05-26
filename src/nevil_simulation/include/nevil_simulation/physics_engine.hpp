#ifndef NEVIL_SIMULATION_PHYSICS_ENGINE_HPP
#define NEVIL_SIMULATION_PHYSICS_ENGINE_HPP

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <functional>

namespace nevil_simulation {

/**
 * @brief Simple physics engine for PiCar-X simulation
 * 
 * This class provides a simple physics engine for simulating
 * the physical behavior of the PiCar-X robot.
 */
class PhysicsEngine {
public:
  /**
   * @brief Constructor
   */
  PhysicsEngine();
  
  /**
   * @brief Destructor
   */
  virtual ~PhysicsEngine();
  
  /**
   * @brief Initialize the physics engine
   * 
   * @return bool True if successful, false otherwise
   */
  bool initialize();
  
  /**
   * @brief Shutdown the physics engine
   */
  void shutdown();
  
  /**
   * @brief Update the physics simulation
   * 
   * @param dt Time step in seconds
   */
  void update(double dt);
  
  /**
   * @brief Set the robot properties
   * 
   * @param mass Mass in kg
   * @param width Width in meters
   * @param length Length in meters
   * @param wheel_radius Wheel radius in meters
   * @param max_linear_velocity Maximum linear velocity in m/s
   * @param max_angular_velocity Maximum angular velocity in rad/s
   */
  void setRobotProperties(
    double mass,
    double width,
    double length,
    double wheel_radius,
    double max_linear_velocity,
    double max_angular_velocity);
  
  /**
   * @brief Set the robot pose
   * 
   * @param x X position in meters
   * @param y Y position in meters
   * @param theta Orientation in radians
   */
  void setRobotPose(double x, double y, double theta);
  
  /**
   * @brief Get the robot pose
   * 
   * @return std::tuple<double, double, double> X, Y, and orientation
   */
  std::tuple<double, double, double> getRobotPose();
  
  /**
   * @brief Set the robot velocity
   * 
   * @param linear Linear velocity in m/s
   * @param angular Angular velocity in rad/s
   */
  void setRobotVelocity(double linear, double angular);
  
  /**
   * @brief Get the robot velocity
   * 
   * @return std::tuple<double, double> Linear and angular velocity
   */
  std::tuple<double, double> getRobotVelocity();
  
  /**
   * @brief Set the motor speeds
   * 
   * @param left Left motor speed (-100 to 100)
   * @param right Right motor speed (-100 to 100)
   */
  void setMotorSpeeds(int left, int right);
  
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
   * @brief Check for collisions with obstacles
   * 
   * @return bool True if collision detected, false otherwise
   */
  bool checkCollisions();
  
  /**
   * @brief Get the distance to the nearest obstacle in a specific direction
   * 
   * @param direction_angle Direction angle in radians (0 = forward)
   * @param max_distance Maximum detection distance in meters
   * @return double Distance to the nearest obstacle in meters
   */
  double getDistanceToObstacle(double direction_angle, double max_distance);
  
  /**
   * @brief Register a collision callback
   * 
   * @param callback Callback function
   * @return int Callback ID
   */
  int registerCollisionCallback(std::function<void(int)> callback);
  
  /**
   * @brief Unregister a collision callback
   * 
   * @param callback_id Callback ID
   * @return bool True if successful, false otherwise
   */
  bool unregisterCollisionCallback(int callback_id);

private:
  /**
   * @brief Update the robot position based on motor speeds
   * 
   * @param dt Time step in seconds
   */
  void updateRobotPosition(double dt);
  
  /**
   * @brief Calculate the robot velocity from motor speeds
   */
  void calculateRobotVelocity();
  
  /**
   * @brief Check for collision with a specific obstacle
   * 
   * @param obstacle_id Obstacle ID
   * @return bool True if collision detected, false otherwise
   */
  bool checkCollisionWithObstacle(int obstacle_id);
  
  /**
   * @brief Calculate distance to a specific obstacle
   * 
   * @param obstacle_id Obstacle ID
   * @param direction_angle Direction angle in radians
   * @return double Distance to the obstacle in meters
   */
  double calculateDistanceToObstacle(int obstacle_id, double direction_angle);

  // Robot properties
  double mass_;
  double width_;
  double length_;
  double wheel_radius_;
  double max_linear_velocity_;
  double max_angular_velocity_;
  
  // Robot state
  double robot_x_;
  double robot_y_;
  double robot_theta_;
  double robot_linear_velocity_;
  double robot_angular_velocity_;
  int left_motor_speed_;
  int right_motor_speed_;
  
  // Obstacle state
  std::map<int, std::tuple<std::string, double, double, std::map<std::string, double>>> obstacles_;
  int next_obstacle_id_;
  
  // Collision callbacks
  std::map<int, std::function<void(int)>> collision_callbacks_;
  int next_callback_id_;
  
  // Mutex for thread safety
  std::mutex state_mutex_;
  
  // Simulation state
  bool initialized_;
};

} // namespace nevil_simulation

#endif // NEVIL_SIMULATION_PHYSICS_ENGINE_HPP