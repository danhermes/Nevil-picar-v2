#ifndef NEVIL_SIMULATION_ENVIRONMENT_GENERATOR_HPP
#define NEVIL_SIMULATION_ENVIRONMENT_GENERATOR_HPP

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nevil_simulation {

/**
 * @brief Environment generator for Nevil-picar simulation
 * 
 * This class provides functionality for generating and managing
 * virtual environments for the Nevil-picar simulation.
 */
class EnvironmentGenerator {
public:
  /**
   * @brief Constructor
   * 
   * @param node ROS2 node for parameter access
   */
  explicit EnvironmentGenerator(rclcpp::Node::SharedPtr node);
  
  /**
   * @brief Destructor
   */
  virtual ~EnvironmentGenerator();
  
  /**
   * @brief Initialize the environment generator
   * 
   * @return bool True if successful, false otherwise
   */
  bool initialize();
  
  /**
   * @brief Load an environment from a YAML file
   * 
   * @param environment_name Environment name
   * @return bool True if successful, false otherwise
   */
  bool loadEnvironment(const std::string& environment_name);
  
  /**
   * @brief Generate a random environment
   * 
   * @param size_x Environment size in X direction (meters)
   * @param size_y Environment size in Y direction (meters)
   * @param num_obstacles Number of obstacles to generate
   * @return bool True if successful, false otherwise
   */
  bool generateRandomEnvironment(double size_x, double size_y, int num_obstacles);
  
  /**
   * @brief Generate an obstacle course environment
   * 
   * @param difficulty Difficulty level (1-5)
   * @return bool True if successful, false otherwise
   */
  bool generateObstacleCourse(int difficulty);
  
  /**
   * @brief Generate a maze environment
   * 
   * @param size Size of the maze (cells)
   * @param complexity Complexity of the maze (0-1)
   * @param density Density of the maze (0-1)
   * @return bool True if successful, false otherwise
   */
  bool generateMaze(int size, double complexity, double density);
  
  /**
   * @brief Get the environment obstacles
   * 
   * @return std::vector<std::tuple<std::string, double, double, std::map<std::string, double>>> Obstacles
   */
  std::vector<std::tuple<std::string, double, double, std::map<std::string, double>>> getObstacles();
  
  /**
   * @brief Get the environment size
   * 
   * @return std::tuple<double, double> Environment size (X, Y)
   */
  std::tuple<double, double> getEnvironmentSize();
  
  /**
   * @brief Get the environment name
   * 
   * @return std::string Environment name
   */
  std::string getEnvironmentName();
  
  /**
   * @brief Get the environment description
   * 
   * @return std::string Environment description
   */
  std::string getEnvironmentDescription();
  
  /**
   * @brief Get the environment visualization markers
   * 
   * @return visualization_msgs::msg::MarkerArray Environment markers
   */
  visualization_msgs::msg::MarkerArray getVisualizationMarkers();
  
  /**
   * @brief Save the current environment to a YAML file
   * 
   * @param filename Filename to save to
   * @return bool True if successful, false otherwise
   */
  bool saveEnvironment(const std::string& filename);
  
  /**
   * @brief Clear the current environment
   */
  void clearEnvironment();

private:
  /**
   * @brief Parse an environment YAML file
   * 
   * @param yaml_path Path to the YAML file
   * @return bool True if successful, false otherwise
   */
  bool parseEnvironmentYaml(const std::string& yaml_path);
  
  /**
   * @brief Create visualization markers for the environment
   */
  void createVisualizationMarkers();
  
  /**
   * @brief Create a box obstacle
   * 
   * @param x X position
   * @param y Y position
   * @param width Width
   * @param length Length
   * @param height Height
   * @param color_r Red component (0-1)
   * @param color_g Green component (0-1)
   * @param color_b Blue component (0-1)
   * @param color_a Alpha component (0-1)
   */
  void createBoxObstacle(
    double x, double y,
    double width, double length, double height,
    double color_r, double color_g, double color_b, double color_a);
  
  /**
   * @brief Create a cylinder obstacle
   * 
   * @param x X position
   * @param y Y position
   * @param radius Radius
   * @param height Height
   * @param color_r Red component (0-1)
   * @param color_g Green component (0-1)
   * @param color_b Blue component (0-1)
   * @param color_a Alpha component (0-1)
   */
  void createCylinderObstacle(
    double x, double y,
    double radius, double height,
    double color_r, double color_g, double color_b, double color_a);
  
  /**
   * @brief Create a sphere obstacle
   * 
   * @param x X position
   * @param y Y position
   * @param radius Radius
   * @param color_r Red component (0-1)
   * @param color_g Green component (0-1)
   * @param color_b Blue component (0-1)
   * @param color_a Alpha component (0-1)
   */
  void createSphereObstacle(
    double x, double y,
    double radius,
    double color_r, double color_g, double color_b, double color_a);

  // ROS2 node
  rclcpp::Node::SharedPtr node_;
  
  // Environment data
  std::string environment_name_;
  std::string environment_description_;
  double environment_size_x_;
  double environment_size_y_;
  std::vector<std::tuple<std::string, double, double, std::map<std::string, double>>> obstacles_;
  
  // Visualization markers
  visualization_msgs::msg::MarkerArray visualization_markers_;
  
  // Initialization state
  bool initialized_;
};

} // namespace nevil_simulation

#endif // NEVIL_SIMULATION_ENVIRONMENT_GENERATOR_HPP