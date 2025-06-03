#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace nevil_simulation {

class SimulationHardwareInterface {
public:
  SimulationHardwareInterface() {
    RCLCPP_INFO(rclcpp::get_logger("SimulationHardwareInterface"), "SimulationHardwareInterface initialized");
  }

  void configure() {
    RCLCPP_INFO(rclcpp::get_logger("SimulationHardwareInterface"), "SimulationHardwareInterface configured");
  }

  void start() {
    RCLCPP_INFO(rclcpp::get_logger("SimulationHardwareInterface"), "SimulationHardwareInterface started");
  }

  void stop() {
    RCLCPP_INFO(rclcpp::get_logger("SimulationHardwareInterface"), "SimulationHardwareInterface stopped");
  }

  void cleanup() {
    RCLCPP_INFO(rclcpp::get_logger("SimulationHardwareInterface"), "SimulationHardwareInterface cleaned up");
  }
};

}  // namespace nevil_simulation
