#ifndef NEVIL_SIMULATION_SIMULATION_HARDWARE_INTERFACE_HPP_
#define NEVIL_SIMULATION_SIMULATION_HARDWARE_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace nevil_simulation {

class SimulationHardwareInterface {
public:
  SimulationHardwareInterface();
  void configure();
  void start();
  void stop();
  void cleanup();
};

}  // namespace nevil_simulation

#endif  // NEVIL_SIMULATION_SIMULATION_HARDWARE_INTERFACE_HPP_
