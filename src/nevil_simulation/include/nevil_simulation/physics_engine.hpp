#ifndef NEVIL_SIMULATION_PHYSICS_ENGINE_HPP_
#define NEVIL_SIMULATION_PHYSICS_ENGINE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace nevil_simulation {

class PhysicsEngine {
public:
  PhysicsEngine();
  void configure();
  void start();
  void stop();
  void cleanup();
};

}  // namespace nevil_simulation

#endif  // NEVIL_SIMULATION_PHYSICS_ENGINE_HPP_
