#ifndef NEVIL_SIMULATION_VISUALIZATION_NODE_HPP_
#define NEVIL_SIMULATION_VISUALIZATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace nevil_simulation {

class VisualizationNode : public rclcpp::Node {
public:
  VisualizationNode();
};

}  // namespace nevil_simulation

#endif  // NEVIL_SIMULATION_VISUALIZATION_NODE_HPP_
