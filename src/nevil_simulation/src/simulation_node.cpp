#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace nevil_simulation {

class SimulationNode : public rclcpp::Node {
public:
  SimulationNode() : Node("simulation_node") {
    RCLCPP_INFO(this->get_logger(), "SimulationNode initialized");
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SimulationNode::timer_callback, this));
  }

private:
  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "SimulationNode is running");
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nevil_simulation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nevil_simulation::SimulationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
