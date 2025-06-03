#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace nevil_simulation {

class VisualizationNode : public rclcpp::Node {
public:
  VisualizationNode() : Node("visualization_node") {
    RCLCPP_INFO(this->get_logger(), "VisualizationNode initialized");
  }
};

}  // namespace nevil_simulation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nevil_simulation::VisualizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
