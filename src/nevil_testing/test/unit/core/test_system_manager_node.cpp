#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nevil_interfaces/msg/system_status.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class TestSystemManagerNode : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    
    // Create a test node
    test_node_ = std::make_shared<rclcpp::Node>("test_system_manager_node");
    
    // Create a subscription to system status
    system_status_sub_ = test_node_->create_subscription<nevil_interfaces::msg::SystemStatus>(
      "/system_status", 10,
      [this](const nevil_interfaces::msg::SystemStatus::SharedPtr msg) {
        system_status_ = msg;
      });
    
    // Create publishers for testing
    mode_change_pub_ = test_node_->create_publisher<std_msgs::msg::String>("/system/change_mode", 10);
    error_pub_ = test_node_->create_publisher<std_msgs::msg::String>("/system/error", 10);
    
    // Wait for the system manager node to be ready
    ASSERT_TRUE(wait_for_system_manager());
  }

  void TearDown() override {
    // Shutdown ROS
    rclcpp::shutdown();
  }

  bool wait_for_system_manager(std::chrono::seconds timeout = std::chrono::seconds(5)) {
    auto start = std::chrono::steady_clock::now();
    rclcpp::Rate rate(10);  // 10 Hz
    
    while (rclcpp::ok()) {
      rclcpp::spin_some(test_node_);
      
      if (system_status_) {
        return true;
      }
      
      rate.sleep();
      
      auto now = std::chrono::steady_clock::now();
      if (now - start > timeout) {
        return false;
      }
    }
    
    return false;
  }

  std::shared_ptr<rclcpp::Node> test_node_;
  rclcpp::Subscription<nevil_interfaces::msg::SystemStatus>::SharedPtr system_status_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_change_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub_;
  nevil_interfaces::msg::SystemStatus::SharedPtr system_status_;
};

TEST_F(TestSystemManagerNode, TestSystemStatusPublishing) {
  // Verify that we received a system status message
  ASSERT_TRUE(system_status_ != nullptr);
  
  // Verify the system status message fields
  EXPECT_FALSE(system_status_->header.stamp.sec == 0 && system_status_->header.stamp.nanosec == 0);
  EXPECT_FALSE(system_status_->mode.empty());
  EXPECT_GE(system_status_->battery_level, 0.0f);
  EXPECT_LE(system_status_->battery_level, 1.0f);
}

TEST_F(TestSystemManagerNode, TestSystemModeChange) {
  // Verify initial mode
  ASSERT_TRUE(system_status_ != nullptr);
  std::string initial_mode = system_status_->mode;
  
  // Create a message to change the mode
  auto mode_msg = std::make_unique<std_msgs::msg::String>();
  mode_msg->data = "autonomous";
  
  // Publish the message
  mode_change_pub_->publish(std::move(mode_msg));
  
  // Wait for the mode to change
  bool mode_changed = false;
  rclcpp::Rate rate(10);  // 10 Hz
  auto start = std::chrono::steady_clock::now();
  
  while (rclcpp::ok()) {
    rclcpp::spin_some(test_node_);
    
    if (system_status_ && system_status_->mode == "autonomous") {
      mode_changed = true;
      break;
    }
    
    rate.sleep();
    
    auto now = std::chrono::steady_clock::now();
    if (now - start > std::chrono::seconds(1)) {
      break;
    }
  }
  
  // Verify the mode changed
  EXPECT_TRUE(mode_changed);
  EXPECT_EQ(system_status_->mode, "autonomous");
  
  // Change back to the initial mode
  mode_msg = std::make_unique<std_msgs::msg::String>();
  mode_msg->data = initial_mode;
  
  // Publish the message
  mode_change_pub_->publish(std::move(mode_msg));
  
  // Wait for the mode to change back
  bool mode_changed_back = false;
  start = std::chrono::steady_clock::now();
  
  while (rclcpp::ok()) {
    rclcpp::spin_some(test_node_);
    
    if (system_status_ && system_status_->mode == initial_mode) {
      mode_changed_back = true;
      break;
    }
    
    rate.sleep();
    
    auto now = std::chrono::steady_clock::now();
    if (now - start > std::chrono::seconds(1)) {
      break;
    }
  }
  
  // Verify the mode changed back
  EXPECT_TRUE(mode_changed_back);
  EXPECT_EQ(system_status_->mode, initial_mode);
}

TEST_F(TestSystemManagerNode, TestErrorHandling) {
  // Verify initial error state
  ASSERT_TRUE(system_status_ != nullptr);
  bool initial_has_errors = system_status_->has_errors;
  
  // Create a message to report an error
  auto error_msg = std::make_unique<std_msgs::msg::String>();
  error_msg->data = "test_error";
  
  // Publish the message
  error_pub_->publish(std::move(error_msg));
  
  // Wait for the error to be processed
  bool error_processed = false;
  rclcpp::Rate rate(10);  // 10 Hz
  auto start = std::chrono::steady_clock::now();
  
  while (rclcpp::ok()) {
    rclcpp::spin_some(test_node_);
    
    if (system_status_ && system_status_->has_errors) {
      // Check if the error code is in the list
      bool found_error = false;
      for (const auto& error_code : system_status_->error_codes) {
        if (error_code == "test_error") {
          found_error = true;
          break;
        }
      }
      
      if (found_error) {
        error_processed = true;
        break;
      }
    }
    
    rate.sleep();
    
    auto now = std::chrono::steady_clock::now();
    if (now - start > std::chrono::seconds(1)) {
      break;
    }
  }
  
  // Verify the error was processed
  EXPECT_TRUE(error_processed);
  EXPECT_TRUE(system_status_->has_errors);
  
  // Check if the error code is in the list
  bool found_error = false;
  for (const auto& error_code : system_status_->error_codes) {
    if (error_code == "test_error") {
      found_error = true;
      break;
    }
  }
  EXPECT_TRUE(found_error);
}

TEST_F(TestSystemManagerNode, TestPerformanceMetrics) {
  // Verify that performance metrics are reported
  ASSERT_TRUE(system_status_ != nullptr);
  
  // Verify CPU usage
  EXPECT_GE(system_status_->cpu_usage, 0.0f);
  EXPECT_LE(system_status_->cpu_usage, 1.0f);
  
  // Verify memory usage
  EXPECT_GE(system_status_->memory_usage, 0.0f);
  EXPECT_LE(system_status_->memory_usage, 1.0f);
  
  // Verify disk usage
  EXPECT_GE(system_status_->disk_usage, 0.0f);
  EXPECT_LE(system_status_->disk_usage, 1.0f);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}