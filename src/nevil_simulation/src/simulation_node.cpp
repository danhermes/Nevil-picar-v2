#include <memory>
#include <string>
#include <vector>
#include <map>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nevil_simulation/simulation_node.hpp"
#include "nevil_simulation/hardware_abstraction_layer.hpp"
#include "nevil_simulation/physics_engine.hpp"

namespace nevil_simulation {

SimulationNode::SimulationNode()
: Node("simulation_node")
{
  RCLCPP_INFO(this->get_logger(), "Initializing Simulation Node");
  
  // Initialize parameters
  this->declare_parameter("use_sim", true);
  this->declare_parameter("default_environment", "empty");
  this->declare_parameter("update_rate", 30.0);
  this->declare_parameter("physics_update_rate", 100.0);
  this->declare_parameter("auto_start", true);
  this->declare_parameter("visualization_enabled", true);
  
  this->declare_parameter("robot.mass", 1.5);
  this->declare_parameter("robot.width", 0.15);
  this->declare_parameter("robot.length", 0.2);
  this->declare_parameter("robot.wheel_radius", 0.03);
  this->declare_parameter("robot.max_linear_velocity", 0.5);
  this->declare_parameter("robot.max_angular_velocity", 1.0);
  
  // Load parameters
  loadParameters();
  
  // Create publishers and subscribers
  createPubSub();
  
  // Initialize hardware abstraction layer and physics engine
  hardware_ = std::make_unique<HardwareAbstractionLayer>(
    shared_from_this(),
    this->get_parameter("use_sim").as_bool()
  );
  
  physics_ = std::make_unique<PhysicsEngine>();
  
  // Set robot properties in physics engine
  physics_->setRobotProperties(
    robot_mass_,
    robot_width_,
    robot_length_,
    robot_wheel_radius_,
    robot_max_linear_velocity_,
    robot_max_angular_velocity_
  );
  
  // Initialize hardware and physics
  if (!initialize()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize simulation");
    return;
  }
  
  // Create TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  
  // Create timer for simulation update
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)),
    std::bind(&SimulationNode::timerCallback, this)
  );
  
  // Load default environment
  loadEnvironment(this->get_parameter("default_environment").as_string());
  
  // Start simulation if auto_start is enabled
  if (this->get_parameter("auto_start").as_bool()) {
    running_ = true;
    RCLCPP_INFO(this->get_logger(), "Simulation started automatically");
  }
  
  RCLCPP_INFO(this->get_logger(), "Simulation Node initialized");
}

SimulationNode::~SimulationNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Simulation Node");
  
  // Stop simulation
  running_ = false;
  
  // Shutdown hardware and physics
  if (hardware_) {
    hardware_->shutdown();
  }
  
  if (physics_) {
    physics_->shutdown();
  }
}

bool SimulationNode::initialize()
{
  // Initialize hardware abstraction layer
  if (!hardware_->initialize()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize hardware abstraction layer");
    return false;
  }
  
  // Initialize physics engine
  if (!physics_->initialize()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize physics engine");
    return false;
  }
  
  // Set initial state
  initialized_ = true;
  running_ = false;
  current_mode_ = "standby";
  last_update_time_ = this->now();
  
  return true;
}

void SimulationNode::loadParameters()
{
  // Get simulation parameters
  update_rate_ = this->get_parameter("update_rate").as_double();
  physics_update_rate_ = this->get_parameter("physics_update_rate").as_double();
  default_environment_ = this->get_parameter("default_environment").as_string();
  auto_start_ = this->get_parameter("auto_start").as_bool();
  visualization_enabled_ = this->get_parameter("visualization_enabled").as_bool();
  
  // Get robot parameters
  robot_mass_ = this->get_parameter("robot.mass").as_double();
  robot_width_ = this->get_parameter("robot.width").as_double();
  robot_length_ = this->get_parameter("robot.length").as_double();
  robot_wheel_radius_ = this->get_parameter("robot.wheel_radius").as_double();
  robot_max_linear_velocity_ = this->get_parameter("robot.max_linear_velocity").as_double();
  robot_max_angular_velocity_ = this->get_parameter("robot.max_angular_velocity").as_double();
}

void SimulationNode::createPubSub()
{
  // Create publishers
  camera_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/camera/image_raw", 10);
  
  ultrasonic_pub_ = this->create_publisher<sensor_msgs::msg::Range>(
    "/ultrasonic_data", 10);
  
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/robot_pose", 10);
  
  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/simulation/markers", 10);
  
  status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/simulation/status", 10);
  
  // Create subscribers
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&SimulationNode::cmdVelCallback, this, std::placeholders::_1));
  
  system_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/system_mode", 10, std::bind(&SimulationNode::systemModeCallback, this, std::placeholders::_1));
  
  reset_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/simulation/control_cmd", 10, std::bind(&SimulationNode::resetCallback, this, std::placeholders::_1));
  
  load_environment_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/simulation/environment_cmd", 10, std::bind(&SimulationNode::loadEnvironmentCallback, this, std::placeholders::_1));
}

void SimulationNode::timerCallback()
{
  if (!initialized_) {
    return;
  }
  
  // Calculate time since last update
  auto current_time = this->now();
  double dt = (current_time - last_update_time_).seconds();
  last_update_time_ = current_time;
  
  if (running_) {
    // Update simulation
    updateSimulation(dt);
    
    // Publish simulation state
    publishSimulationState();
    
    // Publish TF transforms
    publishTransforms();
    
    // Publish visualization markers if enabled
    if (visualization_enabled_) {
      publishVisualizationMarkers();
    }
  }
}

void SimulationNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!initialized_ || !running_) {
    return;
  }
  
  // Convert linear and angular velocity to motor speeds
  double linear_velocity = msg->linear.x;
  double angular_velocity = msg->angular.z;
  
  // Apply limits
  linear_velocity = std::min(std::max(linear_velocity, -robot_max_linear_velocity_), robot_max_linear_velocity_);
  angular_velocity = std::min(std::max(angular_velocity, -robot_max_angular_velocity_), robot_max_angular_velocity_);
  
  // Calculate left and right motor speeds (-100 to 100)
  double wheel_base = robot_width_;
  double left_speed = (linear_velocity - angular_velocity * wheel_base / 2.0) / robot_max_linear_velocity_ * 100.0;
  double right_speed = (linear_velocity + angular_velocity * wheel_base / 2.0) / robot_max_linear_velocity_ * 100.0;
  
  // Apply limits to motor speeds
  left_speed = std::min(std::max(left_speed, -100.0), 100.0);
  right_speed = std::min(std::max(right_speed, -100.0), 100.0);
  
  // Set motor speeds in hardware abstraction layer
  hardware_->setMotorSpeed(static_cast<int>(left_speed), static_cast<int>(right_speed));
  
  // Set robot velocity in physics engine
  physics_->setRobotVelocity(linear_velocity, angular_velocity);
}

void SimulationNode::systemModeCallback(const std_msgs::msg::String::SharedPtr msg)
{
  current_mode_ = msg->data;
  
  // If system is in standby mode, stop the robot
  if (current_mode_ == "standby") {
    hardware_->setMotorSpeed(0, 0);
    physics_->setRobotVelocity(0.0, 0.0);
  }
}

void SimulationNode::resetCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string command = msg->data;
  
  if (command == "start") {
    running_ = true;
    RCLCPP_INFO(this->get_logger(), "Simulation started");
  } else if (command == "stop") {
    running_ = false;
    hardware_->setMotorSpeed(0, 0);
    physics_->setRobotVelocity(0.0, 0.0);
    RCLCPP_INFO(this->get_logger(), "Simulation stopped");
  } else if (command == "reset") {
    resetSimulation();
    RCLCPP_INFO(this->get_logger(), "Simulation reset");
  } else if (command == "pause") {
    running_ = false;
    RCLCPP_INFO(this->get_logger(), "Simulation paused");
  } else if (command == "resume") {
    running_ = true;
    RCLCPP_INFO(this->get_logger(), "Simulation resumed");
  }
}

void SimulationNode::loadEnvironmentCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string environment_name = msg->data;
  loadEnvironment(environment_name);
}

void SimulationNode::updateSimulation(double dt)
{
  // Update physics engine
  physics_->update(dt);
  
  // Update hardware abstraction layer
  hardware_->update(dt);
  
  // Get robot pose from physics engine
  auto [x, y, theta] = physics_->getRobotPose();
  
  // Update robot pose in hardware abstraction layer
  hardware_->setRobotPose(x, y, theta);
}

void SimulationNode::publishSimulationState()
{
  // Get robot pose from physics engine
  auto [x, y, theta] = physics_->getRobotPose();
  
  // Publish robot pose
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = this->now();
  pose_msg.header.frame_id = "map";
  pose_msg.pose.position.x = x;
  pose_msg.pose.position.y = y;
  pose_msg.pose.position.z = 0.0;
  
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  pose_msg.pose.orientation = tf2::toMsg(q);
  
  pose_pub_->publish(pose_msg);
  
  // Publish ultrasonic data
  sensor_msgs::msg::Range range_msg;
  range_msg.header.stamp = this->now();
  range_msg.header.frame_id = "ultrasonic_link";
  range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  range_msg.field_of_view = 0.5;  // ~30 degrees
  range_msg.min_range = 0.02;     // 2cm
  range_msg.max_range = 4.0;      // 4m
  range_msg.range = hardware_->getUltrasonicDistance() / 100.0;  // Convert from cm to m
  
  ultrasonic_pub_->publish(range_msg);
  
  // Publish status
  std_msgs::msg::String status_msg;
  status_msg.data = "running:" + std::to_string(running_) + 
                   ",mode:" + current_mode_ + 
                   ",pose:[" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(theta) + "]";
  
  status_pub_->publish(status_msg);
}

void SimulationNode::publishTransforms()
{
  // Get robot pose from physics engine
  auto [x, y, theta] = physics_->getRobotPose();
  
  // Create transform for robot base
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->now();
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = 0.0;
  
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();
  
  // Publish transform
  tf_broadcaster_->sendTransform(transform);
  
  // Create transform for ultrasonic sensor
  geometry_msgs::msg::TransformStamped ultrasonic_transform;
  ultrasonic_transform.header.stamp = this->now();
  ultrasonic_transform.header.frame_id = "base_link";
  ultrasonic_transform.child_frame_id = "ultrasonic_link";
  
  ultrasonic_transform.transform.translation.x = robot_length_ / 2.0;
  ultrasonic_transform.transform.translation.y = 0.0;
  ultrasonic_transform.transform.translation.z = 0.05;
  
  q.setRPY(0.0, 0.0, 0.0);
  ultrasonic_transform.transform.rotation.x = q.x();
  ultrasonic_transform.transform.rotation.y = q.y();
  ultrasonic_transform.transform.rotation.z = q.z();
  ultrasonic_transform.transform.rotation.w = q.w();
  
  // Publish transform
  tf_broadcaster_->sendTransform(ultrasonic_transform);
  
  // Create transform for camera
  geometry_msgs::msg::TransformStamped camera_transform;
  camera_transform.header.stamp = this->now();
  camera_transform.header.frame_id = "base_link";
  camera_transform.child_frame_id = "camera_link";
  
  camera_transform.transform.translation.x = robot_length_ / 2.0;
  camera_transform.transform.translation.y = 0.0;
  camera_transform.transform.translation.z = 0.1;
  
  q.setRPY(0.0, 0.0, 0.0);
  camera_transform.transform.rotation.x = q.x();
  camera_transform.transform.rotation.y = q.y();
  camera_transform.transform.rotation.z = q.z();
  camera_transform.transform.rotation.w = q.w();
  
  // Publish transform
  tf_broadcaster_->sendTransform(camera_transform);
}

void SimulationNode::publishVisualizationMarkers()
{
  // This would be implemented to create and publish visualization markers
  // for the robot, sensors, and other elements of the simulation
}

void SimulationNode::resetSimulation()
{
  // Reset robot pose
  physics_->setRobotPose(0.0, 0.0, 0.0);
  physics_->setRobotVelocity(0.0, 0.0);
  
  // Reset hardware
  hardware_->setRobotPose(0.0, 0.0, 0.0);
  hardware_->setMotorSpeed(0, 0);
  
  // Reset state
  running_ = false;
  current_mode_ = "standby";
  last_update_time_ = this->now();
}

bool SimulationNode::loadEnvironment(const std::string& environment_name)
{
  // Clear existing obstacles
  physics_->clearObstacles();
  hardware_->clearObstacles();
  
  // Load environment
  bool success = hardware_->loadEnvironment(environment_name);
  
  if (success) {
    RCLCPP_INFO(this->get_logger(), "Loaded environment: %s", environment_name.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to load environment: %s", environment_name.c_str());
  }
  
  return success;
}

} // namespace nevil_simulation

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nevil_simulation::SimulationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}