#include "nevil_realtime/rt_executor.hpp"
#include <iostream>
#include <chrono>
#include <algorithm>
#include <functional>

namespace nevil_realtime {

RTExecutor::RTExecutor(const rclcpp::ExecutorOptions & options)
: rclcpp::Executor(options),
  name_("RTExecutor"),
  running_(false) {
  // Check if we're running on a real-time kernel
  if (!RTThreadUtils::isRealTimeKernel()) {
    RCLCPP_WARN(
      rclcpp::get_logger("nevil_realtime"),
      "Running on a non-real-time kernel. Real-time performance will be limited.");
  }
}

RTExecutor::~RTExecutor() {
  // Stop all threads if they're still running
  if (running_.load()) {
    stop_threads();
  }
}

void RTExecutor::add_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_handle,
  const RTNodeConfig & config) {
  
  std::lock_guard<std::mutex> lock(nodes_mutex_);
  
  // Add the node to the executor
  rclcpp::Executor::add_node(node_handle);
  
  // Store the node's real-time configuration
  node_configs_[node_handle] = config;
  
  // If we're already running, start a thread for this node
  if (running_.load()) {
    node_threads_[node_handle] = std::thread(
      &RTExecutor::run_node, this, node_handle, config);
  }
  
  RCLCPP_INFO(
    rclcpp::get_logger("nevil_realtime"),
    "Added node '%s' with priority %d",
    node_handle->get_name(), config.priority);
}

void RTExecutor::add_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_handle) {
  
  // Create a default configuration for the node
  RTNodeConfig config;
  config.node_name = node_handle->get_name();
  
  // Use different default priorities based on node name patterns
  if (node_handle->get_name().find("motion_control") != std::string::npos) {
    config.priority = 90;
  } else if (node_handle->get_name().find("obstacle") != std::string::npos) {
    config.priority = 85;
  } else if (node_handle->get_name().find("sensor") != std::string::npos) {
    config.priority = 80;
  } else if (node_handle->get_name().find("camera") != std::string::npos) {
    config.priority = 70;
  } else if (node_handle->get_name().find("navigation") != std::string::npos) {
    config.priority = 75;
  } else {
    config.priority = 60;  // Default priority
  }
  
  add_node(node_handle, config);
}

void RTExecutor::remove_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_handle) {
  
  std::lock_guard<std::mutex> lock(nodes_mutex_);
  
  // If we're running, stop the thread for this node
  if (running_.load() && node_threads_.find(node_handle) != node_threads_.end()) {
    // We can't directly stop a thread, so we'll remove it from the executor
    // which will cause its thread function to exit
    rclcpp::Executor::remove_node(node_handle);
    
    // Wait for the thread to finish
    if (node_threads_[node_handle].joinable()) {
      node_threads_[node_handle].join();
    }
    
    // Remove the thread from our map
    node_threads_.erase(node_handle);
  } else {
    // If we're not running, just remove the node from the executor
    rclcpp::Executor::remove_node(node_handle);
  }
  
  // Remove the node's configuration
  node_configs_.erase(node_handle);
  
  RCLCPP_INFO(
    rclcpp::get_logger("nevil_realtime"),
    "Removed node '%s'",
    node_handle->get_name());
}

void RTExecutor::spin() {
  if (running_.load()) {
    throw std::runtime_error("spin() called while already spinning");
  }
  
  running_.store(true);
  
  // Start threads for all nodes
  start_threads();
  
  // Wait for shutdown
  std::unique_lock<std::mutex> lock(wait_mutex_);
  while (running_.load()) {
    wait_cv_.wait(lock);
  }
  
  // Stop all threads
  stop_threads();
}

void RTExecutor::spin_once(std::chrono::nanoseconds timeout) {
  // This is a simplified version that doesn't use real-time threads
  // It's mainly provided for compatibility with the Executor interface
  
  // Wait for work to be available
  if (!wait_for_work(timeout)) {
    return;
  }
  
  // Execute any ready callbacks
  execute_ready_callbacks();
}

template<typename FutureT>
std::future_status RTExecutor::spin_until_future_complete(
  const FutureT & future,
  std::chrono::nanoseconds timeout) {
  
  // If timeout is negative, wait indefinitely
  if (timeout < std::chrono::nanoseconds::zero()) {
    while (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      spin_once(std::chrono::milliseconds(100));
    }
    return std::future_status::ready;
  }
  
  // Otherwise, wait until timeout
  auto start = std::chrono::steady_clock::now();
  auto time_left = timeout;
  
  while (future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
    spin_once(std::min(time_left, std::chrono::milliseconds(10)));
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - start;
    
    if (elapsed > timeout) {
      return std::future_status::timeout;
    }
    
    time_left = timeout - elapsed;
  }
  
  return std::future_status::ready;
}

void RTExecutor::cancel() {
  // Stop spinning
  running_.store(false);
  
  // Notify any waiting threads
  wait_cv_.notify_all();
  
  // Cancel any pending callbacks
  rclcpp::Executor::cancel();
}

std::string RTExecutor::get_name() const {
  return name_;
}

void RTExecutor::set_name(const std::string & name) {
  name_ = name;
}

size_t RTExecutor::get_number_of_threads() const {
  return node_threads_.size();
}

bool RTExecutor::wait_for_work(std::chrono::nanoseconds timeout) {
  // This is called by spin_once, but our real-time threads handle this differently
  // This implementation is mainly for compatibility
  
  // Check if we have any ready callbacks
  if (rclcpp::Executor::get_number_of_ready_callbacks() > 0) {
    return true;
  }
  
  // Wait for work to be available
  std::unique_lock<std::mutex> lock(wait_mutex_);
  
  // If timeout is negative, wait indefinitely
  if (timeout < std::chrono::nanoseconds::zero()) {
    wait_cv_.wait(lock, [this]() {
      return rclcpp::Executor::get_number_of_ready_callbacks() > 0;
    });
    return true;
  }
  
  // Otherwise, wait until timeout
  return wait_cv_.wait_for(lock, timeout, [this]() {
    return rclcpp::Executor::get_number_of_ready_callbacks() > 0;
  });
}

void RTExecutor::execute_ready_callbacks() {
  // Execute any ready callbacks
  rclcpp::Executor::execute_ready_callbacks();
}

void RTExecutor::run_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_handle,
  const RTNodeConfig & config) {
  
  // Set thread name
  RTThreadUtils::setThreadName("rt_" + config.node_name);
  
  // Lock memory to prevent paging
  RTThreadUtils::lockMemory();
  
  // Set thread priority
  RTThreadUtils::setThreadPriority(config.policy, config.priority);
  
  // Set CPU affinity if specified
  if (!config.cpu_ids.empty()) {
    RTThreadUtils::setThreadAffinity(config.cpu_ids);
  }
  
  RCLCPP_INFO(
    rclcpp::get_logger("nevil_realtime"),
    "Started real-time thread for node '%s' with priority %d",
    node_handle->get_name(), config.priority);
  
  // Run until the executor is stopped or the node is removed
  while (running_.load()) {
    // Check if the node is still in the executor
    {
      std::lock_guard<std::mutex> lock(nodes_mutex_);
      if (node_configs_.find(node_handle) == node_configs_.end()) {
        break;
      }
    }
    
    // Process callbacks for this node
    // Note: This is a simplified version that doesn't handle all the
    // complexities of the Executor::spin_node_once() method
    
    // Wait for work to be available (with a timeout to check if we should exit)
    wait_for_work(std::chrono::milliseconds(100));
    
    // Execute any ready callbacks for this node
    execute_ready_callbacks();
  }
  
  RCLCPP_INFO(
    rclcpp::get_logger("nevil_realtime"),
    "Stopped real-time thread for node '%s'",
    node_handle->get_name());
}

void RTExecutor::start_threads() {
  std::lock_guard<std::mutex> lock(nodes_mutex_);
  
  // Start a thread for each node
  for (const auto & node_config : node_configs_) {
    auto node_handle = node_config.first;
    auto config = node_config.second;
    
    node_threads_[node_handle] = std::thread(
      &RTExecutor::run_node, this, node_handle, config);
  }
}

void RTExecutor::stop_threads() {
  std::lock_guard<std::mutex> lock(nodes_mutex_);
  
  // Stop all threads
  for (auto & thread_pair : node_threads_) {
    if (thread_pair.second.joinable()) {
      thread_pair.second.join();
    }
  }
  
  // Clear the threads map
  node_threads_.clear();
}

} // namespace nevil_realtime