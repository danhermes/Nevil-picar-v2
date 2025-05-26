#ifndef NEVIL_REALTIME_RT_EXECUTOR_HPP
#define NEVIL_REALTIME_RT_EXECUTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <memory>
#include <thread>
#include <vector>
#include <map>
#include <string>
#include <functional>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include "nevil_realtime/rt_thread_utils.hpp"

namespace nevil_realtime {

/**
 * @brief Configuration for a node in the RTExecutor
 */
struct RTNodeConfig {
  std::string node_name;
  SchedPolicy policy = SchedPolicy::FIFO;
  int priority = 80;
  std::vector<int> cpu_ids = {};
};

/**
 * @brief Real-time executor for ROS2 nodes
 * 
 * This executor extends ROS2's MultiThreadedExecutor to provide
 * deterministic execution of callbacks with proper priority handling.
 * It assigns each node to a dedicated thread with configurable
 * real-time scheduling parameters.
 */
class RTExecutor : public rclcpp::Executor {
public:
  /**
   * @brief Constructor
   * 
   * @param options Executor options
   */
  explicit RTExecutor(const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions());
  
  /**
   * @brief Destructor
   */
  virtual ~RTExecutor();
  
  /**
   * @brief Add a node to the executor with real-time configuration
   * 
   * @param node_handle Node to add
   * @param config Real-time configuration for the node
   */
  void add_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_handle,
    const RTNodeConfig & config);
  
  /**
   * @brief Add a node to the executor with default real-time configuration
   * 
   * @param node_handle Node to add
   */
  void add_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_handle) override;
  
  /**
   * @brief Remove a node from the executor
   * 
   * @param node_handle Node to remove
   */
  void remove_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_handle) override;
  
  /**
   * @brief Spin the executor
   * 
   * This method will block until the executor is shut down.
   */
  void spin() override;
  
  /**
   * @brief Spin the executor once
   * 
   * @param timeout Maximum time to wait for work
   */
  void spin_once(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1)) override;
  
  /**
   * @brief Spin the executor until future is complete
   * 
   * @param future Future to wait for
   * @param timeout Maximum time to wait
   * @return std::future_status Status of the future
   */
  template<typename FutureT>
  std::future_status spin_until_future_complete(
    const FutureT & future,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  
  /**
   * @brief Cancel all pending callbacks
   */
  void cancel() override;
  
  /**
   * @brief Get the name of the executor
   * 
   * @return std::string Executor name
   */
  std::string get_name() const;
  
  /**
   * @brief Set the name of the executor
   * 
   * @param name Executor name
   */
  void set_name(const std::string & name);
  
  /**
   * @brief Get the number of threads used by the executor
   * 
   * @return size_t Number of threads
   */
  size_t get_number_of_threads() const;

protected:
  /**
   * @brief Wait for work to become available
   * 
   * @param timeout Maximum time to wait
   * @return bool True if work is available, false if timeout
   */
  bool wait_for_work(std::chrono::nanoseconds timeout) override;
  
  /**
   * @brief Execute pending callbacks
   */
  void execute_ready_callbacks() override;

private:
  /**
   * @brief Thread function for a node
   * 
   * @param node_handle Node to execute
   * @param config Real-time configuration for the node
   */
  void run_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_handle,
    const RTNodeConfig & config);
  
  /**
   * @brief Start all node threads
   */
  void start_threads();
  
  /**
   * @brief Stop all node threads
   */
  void stop_threads();

  std::string name_;
  std::atomic<bool> running_;
  std::map<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr, RTNodeConfig> node_configs_;
  std::map<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr, std::thread> node_threads_;
  std::mutex wait_mutex_;
  std::condition_variable wait_cv_;
  std::mutex nodes_mutex_;
};

} // namespace nevil_realtime

#endif // NEVIL_REALTIME_RT_EXECUTOR_HPP