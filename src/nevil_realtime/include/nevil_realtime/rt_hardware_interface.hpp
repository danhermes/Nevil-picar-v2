#ifndef NEVIL_REALTIME_RT_HARDWARE_INTERFACE_HPP
#define NEVIL_REALTIME_RT_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <functional>
#include <chrono>

#include "nevil_realtime/rt_thread_utils.hpp"

namespace nevil_realtime {

/**
 * @brief Real-time safe hardware interface for PiCar-X
 * 
 * This class provides low-latency access to the PiCar-X hardware
 * with proper mutex handling and bounded execution time.
 */
class RTHardwareInterface {
public:
  /**
   * @brief Constructor
   * 
   * @param simulation_mode Whether to run in simulation mode
   */
  explicit RTHardwareInterface(bool simulation_mode = false);
  
  /**
   * @brief Destructor
   */
  virtual ~RTHardwareInterface();
  
  /**
   * @brief Initialize the hardware interface
   * 
   * @return bool True if successful, false otherwise
   */
  bool initialize();
  
  /**
   * @brief Shutdown the hardware interface
   */
  void shutdown();
  
  /**
   * @brief Set motor speeds
   * 
   * @param left Left motor speed (-100 to 100)
   * @param right Right motor speed (-100 to 100)
   * @return bool True if successful, false otherwise
   */
  bool setMotorSpeed(int left, int right);
  
  /**
   * @brief Set servo angle
   * 
   * @param servo_id Servo ID (0 = steering, 1 = camera pan, 2 = camera tilt)
   * @param angle Angle in degrees
   * @return bool True if successful, false otherwise
   */
  bool setServoAngle(int servo_id, float angle);
  
  /**
   * @brief Get ultrasonic distance
   * 
   * @return float Distance in centimeters
   */
  float getUltrasonicDistance();
  
  /**
   * @brief Get infrared sensor readings
   * 
   * @return std::vector<bool> Sensor readings (true = obstacle detected)
   */
  std::vector<bool> getInfraredSensors();
  
  /**
   * @brief Get camera image
   * 
   * @return std::vector<uint8_t> Raw image data
   */
  std::vector<uint8_t> getCameraImage();
  
  /**
   * @brief Register a callback for hardware events
   * 
   * @param event_type Event type
   * @param callback Callback function
   * @return int Callback ID
   */
  int registerCallback(
    const std::string& event_type,
    std::function<void(const std::string&, const std::vector<uint8_t>&)> callback);
  
  /**
   * @brief Unregister a callback
   * 
   * @param callback_id Callback ID
   * @return bool True if successful, false otherwise
   */
  bool unregisterCallback(int callback_id);
  
  /**
   * @brief Get the execution time statistics for a hardware operation
   * 
   * @param operation_name Operation name
   * @return std::tuple<double, double, double> Min, max, and average execution time in microseconds
   */
  std::tuple<double, double, double> getExecutionTimeStats(const std::string& operation_name);
  
  /**
   * @brief Reset the execution time statistics
   */
  void resetExecutionTimeStats();
  
  /**
   * @brief Check if the hardware interface is in simulation mode
   * 
   * @return bool True if in simulation mode, false otherwise
   */
  bool isSimulationMode() const;
  
  /**
   * @brief Set the simulation mode
   * 
   * @param simulation_mode Whether to run in simulation mode
   */
  void setSimulationMode(bool simulation_mode);

private:
  /**
   * @brief Measure the execution time of a hardware operation
   * 
   * @param operation_name Operation name
   * @param func Function to measure
   * @return auto Return value of the function
   */
  template<typename Func>
  auto measureExecutionTime(const std::string& operation_name, Func&& func);
  
  /**
   * @brief Update the execution time statistics for a hardware operation
   * 
   * @param operation_name Operation name
   * @param execution_time Execution time in microseconds
   */
  void updateExecutionTimeStats(
    const std::string& operation_name,
    double execution_time);

  bool simulation_mode_;
  bool initialized_;
  
  // Mutex for hardware access
  RTMutex hardware_mutex_;
  
  // Execution time statistics
  std::map<std::string, std::tuple<double, double, double>> execution_time_stats_;
  std::map<std::string, int> execution_time_count_;
  RTMutex stats_mutex_;
  
  // Callback management
  std::map<int, std::pair<std::string, std::function<void(const std::string&, const std::vector<uint8_t>&)>>> callbacks_;
  int next_callback_id_;
  RTMutex callbacks_mutex_;
  
  // Hardware handles (would be replaced with actual hardware drivers)
  void* motor_handle_;
  void* servo_handle_;
  void* ultrasonic_handle_;
  void* infrared_handle_;
  void* camera_handle_;
};

} // namespace nevil_realtime

#endif // NEVIL_REALTIME_RT_HARDWARE_INTERFACE_HPP