#include "nevil_realtime/rt_hardware_interface.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace nevil_realtime {

RTHardwareInterface::RTHardwareInterface(bool simulation_mode)
: simulation_mode_(simulation_mode),
  initialized_(false),
  next_callback_id_(0),
  motor_handle_(nullptr),
  servo_handle_(nullptr),
  ultrasonic_handle_(nullptr),
  infrared_handle_(nullptr),
  camera_handle_(nullptr) {
}

RTHardwareInterface::~RTHardwareInterface() {
  if (initialized_) {
    shutdown();
  }
}

bool RTHardwareInterface::initialize() {
  std::lock_guard<RTMutex> lock(hardware_mutex_);
  
  if (initialized_) {
    return true;  // Already initialized
  }
  
  if (simulation_mode_) {
    // In simulation mode, we don't need to initialize actual hardware
    initialized_ = true;
    return true;
  }
  
  // Initialize hardware components
  // This would be replaced with actual hardware initialization code
  try {
    // Initialize motors
    motor_handle_ = nullptr;  // Replace with actual initialization
    
    // Initialize servos
    servo_handle_ = nullptr;  // Replace with actual initialization
    
    // Initialize ultrasonic sensor
    ultrasonic_handle_ = nullptr;  // Replace with actual initialization
    
    // Initialize infrared sensors
    infrared_handle_ = nullptr;  // Replace with actual initialization
    
    // Initialize camera
    camera_handle_ = nullptr;  // Replace with actual initialization
    
    initialized_ = true;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Failed to initialize hardware: " << e.what() << std::endl;
    shutdown();
    return false;
  }
}

void RTHardwareInterface::shutdown() {
  std::lock_guard<RTMutex> lock(hardware_mutex_);
  
  if (!initialized_) {
    return;  // Already shut down
  }
  
  if (simulation_mode_) {
    // In simulation mode, we don't need to shut down actual hardware
    initialized_ = false;
    return;
  }
  
  // Shut down hardware components
  // This would be replaced with actual hardware shutdown code
  try {
    // Shut down motors
    motor_handle_ = nullptr;
    
    // Shut down servos
    servo_handle_ = nullptr;
    
    // Shut down ultrasonic sensor
    ultrasonic_handle_ = nullptr;
    
    // Shut down infrared sensors
    infrared_handle_ = nullptr;
    
    // Shut down camera
    camera_handle_ = nullptr;
    
    initialized_ = false;
  } catch (const std::exception& e) {
    std::cerr << "Failed to shut down hardware: " << e.what() << std::endl;
  }
}

bool RTHardwareInterface::setMotorSpeed(int left, int right) {
  return measureExecutionTime("setMotorSpeed", [this, left, right]() {
    std::lock_guard<RTMutex> lock(hardware_mutex_);
    
    if (!initialized_) {
      return false;
    }
    
    // Clamp values to valid range
    int clamped_left = std::max(-100, std::min(100, left));
    int clamped_right = std::max(-100, std::min(100, right));
    
    if (simulation_mode_) {
      // In simulation mode, just log the command
      return true;
    }
    
    // Set motor speeds
    // This would be replaced with actual hardware control code
    
    return true;
  });
}

bool RTHardwareInterface::setServoAngle(int servo_id, float angle) {
  return measureExecutionTime("setServoAngle", [this, servo_id, angle]() {
    std::lock_guard<RTMutex> lock(hardware_mutex_);
    
    if (!initialized_) {
      return false;
    }
    
    // Validate servo ID
    if (servo_id < 0 || servo_id > 2) {
      return false;
    }
    
    // Clamp angle to valid range
    float clamped_angle = std::max(-90.0f, std::min(90.0f, angle));
    
    if (simulation_mode_) {
      // In simulation mode, just log the command
      return true;
    }
    
    // Set servo angle
    // This would be replaced with actual hardware control code
    
    return true;
  });
}

float RTHardwareInterface::getUltrasonicDistance() {
  return measureExecutionTime("getUltrasonicDistance", [this]() {
    std::lock_guard<RTMutex> lock(hardware_mutex_);
    
    if (!initialized_) {
      return -1.0f;
    }
    
    if (simulation_mode_) {
      // In simulation mode, return a random value
      static std::random_device rd;
      static std::mt19937 gen(rd());
      static std::uniform_real_distribution<float> dist(10.0f, 200.0f);
      return dist(gen);
    }
    
    // Get ultrasonic distance
    // This would be replaced with actual hardware reading code
    
    return 100.0f;  // Default value
  });
}

std::vector<bool> RTHardwareInterface::getInfraredSensors() {
  return measureExecutionTime("getInfraredSensors", [this]() {
    std::lock_guard<RTMutex> lock(hardware_mutex_);
    
    if (!initialized_) {
      return std::vector<bool>();
    }
    
    if (simulation_mode_) {
      // In simulation mode, return random values
      static std::random_device rd;
      static std::mt19937 gen(rd());
      static std::uniform_int_distribution<int> dist(0, 1);
      
      std::vector<bool> readings(3);
      for (auto& reading : readings) {
        reading = (dist(gen) == 1);
      }
      
      return readings;
    }
    
    // Get infrared sensor readings
    // This would be replaced with actual hardware reading code
    
    return std::vector<bool>(3, false);  // Default value
  });
}

std::vector<uint8_t> RTHardwareInterface::getCameraImage() {
  return measureExecutionTime("getCameraImage", [this]() {
    std::lock_guard<RTMutex> lock(hardware_mutex_);
    
    if (!initialized_) {
      return std::vector<uint8_t>();
    }
    
    if (simulation_mode_) {
      // In simulation mode, return a dummy image
      // This is just a placeholder, not a real image
      std::vector<uint8_t> dummy_image(320 * 240 * 3, 128);
      return dummy_image;
    }
    
    // Get camera image
    // This would be replaced with actual hardware reading code
    
    return std::vector<uint8_t>();  // Default value
  });
}

int RTHardwareInterface::registerCallback(
  const std::string& event_type,
  std::function<void(const std::string&, const std::vector<uint8_t>&)> callback) {
  
  std::lock_guard<RTMutex> lock(callbacks_mutex_);
  
  int callback_id = next_callback_id_++;
  callbacks_[callback_id] = std::make_pair(event_type, callback);
  
  return callback_id;
}

bool RTHardwareInterface::unregisterCallback(int callback_id) {
  std::lock_guard<RTMutex> lock(callbacks_mutex_);
  
  auto it = callbacks_.find(callback_id);
  if (it == callbacks_.end()) {
    return false;
  }
  
  callbacks_.erase(it);
  return true;
}

std::tuple<double, double, double> RTHardwareInterface::getExecutionTimeStats(
  const std::string& operation_name) {
  
  std::lock_guard<RTMutex> lock(stats_mutex_);
  
  auto it = execution_time_stats_.find(operation_name);
  if (it == execution_time_stats_.end()) {
    return std::make_tuple(0.0, 0.0, 0.0);
  }
  
  return it->second;
}

void RTHardwareInterface::resetExecutionTimeStats() {
  std::lock_guard<RTMutex> lock(stats_mutex_);
  
  execution_time_stats_.clear();
  execution_time_count_.clear();
}

bool RTHardwareInterface::isSimulationMode() const {
  return simulation_mode_;
}

void RTHardwareInterface::setSimulationMode(bool simulation_mode) {
  // If we're changing modes, we need to reinitialize
  if (simulation_mode_ != simulation_mode) {
    if (initialized_) {
      shutdown();
    }
    
    simulation_mode_ = simulation_mode;
    
    if (initialized_) {
      initialize();
    }
  } else {
    simulation_mode_ = simulation_mode;
  }
}

template<typename Func>
auto RTHardwareInterface::measureExecutionTime(const std::string& operation_name, Func&& func) {
  auto start = std::chrono::high_resolution_clock::now();
  
  auto result = func();
  
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  
  updateExecutionTimeStats(operation_name, static_cast<double>(duration));
  
  return result;
}

void RTHardwareInterface::updateExecutionTimeStats(
  const std::string& operation_name,
  double execution_time) {
  
  std::lock_guard<RTMutex> lock(stats_mutex_);
  
  auto it = execution_time_stats_.find(operation_name);
  if (it == execution_time_stats_.end()) {
    // First execution of this operation
    execution_time_stats_[operation_name] = std::make_tuple(
      execution_time,  // min
      execution_time,  // max
      execution_time   // avg
    );
    execution_time_count_[operation_name] = 1;
  } else {
    // Update statistics
    double min_time = std::get<0>(it->second);
    double max_time = std::get<1>(it->second);
    double avg_time = std::get<2>(it->second);
    int count = execution_time_count_[operation_name];
    
    min_time = std::min(min_time, execution_time);
    max_time = std::max(max_time, execution_time);
    avg_time = (avg_time * count + execution_time) / (count + 1);
    
    execution_time_stats_[operation_name] = std::make_tuple(min_time, max_time, avg_time);
    execution_time_count_[operation_name] = count + 1;
  }
}

} // namespace nevil_realtime