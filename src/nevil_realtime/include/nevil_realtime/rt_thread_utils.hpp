#ifndef NEVIL_REALTIME_RT_THREAD_UTILS_HPP
#define NEVIL_REALTIME_RT_THREAD_UTILS_HPP

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <thread>

namespace nevil_realtime {

/**
 * @brief Scheduling policy types for real-time threads
 */
enum class SchedPolicy {
  NORMAL = SCHED_OTHER,  // Standard Linux scheduling
  FIFO = SCHED_FIFO,     // First-in, first-out real-time scheduling
  RR = SCHED_RR          // Round-robin real-time scheduling
};

/**
 * @brief Utility class for managing real-time threads
 * 
 * This class provides utilities for creating and managing real-time threads
 * with specific scheduling policies, priorities, and CPU affinities.
 */
class RTThreadUtils {
public:
  /**
   * @brief Set the scheduling policy and priority for the current thread
   * 
   * @param policy Scheduling policy (NORMAL, FIFO, RR)
   * @param priority Thread priority (0-99 for real-time policies)
   * @return bool True if successful, false otherwise
   */
  static bool setThreadPriority(SchedPolicy policy, int priority);
  
  /**
   * @brief Set CPU affinity for the current thread
   * 
   * @param cpu_ids Vector of CPU IDs to which the thread should be pinned
   * @return bool True if successful, false otherwise
   */
  static bool setThreadAffinity(const std::vector<int>& cpu_ids);
  
  /**
   * @brief Lock all current and future memory to prevent paging
   * 
   * @return bool True if successful, false otherwise
   */
  static bool lockMemory();
  
  /**
   * @brief Create a real-time thread with specified parameters
   * 
   * @param func Function to execute in the thread
   * @param policy Scheduling policy
   * @param priority Thread priority
   * @param cpu_ids CPU affinity (empty for no specific affinity)
   * @return std::thread The created thread
   */
  static std::thread createRTThread(
    std::function<void()> func,
    SchedPolicy policy = SchedPolicy::FIFO,
    int priority = 80,
    const std::vector<int>& cpu_ids = {});
  
  /**
   * @brief Get the current thread's scheduling policy
   * 
   * @return SchedPolicy The current scheduling policy
   */
  static SchedPolicy getCurrentPolicy();
  
  /**
   * @brief Get the current thread's priority
   * 
   * @return int The current priority
   */
  static int getCurrentPriority();
  
  /**
   * @brief Get the maximum allowed priority for a given policy
   * 
   * @param policy The scheduling policy
   * @return int The maximum priority
   */
  static int getMaxPriority(SchedPolicy policy);
  
  /**
   * @brief Get the minimum allowed priority for a given policy
   * 
   * @param policy The scheduling policy
   * @return int The minimum priority
   */
  static int getMinPriority(SchedPolicy policy);
  
  /**
   * @brief Check if the current kernel supports real-time scheduling
   * 
   * @return bool True if supported, false otherwise
   */
  static bool isRealTimeKernel();
  
  /**
   * @brief Get the name of the current thread
   * 
   * @return std::string The thread name
   */
  static std::string getThreadName();
  
  /**
   * @brief Set the name of the current thread
   * 
   * @param name The name to set (max 15 characters)
   * @return bool True if successful, false otherwise
   */
  static bool setThreadName(const std::string& name);
};

/**
 * @brief Mutex with priority inheritance to prevent priority inversion
 * 
 * This mutex implementation uses the PTHREAD_PRIO_INHERIT protocol
 * to ensure that a thread holding the mutex temporarily inherits
 * the priority of the highest-priority thread waiting for the mutex.
 */
class RTMutex {
public:
  RTMutex();
  ~RTMutex();
  
  /**
   * @brief Lock the mutex
   */
  void lock();
  
  /**
   * @brief Try to lock the mutex without blocking
   * 
   * @return bool True if the lock was acquired, false otherwise
   */
  bool try_lock();
  
  /**
   * @brief Unlock the mutex
   */
  void unlock();
  
  /**
   * @brief Get the native pthread mutex handle
   * 
   * @return pthread_mutex_t* Pointer to the native mutex
   */
  pthread_mutex_t* native_handle();

private:
  pthread_mutex_t mutex_;
  pthread_mutexattr_t attr_;
};

/**
 * @brief Condition variable compatible with RTMutex
 */
class RTConditionVariable {
public:
  RTConditionVariable();
  ~RTConditionVariable();
  
  /**
   * @brief Notify one waiting thread
   */
  void notify_one();
  
  /**
   * @brief Notify all waiting threads
   */
  void notify_all();
  
  /**
   * @brief Wait for the condition variable to be notified
   * 
   * @param lock A locked RTMutex
   */
  void wait(std::unique_lock<RTMutex>& lock);
  
  /**
   * @brief Wait for the condition variable to be notified or until a timeout
   * 
   * @param lock A locked RTMutex
   * @param rel_time Relative timeout duration
   * @return bool False if the timeout expired, true otherwise
   */
  template<class Rep, class Period>
  bool wait_for(std::unique_lock<RTMutex>& lock,
                const std::chrono::duration<Rep, Period>& rel_time);
  
  /**
   * @brief Get the native pthread condition variable handle
   * 
   * @return pthread_cond_t* Pointer to the native condition variable
   */
  pthread_cond_t* native_handle();

private:
  pthread_cond_t cond_;
  pthread_condattr_t attr_;
};

} // namespace nevil_realtime

#endif // NEVIL_REALTIME_RT_THREAD_UTILS_HPP