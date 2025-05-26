#include "nevil_realtime/rt_thread_utils.hpp"
#include <iostream>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <sys/syscall.h>
#include <time.h>
#include <chrono>

namespace nevil_realtime {

bool RTThreadUtils::setThreadPriority(SchedPolicy policy, int priority) {
  struct sched_param param;
  param.sched_priority = priority;
  
  // Validate priority range
  if (priority < getMinPriority(policy) || priority > getMaxPriority(policy)) {
    std::cerr << "Priority " << priority << " out of range for policy " 
              << static_cast<int>(policy) << std::endl;
    return false;
  }
  
  int result = pthread_setschedparam(pthread_self(), static_cast<int>(policy), &param);
  if (result != 0) {
    std::cerr << "Failed to set thread priority: " << strerror(result) << std::endl;
    return false;
  }
  
  return true;
}

bool RTThreadUtils::setThreadAffinity(const std::vector<int>& cpu_ids) {
  if (cpu_ids.empty()) {
    return true;  // No specific affinity requested
  }
  
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  
  for (int cpu : cpu_ids) {
    CPU_SET(cpu, &cpuset);
  }
  
  int result = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
  if (result != 0) {
    std::cerr << "Failed to set thread affinity: " << strerror(result) << std::endl;
    return false;
  }
  
  return true;
}

bool RTThreadUtils::lockMemory() {
  int result = mlockall(MCL_CURRENT | MCL_FUTURE);
  if (result != 0) {
    std::cerr << "Failed to lock memory: " << strerror(errno) << std::endl;
    return false;
  }
  
  return true;
}

std::thread RTThreadUtils::createRTThread(
    std::function<void()> func,
    SchedPolicy policy,
    int priority,
    const std::vector<int>& cpu_ids) {
  
  return std::thread([func, policy, priority, cpu_ids]() {
    // Set thread name
    setThreadName("rt_thread");
    
    // Lock memory to prevent paging
    lockMemory();
    
    // Set thread priority
    setThreadPriority(policy, priority);
    
    // Set CPU affinity if specified
    if (!cpu_ids.empty()) {
      setThreadAffinity(cpu_ids);
    }
    
    // Execute the thread function
    func();
  });
}

SchedPolicy RTThreadUtils::getCurrentPolicy() {
  struct sched_param param;
  int policy;
  
  if (pthread_getschedparam(pthread_self(), &policy, &param) != 0) {
    return SchedPolicy::NORMAL;  // Default to normal if error
  }
  
  return static_cast<SchedPolicy>(policy);
}

int RTThreadUtils::getCurrentPriority() {
  struct sched_param param;
  int policy;
  
  if (pthread_getschedparam(pthread_self(), &policy, &param) != 0) {
    return 0;  // Default to 0 if error
  }
  
  return param.sched_priority;
}

int RTThreadUtils::getMaxPriority(SchedPolicy policy) {
  return sched_get_priority_max(static_cast<int>(policy));
}

int RTThreadUtils::getMinPriority(SchedPolicy policy) {
  return sched_get_priority_min(static_cast<int>(policy));
}

bool RTThreadUtils::isRealTimeKernel() {
  // Check for PREEMPT_RT by looking for "PREEMPT" or "PREEMPT_RT" in kernel version
  FILE* fp = popen("uname -a", "r");
  if (!fp) {
    return false;
  }
  
  char buffer[1024];
  std::string output;
  
  while (fgets(buffer, sizeof(buffer), fp) != nullptr) {
    output += buffer;
  }
  
  pclose(fp);
  
  return (output.find("PREEMPT") != std::string::npos);
}

std::string RTThreadUtils::getThreadName() {
  char name[16] = {0};  // 16 bytes is the maximum thread name length
  
  int result = pthread_getname_np(pthread_self(), name, sizeof(name));
  if (result != 0) {
    return "";
  }
  
  return std::string(name);
}

bool RTThreadUtils::setThreadName(const std::string& name) {
  // Truncate name if longer than 15 characters (16 bytes including null terminator)
  std::string truncated_name = name.substr(0, 15);
  
  int result = pthread_setname_np(pthread_self(), truncated_name.c_str());
  if (result != 0) {
    std::cerr << "Failed to set thread name: " << strerror(result) << std::endl;
    return false;
  }
  
  return true;
}

// RTMutex implementation
RTMutex::RTMutex() {
  pthread_mutexattr_init(&attr_);
  pthread_mutexattr_setprotocol(&attr_, PTHREAD_PRIO_INHERIT);
  pthread_mutex_init(&mutex_, &attr_);
}

RTMutex::~RTMutex() {
  pthread_mutex_destroy(&mutex_);
  pthread_mutexattr_destroy(&attr_);
}

void RTMutex::lock() {
  pthread_mutex_lock(&mutex_);
}

bool RTMutex::try_lock() {
  return (pthread_mutex_trylock(&mutex_) == 0);
}

void RTMutex::unlock() {
  pthread_mutex_unlock(&mutex_);
}

pthread_mutex_t* RTMutex::native_handle() {
  return &mutex_;
}

// RTConditionVariable implementation
RTConditionVariable::RTConditionVariable() {
  pthread_condattr_init(&attr_);
  pthread_condattr_setclock(&attr_, CLOCK_MONOTONIC);
  pthread_cond_init(&cond_, &attr_);
}

RTConditionVariable::~RTConditionVariable() {
  pthread_cond_destroy(&cond_);
  pthread_condattr_destroy(&attr_);
}

void RTConditionVariable::notify_one() {
  pthread_cond_signal(&cond_);
}

void RTConditionVariable::notify_all() {
  pthread_cond_broadcast(&cond_);
}

void RTConditionVariable::wait(std::unique_lock<RTMutex>& lock) {
  pthread_cond_wait(&cond_, lock.mutex()->native_handle());
}

template<class Rep, class Period>
bool RTConditionVariable::wait_for(std::unique_lock<RTMutex>& lock,
                                  const std::chrono::duration<Rep, Period>& rel_time) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(rel_time);
  auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(rel_time - seconds);
  
  ts.tv_sec += seconds.count();
  ts.tv_nsec += nanoseconds.count();
  
  // Handle nanosecond overflow
  if (ts.tv_nsec >= 1000000000) {
    ts.tv_sec += 1;
    ts.tv_nsec -= 1000000000;
  }
  
  int result = pthread_cond_timedwait(&cond_, lock.mutex()->native_handle(), &ts);
  return (result != ETIMEDOUT);
}

pthread_cond_t* RTConditionVariable::native_handle() {
  return &cond_;
}

} // namespace nevil_realtime