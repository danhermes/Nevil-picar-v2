#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <atomic>
#include <cstring>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>

/**
 * @brief Priority Inheritance Demonstration
 * 
 * This program demonstrates the priority inheritance mechanism in PREEMPT-RT
 * by creating three threads with different priorities that share a mutex.
 * Without priority inheritance, this would lead to priority inversion.
 * 
 * Thread priorities:
 * - High-priority thread (90): Tries to acquire the mutex but is blocked
 * - Medium-priority thread (70): Runs CPU-intensive work
 * - Low-priority thread (50): Holds the mutex for a while
 * 
 * Without priority inheritance, the medium-priority thread would prevent
 * the low-priority thread from releasing the mutex, causing the high-priority
 * thread to wait longer than necessary (priority inversion).
 * 
 * With priority inheritance, the low-priority thread temporarily inherits
 * the priority of the high-priority thread while holding the mutex,
 * allowing it to preempt the medium-priority thread and release the mutex
 * more quickly.
 */

// Global variables
pthread_mutex_t mutex;
pthread_mutexattr_t mutex_attr;
std::atomic<bool> running(true);
std::atomic<int> low_priority_count(0);
std::atomic<int> medium_priority_count(0);
std::atomic<int> high_priority_count(0);
std::atomic<int> mutex_held_time_ms(0);
std::atomic<int> high_priority_wait_time_ms(0);

// Function to set thread priority and scheduling policy
bool setThreadPriority(int priority, int policy = SCHED_FIFO) {
    struct sched_param param;
    param.sched_priority = priority;
    
    int result = pthread_setschedparam(pthread_self(), policy, &param);
    if (result != 0) {
        std::cerr << "Failed to set thread priority: " << strerror(result) << std::endl;
        return false;
    }
    
    return true;
}

// Function to get thread priority
int getThreadPriority() {
    struct sched_param param;
    int policy;
    
    if (pthread_getschedparam(pthread_self(), &policy, &param) != 0) {
        return -1;
    }
    
    return param.sched_priority;
}

// Function to get thread scheduling policy
std::string getThreadPolicyString() {
    struct sched_param param;
    int policy;
    
    if (pthread_getschedparam(pthread_self(), &policy, &param) != 0) {
        return "UNKNOWN";
    }
    
    switch (policy) {
        case SCHED_FIFO:
            return "SCHED_FIFO";
        case SCHED_RR:
            return "SCHED_RR";
        case SCHED_OTHER:
            return "SCHED_OTHER";
        default:
            return "UNKNOWN";
    }
}

// Function to set thread name
bool setThreadName(const std::string& name) {
    // Truncate name if longer than 15 characters (16 bytes including null terminator)
    std::string truncated_name = name.substr(0, 15);
    
    int result = pthread_setname_np(pthread_self(), truncated_name.c_str());
    if (result != 0) {
        std::cerr << "Failed to set thread name: " << strerror(result) << std::endl;
        return false;
    }
    
    return true;
}

// Low-priority thread function
void lowPriorityThread() {
    setThreadName("low_priority");
    setThreadPriority(50);
    
    std::cout << "Low-priority thread started with priority " 
              << getThreadPriority() << " (" << getThreadPolicyString() << ")" 
              << std::endl;
    
    while (running) {
        // Acquire the mutex
        auto start_time = std::chrono::steady_clock::now();
        
        pthread_mutex_lock(&mutex);
        
        // Hold the mutex for a while
        std::cout << "Low-priority thread acquired mutex with priority " 
                  << getThreadPriority() << std::endl;
        
        // Simulate some work
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Release the mutex
        pthread_mutex_unlock(&mutex);
        
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time).count();
        
        mutex_held_time_ms.store(duration);
        
        std::cout << "Low-priority thread released mutex with priority " 
                  << getThreadPriority() << std::endl;
        
        // Increment counter
        low_priority_count++;
        
        // Sleep for a while
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

// Medium-priority thread function
void mediumPriorityThread() {
    setThreadName("medium_priority");
    setThreadPriority(70);
    
    std::cout << "Medium-priority thread started with priority " 
              << getThreadPriority() << " (" << getThreadPolicyString() << ")" 
              << std::endl;
    
    while (running) {
        // CPU-intensive work
        for (int i = 0; i < 10000000; i++) {
            // Just burn CPU cycles
            asm volatile("" : : : "memory");
        }
        
        // Increment counter
        medium_priority_count++;
        
        // Yield to allow other threads to run
        std::this_thread::yield();
    }
}

// High-priority thread function
void highPriorityThread() {
    setThreadName("high_priority");
    setThreadPriority(90);
    
    std::cout << "High-priority thread started with priority " 
              << getThreadPriority() << " (" << getThreadPolicyString() << ")" 
              << std::endl;
    
    while (running) {
        // Wait for a while
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        // Try to acquire the mutex
        std::cout << "High-priority thread trying to acquire mutex" << std::endl;
        
        auto start_time = std::chrono::steady_clock::now();
        
        pthread_mutex_lock(&mutex);
        
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time).count();
        
        high_priority_wait_time_ms.store(duration);
        
        std::cout << "High-priority thread acquired mutex after " 
                  << duration << "ms" << std::endl;
        
        // Simulate some work
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // Release the mutex
        pthread_mutex_unlock(&mutex);
        
        std::cout << "High-priority thread released mutex" << std::endl;
        
        // Increment counter
        high_priority_count++;
    }
}

// Monitor thread function
void monitorThread() {
    setThreadName("monitor");
    
    int prev_low_count = 0;
    int prev_medium_count = 0;
    int prev_high_count = 0;
    
    while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        int low_count = low_priority_count.load();
        int medium_count = medium_priority_count.load();
        int high_count = high_priority_count.load();
        
        int low_rate = low_count - prev_low_count;
        int medium_rate = medium_count - prev_medium_count;
        int high_rate = high_count - prev_high_count;
        
        prev_low_count = low_count;
        prev_medium_count = medium_count;
        prev_high_count = high_count;
        
        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << "Thread execution rates (iterations/second):" << std::endl;
        std::cout << "  Low-priority: " << low_rate << std::endl;
        std::cout << "  Medium-priority: " << medium_rate << std::endl;
        std::cout << "  High-priority: " << high_rate << std::endl;
        std::cout << "Mutex held time: " << mutex_held_time_ms.load() << "ms" << std::endl;
        std::cout << "High-priority wait time: " << high_priority_wait_time_ms.load() << "ms" << std::endl;
        std::cout << "---------------------------------------------------" << std::endl;
    }
}

int main(int argc, char** argv) {
    bool use_priority_inheritance = true;
    
    // Parse command-line arguments
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--no-pi") {
            use_priority_inheritance = false;
        }
    }
    
    // Check if we're running on a PREEMPT-RT kernel
    bool is_rt_kernel = false;
    FILE* fp = popen("uname -a", "r");
    if (fp) {
        char buffer[1024];
        std::string output;
        
        while (fgets(buffer, sizeof(buffer), fp) != nullptr) {
            output += buffer;
        }
        
        pclose(fp);
        
        is_rt_kernel = (output.find("PREEMPT") != std::string::npos);
    }
    
    std::cout << "Running on " << (is_rt_kernel ? "PREEMPT-RT" : "standard") << " kernel" << std::endl;
    
    // Lock memory to prevent paging
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "Warning: Failed to lock memory: " << strerror(errno) << std::endl;
    } else {
        std::cout << "Memory locked successfully" << std::endl;
    }
    
    // Initialize mutex with or without priority inheritance
    pthread_mutexattr_init(&mutex_attr);
    
    if (use_priority_inheritance) {
        pthread_mutexattr_setprotocol(&mutex_attr, PTHREAD_PRIO_INHERIT);
        std::cout << "Using mutex with priority inheritance" << std::endl;
    } else {
        pthread_mutexattr_setprotocol(&mutex_attr, PTHREAD_PRIO_NONE);
        std::cout << "Using mutex without priority inheritance" << std::endl;
    }
    
    pthread_mutex_init(&mutex, &mutex_attr);
    
    // Create threads
    std::thread low_thread(lowPriorityThread);
    std::thread medium_thread(mediumPriorityThread);
    std::thread high_thread(highPriorityThread);
    std::thread monitor(monitorThread);
    
    // Wait for user input to stop
    std::cout << "Press Enter to stop..." << std::endl;
    std::cin.get();
    
    // Stop threads
    running.store(false);
    
    // Join threads
    low_thread.join();
    medium_thread.join();
    high_thread.join();
    monitor.join();
    
    // Clean up
    pthread_mutex_destroy(&mutex);
    pthread_mutexattr_destroy(&mutex_attr);
    
    return 0;
}