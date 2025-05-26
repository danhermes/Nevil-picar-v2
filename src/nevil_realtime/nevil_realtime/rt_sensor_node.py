#!/usr/bin/env python3

import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import RealtimeCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray, Header
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import the real-time configuration manager
from nevil_realtime.rt_config_manager import RTConfigManager


class RTSensorNode(Node):
    """
    Real-time sensor processing node for Nevil-picar v2.0.
    
    This node demonstrates real-time processing of sensor data with
    proper priority handling and bounded execution time.
    """
    
    def __init__(self):
        super().__init__('rt_sensor_node')
        
        # Create a realtime callback group
        self.rt_callback_group = RealtimeCallbackGroup()
        
        # Declare parameters
        self.declare_parameter(
            'update_rate', 50.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Sensor update rate in Hz'
            )
        )
        
        self.declare_parameter(
            'filter_window_size', 5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Window size for moving average filter'
            )
        )
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.filter_window_size = self.get_parameter('filter_window_size').value
        
        # Create publishers
        self.ultrasonic_pub = self.create_publisher(
            Range,
            'ultrasonic_data',
            10
        )
        
        self.filtered_data_pub = self.create_publisher(
            Float32MultiArray,
            'filtered_sensor_data',
            10
        )
        
        # Create timers
        self.sensor_timer = self.create_timer(
            1.0 / self.update_rate,
            self.sensor_callback,
            callback_group=self.rt_callback_group
        )
        
        # Initialize sensor data buffers
        self.ultrasonic_buffer = []
        
        # Initialize latency tracking
        self.callback_latencies = []
        self.max_latency = 0.0
        self.min_latency = float('inf')
        self.avg_latency = 0.0
        self.latency_count = 0
        
        # Create a timer for latency reporting
        self.create_timer(
            10.0,
            self.report_latency
        )
        
        self.get_logger().info('Real-time sensor node initialized')
        
        # Try to set thread priority using Python's os module
        try:
            import ctypes
            libc = ctypes.CDLL('libc.so.6')
            
            # Define constants for scheduling policies
            SCHED_OTHER = 0
            SCHED_FIFO = 1
            
            # Define the sched_param struct
            class SchedParam(ctypes.Structure):
                _fields_ = [("sched_priority", ctypes.c_int)]
            
            # Get the current thread ID
            tid = ctypes.CDLL('libc.so.6').syscall(186)
            
            # Set the scheduling policy and priority
            param = SchedParam(80)  # Priority 80
            result = libc.sched_setscheduler(tid, SCHED_FIFO, ctypes.byref(param))
            
            if result == 0:
                self.get_logger().info('Set thread priority to 80 (SCHED_FIFO)')
            else:
                self.get_logger().warn(
                    'Failed to set thread priority. '
                    'Run with sudo or use launch script with chrt.'
                )
        except Exception as e:
            self.get_logger().warn(f'Failed to set thread priority: {e}')
    
    def sensor_callback(self):
        """
        Real-time sensor processing callback.
        
        This callback reads sensor data, applies filtering, and publishes
        the results with bounded execution time.
        """
        # Record callback start time for latency measurement
        start_time = time.monotonic()
        
        # Simulate reading from ultrasonic sensor
        # In a real implementation, this would read from actual hardware
        distance = self.simulate_ultrasonic_reading()
        
        # Apply filtering
        self.ultrasonic_buffer.append(distance)
        if len(self.ultrasonic_buffer) > self.filter_window_size:
            self.ultrasonic_buffer.pop(0)
        
        filtered_distance = self.apply_filter(self.ultrasonic_buffer)
        
        # Create and publish Range message
        range_msg = Range()
        range_msg.header = Header()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = 'ultrasonic_sensor'
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.1  # ~5 degrees
        range_msg.min_range = 0.02  # 2cm
        range_msg.max_range = 4.0   # 400cm
        range_msg.range = filtered_distance
        
        self.ultrasonic_pub.publish(range_msg)
        
        # Create and publish filtered data message
        filtered_data_msg = Float32MultiArray()
        filtered_data_msg.data = [filtered_distance]
        
        self.filtered_data_pub.publish(filtered_data_msg)
        
        # Calculate and record callback latency
        end_time = time.monotonic()
        latency = (end_time - start_time) * 1000.0  # Convert to milliseconds
        
        self.callback_latencies.append(latency)
        if len(self.callback_latencies) > 1000:
            self.callback_latencies.pop(0)
        
        # Update latency statistics
        self.max_latency = max(self.max_latency, latency)
        self.min_latency = min(self.min_latency, latency)
        
        self.latency_count += 1
        self.avg_latency = (self.avg_latency * (self.latency_count - 1) + latency) / self.latency_count
    
    def simulate_ultrasonic_reading(self):
        """Simulate reading from ultrasonic sensor."""
        # In a real implementation, this would read from actual hardware
        # Here we just generate a random value with some noise
        base_distance = 0.5  # 50cm
        noise = np.random.normal(0, 0.02)  # Gaussian noise with 2cm std dev
        return max(0.0, base_distance + noise)
    
    def apply_filter(self, data):
        """Apply filtering to sensor data."""
        # Simple moving average filter
        if not data:
            return 0.0
        
        # Remove outliers (values more than 3 std devs from mean)
        if len(data) >= 3:
            mean = np.mean(data)
            std = np.std(data)
            filtered_data = [x for x in data if abs(x - mean) <= 3 * std]
            if not filtered_data:
                filtered_data = data  # If all data was filtered out, use original data
        else:
            filtered_data = data
        
        return np.mean(filtered_data)
    
    def report_latency(self):
        """Report callback latency statistics."""
        if self.latency_count == 0:
            return
        
        self.get_logger().info(
            f'Sensor callback latency (ms): '
            f'min={self.min_latency:.3f}, '
            f'max={self.max_latency:.3f}, '
            f'avg={self.avg_latency:.3f}, '
            f'current={self.callback_latencies[-1] if self.callback_latencies else 0.0:.3f}'
        )
        
        # Check if we're meeting real-time requirements
        if self.max_latency > 10.0:  # 10ms threshold
            self.get_logger().warn(
                f'Maximum latency ({self.max_latency:.3f}ms) exceeds real-time threshold (10ms)'
            )


def main(args=None):
    """Main function."""
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create the node
    node = RTSensorNode()
    
    # Use a MultiThreadedExecutor for better performance
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        # Spin the node
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()