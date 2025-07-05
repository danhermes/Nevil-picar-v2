#!/usr/bin/env python3

import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Import the real-time configuration manager
from nevil_realtime.rt_config_manager import RTConfigManager
from nevil_realtime.rt_hardware_interface import RTHardwareInterface


class RTMotorControlNode(Node):
    """
    Real-time motor control node for Nevil-picar v2.0.
    
    This node demonstrates real-time control of motors with
    proper priority handling, mutex management, and bounded execution time.
    """
    
    def __init__(self):
        super().__init__('rt_motor_control_node')
        
        # Create a realtime callback group
        self.rt_callback_group = RealtimeCallbackGroup()
        
        # Declare parameters
        self.declare_parameter(
            'control_rate', 50.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Control loop rate in Hz'
            )
        )
        
        self.declare_parameter(
            'max_linear_speed', 0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Maximum linear speed in m/s'
            )
        )
        
        self.declare_parameter(
            'max_angular_speed', 1.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Maximum angular speed in rad/s'
            )
        )
        
        self.declare_parameter(
            'emergency_stop_distance', 0.2,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Emergency stop distance in meters'
            )
        )
        
        # Get parameters
        self.control_rate = self.get_parameter('control_rate').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.emergency_stop_distance = self.get_parameter('emergency_stop_distance').value
        
        # Create subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10,
            callback_group=self.rt_callback_group
        )
        
        self.ultrasonic_sub = self.create_subscription(
            Range,
            'ultrasonic_data',
            self.ultrasonic_callback,
            10,
            callback_group=self.rt_callback_group
        )
        
        # Create publishers
        self.emergency_stop_pub = self.create_publisher(
            Bool,
            'emergency_stop',
            10
        )
        
        # Create timers
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_callback,
            callback_group=self.rt_callback_group
        )
        
        # Initialize state variables
        self.current_cmd_vel = Twist()
        self.last_cmd_vel_time = self.get_clock().now()
        self.current_distance = float('inf')
        self.emergency_stop = False
        
        # Initialize hardware interface
        self.hardware_interface = RTHardwareInterface(self)
        
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
        
        # Create a watchdog timer to detect command timeout
        self.watchdog_timer = self.create_timer(
            0.5,  # 500ms
            self.watchdog_callback
        )
        
        self.get_logger().info('Real-time motor control node initialized')
        
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
            param = SchedParam(90)  # Priority 90 (highest)
            result = libc.sched_setscheduler(tid, SCHED_FIFO, ctypes.byref(param))
            
            if result == 0:
                self.get_logger().info('Set thread priority to 90 (SCHED_FIFO)')
            else:
                self.get_logger().warn(
                    'Failed to set thread priority. '
                    'Run with sudo or use launch script with chrt.'
                )
        except Exception as e:
            self.get_logger().warn(f'Failed to set thread priority: {e}')
    
    def cmd_vel_callback(self, msg):
        """
        Callback for velocity commands.
        
        This callback receives velocity commands and stores them for
        processing in the control loop.
        """
        # Store the command
        self.current_cmd_vel = msg
        self.last_cmd_vel_time = self.get_clock().now()
        
        # Reset emergency stop if we receive a zero command
        if abs(msg.linear.x) < 0.001 and abs(msg.angular.z) < 0.001:
            self.emergency_stop = False
    
    def ultrasonic_callback(self, msg):
        """
        Callback for ultrasonic sensor data.
        
        This callback receives distance measurements and checks for
        emergency stop conditions.
        """
        # Store the distance
        self.current_distance = msg.range
        
        # Check for emergency stop condition
        if (msg.range < self.emergency_stop_distance and 
            self.current_cmd_vel.linear.x > 0.0):
            
            self.emergency_stop = True
            
            # Publish emergency stop message
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_stop_pub.publish(emergency_msg)
            
            self.get_logger().warn(
                f'Emergency stop triggered: obstacle at {msg.range:.2f}m'
            )
    
    def control_callback(self):
        """
        Real-time control loop.
        
        This callback computes motor commands based on the current velocity
        command and safety constraints, with bounded execution time.
        """
        # Record callback start time for latency measurement
        start_time = time.monotonic()
        
        # Check if we're in emergency stop mode
        if self.emergency_stop:
            self.set_motor_speeds(0.0, 0.0)
            
            # Calculate and record callback latency
            end_time = time.monotonic()
            latency = (end_time - start_time) * 1000.0  # Convert to milliseconds
            self.update_latency_stats(latency)
            
            return
        
        # Get the current command
        cmd_vel = self.current_cmd_vel
        
        # Clamp to maximum speeds
        linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, cmd_vel.linear.x))
        angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, cmd_vel.angular.z))
        
        # Convert to differential drive commands
        # Simple kinematic model: v_l = v - ω*L/2, v_r = v + ω*L/2
        # where L is the wheel base (distance between wheels)
        wheel_base = 0.16  # 16cm, adjust based on actual robot dimensions
        
        left_speed = linear_x - angular_z * wheel_base / 2.0
        right_speed = linear_x + angular_z * wheel_base / 2.0
        
        # Normalize to motor speed range (-1.0 to 1.0)
        max_wheel_speed = self.max_linear_speed + self.max_angular_speed * wheel_base / 2.0
        
        left_normalized = left_speed / max_wheel_speed
        right_normalized = right_speed / max_wheel_speed
        
        # Set motor speeds
        self.set_motor_speeds(left_normalized, right_normalized)
        
        # Calculate and record callback latency
        end_time = time.monotonic()
        latency = (end_time - start_time) * 1000.0  # Convert to milliseconds
        self.update_latency_stats(latency)
    
    def set_motor_speeds(self, left, right):
        """
        Set motor speeds with proper mutex handling.
        
        This method would interface with the actual hardware in a real
        implementation, with proper mutex handling for hardware access.
        """
        # Clamp values to valid range
        left_clamped = max(-1.0, min(1.0, left))
        right_clamped = max(-1.0, min(1.0, right))
        
        # Convert to motor speed values (-100 to 100)
        left_motor = int(left_clamped * 100.0)
        right_motor = int(right_clamped * 100.0)
        
        # Use the RTHardwareInterface to control the motors
        self.hardware_interface.set_motor_speeds(left_clamped, right_clamped)
        
        # Debug output (uncomment for debugging)
        # self.get_logger().debug(
        #     f'Motor speeds: left={left_motor}, right={right_motor}'
        # )
    
    def watchdog_callback(self):
        """
        Watchdog timer callback.
        
        This callback checks if we've received a command recently,
        and stops the robot if not.
        """
        current_time = self.get_clock().now()
        time_since_last_cmd = current_time - self.last_cmd_vel_time
        
        # If we haven't received a command in 1 second, stop the robot
        if time_since_last_cmd.nanoseconds > 1e9:
            self.get_logger().warn('Command timeout, stopping robot')
            self.hardware_interface.stop()
    
    def update_latency_stats(self, latency):
        """Update latency statistics."""
        self.callback_latencies.append(latency)
        if len(self.callback_latencies) > 1000:
            self.callback_latencies.pop(0)
        
        # Update latency statistics
        self.max_latency = max(self.max_latency, latency)
        self.min_latency = min(self.min_latency, latency)
        
        self.latency_count += 1
        self.avg_latency = (self.avg_latency * (self.latency_count - 1) + latency) / self.latency_count
    
    def report_latency(self):
        """Report callback latency statistics."""
        if self.latency_count == 0:
            return
        
        self.get_logger().info(
            f'Control callback latency (ms): '
            f'min={self.min_latency:.3f}, '
            f'max={self.max_latency:.3f}, '
            f'avg={self.avg_latency:.3f}, '
            f'current={self.callback_latencies[-1] if self.callback_latencies else 0.0:.3f}'
        )
        
        # Check if we're meeting real-time requirements
        if self.max_latency > 5.0:  # 5ms threshold for motor control
            self.get_logger().warn(
                f'Maximum latency ({self.max_latency:.3f}ms) exceeds real-time threshold (5ms)'
            )


def main(args=None):
    """Main function."""
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create the node
    node = RTMotorControlNode()
    
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
        node.hardware_interface.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()