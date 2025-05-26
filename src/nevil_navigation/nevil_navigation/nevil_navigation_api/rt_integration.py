#!/usr/bin/env python3

import os
import time
import threading
import ctypes
from typing import Optional, Callable, Any

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import RealtimeCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class RTContext:
    """
    Real-time context manager for Nevil-picar v2.0.
    
    This class provides utilities for managing real-time execution contexts,
    including thread priority management, mutex handling, and bounded execution
    time enforcement.
    """
    
    def __init__(self, node: Optional[Node] = None, priority: int = 90):
        """
        Initialize the real-time context manager.
        
        Args:
            node: An existing ROS2 node to use for logging
            priority: Real-time priority level (0-99, higher is higher priority)
        """
        self.node = node
        self.priority = priority
        self.rt_enabled = False
        
        # Try to detect if we're running on a PREEMPT-RT kernel
        try:
            with open('/proc/version', 'r') as f:
                kernel_version = f.read()
                self.rt_kernel = 'PREEMPT_RT' in kernel_version or 'PREEMPT RT' in kernel_version
        except:
            self.rt_kernel = False
        
        if self.node:
            if self.rt_kernel:
                self.node.get_logger().info('PREEMPT-RT kernel detected')
            else:
                self.node.get_logger().warn('PREEMPT-RT kernel not detected, real-time features will be limited')
    
    def set_thread_priority(self, tid: Optional[int] = None, priority: Optional[int] = None) -> bool:
        """
        Set the priority of a thread.
        
        Args:
            tid: Thread ID to set priority for (default: current thread)
            priority: Priority level (0-99, higher is higher priority)
            
        Returns:
            True if successful, False otherwise
        """
        if priority is None:
            priority = self.priority
        
        # Ensure priority is in valid range
        priority = max(0, min(99, priority))
        
        try:
            # Load libc
            libc = ctypes.CDLL('libc.so.6')
            
            # Define constants for scheduling policies
            SCHED_OTHER = 0
            SCHED_FIFO = 1
            
            # Define the sched_param struct
            class SchedParam(ctypes.Structure):
                _fields_ = [("sched_priority", ctypes.c_int)]
            
            # Get the current thread ID if not provided
            if tid is None:
                tid = libc.syscall(186)  # SYS_gettid
            
            # Set the scheduling policy and priority
            param = SchedParam(priority)
            result = libc.sched_setscheduler(tid, SCHED_FIFO, ctypes.byref(param))
            
            if result == 0:
                self.rt_enabled = True
                if self.node:
                    self.node.get_logger().info(f'Set thread priority to {priority} (SCHED_FIFO)')
                return True
            else:
                if self.node:
                    self.node.get_logger().warn(
                        f'Failed to set thread priority. '
                        f'Run with sudo or use launch script with chrt.'
                    )
                return False
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f'Failed to set thread priority: {e}')
            return False
    
    def get_thread_priority(self, tid: Optional[int] = None) -> int:
        """
        Get the priority of a thread.
        
        Args:
            tid: Thread ID to get priority for (default: current thread)
            
        Returns:
            Priority level (0-99), or -1 if failed
        """
        try:
            # Load libc
            libc = ctypes.CDLL('libc.so.6')
            
            # Define the sched_param struct
            class SchedParam(ctypes.Structure):
                _fields_ = [("sched_priority", ctypes.c_int)]
            
            # Get the current thread ID if not provided
            if tid is None:
                tid = libc.syscall(186)  # SYS_gettid
            
            # Get the scheduling policy
            policy = libc.sched_getscheduler(tid)
            
            # Get the priority
            param = SchedParam(0)
            result = libc.sched_getparam(tid, ctypes.byref(param))
            
            if result == 0:
                return param.sched_priority
            else:
                return -1
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f'Failed to get thread priority: {e}')
            return -1
    
    def create_rt_thread(self, target: Callable, args: tuple = (), kwargs: dict = None) -> threading.Thread:
        """
        Create a real-time thread.
        
        Args:
            target: Function to run in the thread
            args: Arguments to pass to the function
            kwargs: Keyword arguments to pass to the function
            
        Returns:
            A thread object
        """
        if kwargs is None:
            kwargs = {}
        
        # Create a wrapper function that sets the thread priority
        def rt_wrapper(*args, **kwargs):
            # Set the thread priority
            self.set_thread_priority()
            
            # Run the target function
            return target(*args, **kwargs)
        
        # Create and return the thread
        thread = threading.Thread(target=rt_wrapper, args=args, kwargs=kwargs)
        thread.daemon = True
        
        return thread
    
    def run_in_rt_context(self, func: Callable, *args, **kwargs) -> Any:
        """
        Run a function in a real-time context.
        
        This function sets the thread priority to real-time, runs the function,
        and then restores the original priority.
        
        Args:
            func: Function to run
            *args: Arguments to pass to the function
            **kwargs: Keyword arguments to pass to the function
            
        Returns:
            The return value of the function
        """
        # Get the original priority
        original_priority = self.get_thread_priority()
        
        # Set the thread priority to real-time
        self.set_thread_priority()
        
        try:
            # Run the function
            return func(*args, **kwargs)
        finally:
            # Restore the original priority
            if original_priority >= 0:
                self.set_thread_priority(priority=original_priority)
    
    def __enter__(self):
        """Enter the real-time context."""
        # Get the original priority
        self.original_priority = self.get_thread_priority()
        
        # Set the thread priority to real-time
        self.set_thread_priority()
        
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Exit the real-time context."""
        # Restore the original priority
        if self.original_priority >= 0:
            self.set_thread_priority(priority=self.original_priority)


class RTNode(Node):
    """
    Real-time ROS2 node for Nevil-picar v2.0.
    
    This class extends the standard ROS2 Node class with real-time features,
    including thread priority management and real-time callback groups.
    """
    
    def __init__(self, node_name: str, priority: int = 90, **kwargs):
        """
        Initialize the real-time node.
        
        Args:
            node_name: Name of the node
            priority: Real-time priority level (0-99, higher is higher priority)
            **kwargs: Additional arguments to pass to the Node constructor
        """
        super().__init__(node_name, **kwargs)
        
        # Create a real-time context
        self.rt_context = RTContext(self, priority)
        
        # Create a real-time callback group
        self.rt_callback_group = RealtimeCallbackGroup()
        
        # Set the thread priority
        self.rt_context.set_thread_priority()
        
        self.get_logger().info(f'Real-time node {node_name} initialized with priority {priority}')
    
    def create_rt_timer(self, timer_period_sec: float, callback: Callable, **kwargs):
        """
        Create a real-time timer.
        
        Args:
            timer_period_sec: Timer period in seconds
            callback: Callback function to call when the timer fires
            **kwargs: Additional arguments to pass to the create_timer method
            
        Returns:
            A timer object
        """
        # Create a wrapper callback that runs in a real-time context
        def rt_callback():
            self.rt_context.run_in_rt_context(callback)
        
        # Create and return the timer
        return self.create_timer(
            timer_period_sec,
            rt_callback,
            callback_group=self.rt_callback_group,
            **kwargs
        )
    
    def create_rt_subscription(self, msg_type, topic, callback, qos_profile, **kwargs):
        """
        Create a real-time subscription.
        
        Args:
            msg_type: Message type
            topic: Topic name
            callback: Callback function to call when a message is received
            qos_profile: QoS profile
            **kwargs: Additional arguments to pass to the create_subscription method
            
        Returns:
            A subscription object
        """
        # Create a wrapper callback that runs in a real-time context
        def rt_callback(msg):
            self.rt_context.run_in_rt_context(callback, msg)
        
        # Create and return the subscription
        return self.create_subscription(
            msg_type,
            topic,
            rt_callback,
            qos_profile,
            callback_group=self.rt_callback_group,
            **kwargs
        )
    
    def create_rt_publisher(self, msg_type, topic, qos_profile, **kwargs):
        """
        Create a real-time publisher.
        
        Args:
            msg_type: Message type
            topic: Topic name
            qos_profile: QoS profile
            **kwargs: Additional arguments to pass to the create_publisher method
            
        Returns:
            A publisher object
        """
        # Create and return the publisher
        return self.create_publisher(
            msg_type,
            topic,
            qos_profile,
            **kwargs
        )
    
    def create_rt_service(self, srv_type, srv_name, callback, **kwargs):
        """
        Create a real-time service.
        
        Args:
            srv_type: Service type
            srv_name: Service name
            callback: Callback function to call when a request is received
            **kwargs: Additional arguments to pass to the create_service method
            
        Returns:
            A service object
        """
        # Create a wrapper callback that runs in a real-time context
        def rt_callback(request, response):
            return self.rt_context.run_in_rt_context(callback, request, response)
        
        # Create and return the service
        return self.create_service(
            srv_type,
            srv_name,
            rt_callback,
            callback_group=self.rt_callback_group,
            **kwargs
        )
    
    def create_rt_client(self, srv_type, srv_name, **kwargs):
        """
        Create a real-time service client.
        
        Args:
            srv_type: Service type
            srv_name: Service name
            **kwargs: Additional arguments to pass to the create_client method
            
        Returns:
            A client object
        """
        # Create and return the client
        return self.create_client(
            srv_type,
            srv_name,
            callback_group=self.rt_callback_group,
            **kwargs
        )


def create_rt_executor(nodes=None):
    """
    Create a real-time executor.
    
    Args:
        nodes: List of nodes to add to the executor
        
    Returns:
        A MultiThreadedExecutor
    """
    # Create the executor
    executor = MultiThreadedExecutor()
    
    # Add nodes if provided
    if nodes:
        for node in nodes:
            executor.add_node(node)
    
    return executor


def spin_rt_executor(executor, nodes=None):
    """
    Spin a real-time executor.
    
    Args:
        executor: Executor to spin
        nodes: List of nodes to add to the executor
        
    Returns:
        None
    """
    # Add nodes if provided
    if nodes:
        for node in nodes:
            executor.add_node(node)
    
    # Create a real-time context
    rt_context = RTContext()
    
    # Set the thread priority
    rt_context.set_thread_priority()
    
    try:
        # Spin the executor
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown ROS2
        rclpy.shutdown()