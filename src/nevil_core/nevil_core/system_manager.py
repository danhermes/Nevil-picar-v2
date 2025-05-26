#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class SystemManager(Node):
    """
    System Manager Node for Nevil-picar v2.0
    
    This node is responsible for:
    - Managing system modes
    - Coordinating node lifecycles
    - Monitoring system health
    """
    
    def __init__(self):
        super().__init__('system_manager')
        
        # QoS profile for system critical messages
        system_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize parameters
        self.declare_parameter('system_mode', 'standby')
        
        # Create publishers
        self.mode_publisher = self.create_publisher(
            String, 
            '/system_mode', 
            qos_profile=system_qos
        )
        
        # Create timers
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('System Manager initialized')
        
    def timer_callback(self):
        """Publish current system mode periodically"""
        msg = String()
        mode = self.get_parameter('system_mode').get_parameter_value().string_value
        msg.data = mode
        self.mode_publisher.publish(msg)
        self.get_logger().debug(f'Publishing system mode: {mode}')


def main(args=None):
    rclpy.init(args=args)
    
    system_manager = SystemManager()
    
    try:
        rclpy.spin(system_manager)
    except KeyboardInterrupt:
        pass
    finally:
        system_manager.get_logger().info('Shutting down System Manager')
        system_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()