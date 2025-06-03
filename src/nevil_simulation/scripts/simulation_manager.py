#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimulationManager(Node):
    def __init__(self):
        super().__init__('simulation_manager')
        self.publisher_ = self.create_publisher(String, 'simulation_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Simulation Manager initialized')

    def timer_callback(self):
        msg = String()
        msg.data = 'Simulation running'
        self.publisher_.publish(msg)
        self.get_logger().debug('Published simulation status')

def main(args=None):
    rclpy.init(args=args)
    node = SimulationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
