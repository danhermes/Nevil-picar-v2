#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.publisher_ = self.create_publisher(String, 'perception_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Perception node initialized')

    def timer_callback(self):
        msg = String()
        msg.data = 'Perception system active'
        self.publisher_.publish(msg)
        self.get_logger().debug('Published perception status')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
