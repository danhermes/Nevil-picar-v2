#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AIInterfaceNode(Node):
    def __init__(self):
        super().__init__('ai_interface')
        self.publisher_ = self.create_publisher(String, 'ai_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('AI Interface Node initialized')

    def timer_callback(self):
        msg = String()
        msg.data = 'AI system active'
        self.publisher_.publish(msg)
        self.get_logger().debug('Published AI status')

def main(args=None):
    rclpy.init(args=args)
    node = AIInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
