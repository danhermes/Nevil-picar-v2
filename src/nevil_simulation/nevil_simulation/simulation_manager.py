#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, Range
from visualization_msgs.msg import MarkerArray

class SimulationManager(Node):
    """
    Simulation Manager for Nevil-picar v2.0
    
    This node manages the simulation environment and provides a high-level
    interface for controlling the simulation.
    """
    
    def __init__(self):
        super().__init__('simulation_manager')
        
        # Create callback groups
        self.timer_callback_group = ReentrantCallbackGroup()
        self.service_callback_group = ReentrantCallbackGroup()
        self.subscription_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('update_rate', 20.0),
                ('auto_start', True),
                ('default_environment', 'empty'),
                ('environments_path', 'share/nevil_simulation/environments'),
                ('robot.mass', 1.5),
                ('robot.width', 0.15),
                ('robot.length', 0.2),
                ('robot.wheel_radius', 0.03),
                ('robot.max_linear_velocity', 0.5),
                ('robot.max_angular_velocity', 1.0),
                ('visualization.enabled', True),
                ('visualization.show_robot', True),
                ('visualization.show_sensors', True),
                ('visualization.show_debug', False),
                ('physics.enabled', True),
                ('physics.update_rate', 100.0),
                ('physics.gravity', 9.81),
                ('physics.friction', 0.5),
            ]
        )
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.auto_start = self.get_parameter('auto_start').value
        self.default_environment = self.get_parameter('default_environment').value
        self.environments_path = self.get_parameter('environments_path').value
        
        # Initialize state
        self.running = False
        self.current_environment = None
        self.simulation_time = 0.0
        self.environments = {}
        
        # Create publishers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.status_pub = self.create_publisher(
            String, '/simulation/status', qos_profile)
        
        self.control_pub = self.create_publisher(
            String, '/simulation/control', qos_profile)
        
        self.environment_pub = self.create_publisher(
            String, '/simulation/environment', qos_profile)
        
        # Create subscribers
        self.control_sub = self.create_subscription(
            String,
            '/simulation/control_cmd',
            self.control_callback,
            qos_profile,
            callback_group=self.subscription_callback_group
        )
        
        self.environment_sub = self.create_subscription(
            String,
            '/simulation/environment_cmd',
            self.environment_callback,
            qos_profile,
            callback_group=self.subscription_callback_group
        )
        
        # Create timer
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.timer_callback,
            callback_group=self.timer_callback_group
        )
        
        # Load available environments
        self.load_available_environments()
        
        # Start simulation if auto_start is enabled
        if self.auto_start:
            self.start_simulation()
        
        self.get_logger().info('Simulation Manager initialized')
    
    def load_available_environments(self):
        """Load available environments from the environments directory"""
        try:
            # Get the package share directory
            package_share_dir = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                self.environments_path
            )
            
            # List all YAML files in the environments directory
            for filename in os.listdir(package_share_dir):
                if filename.endswith('.yaml'):
                    env_name = os.path.splitext(filename)[0]
                    env_path = os.path.join(package_share_dir, filename)
                    
                    # Load environment metadata
                    with open(env_path, 'r') as file:
                        env_data = yaml.safe_load(file)
                        
                        self.environments[env_name] = {
                            'path': env_path,
                            'name': env_data.get('name', env_name),
                            'description': env_data.get('description', ''),
                            'size': env_data.get('size', [10.0, 10.0]),
                            'obstacle_count': len(env_data.get('obstacles', []))
                        }
            
            self.get_logger().info(f'Loaded {len(self.environments)} environments')
        except Exception as e:
            self.get_logger().error(f'Failed to load environments: {str(e)}')
    
    def start_simulation(self):
        """Start the simulation"""
        if not self.running:
            self.running = True
            self.simulation_time = 0.0
            
            # Load default environment if none is loaded
            if self.current_environment is None:
                self.load_environment(self.default_environment)
            
            # Publish control command
            msg = String()
            msg.data = 'start'
            self.control_pub.publish(msg)
            
            self.get_logger().info('Simulation started')
    
    def stop_simulation(self):
        """Stop the simulation"""
        if self.running:
            self.running = False
            
            # Publish control command
            msg = String()
            msg.data = 'stop'
            self.control_pub.publish(msg)
            
            self.get_logger().info('Simulation stopped')
    
    def reset_simulation(self):
        """Reset the simulation"""
        # Publish control command
        msg = String()
        msg.data = 'reset'
        self.control_pub.publish(msg)
        
        self.simulation_time = 0.0
        
        self.get_logger().info('Simulation reset')
    
    def load_environment(self, environment_name):
        """Load an environment"""
        if environment_name in self.environments:
            self.current_environment = environment_name
            
            # Publish environment command
            msg = String()
            msg.data = environment_name
            self.environment_pub.publish(msg)
            
            self.get_logger().info(f'Loaded environment: {environment_name}')
            return True
        else:
            self.get_logger().error(f'Environment not found: {environment_name}')
            return False
    
    def control_callback(self, msg):
        """Handle control commands"""
        command = msg.data.lower()
        
        if command == 'start':
            self.start_simulation()
        elif command == 'stop':
            self.stop_simulation()
        elif command == 'reset':
            self.reset_simulation()
        elif command == 'pause':
            self.running = False
            self.get_logger().info('Simulation paused')
        elif command == 'resume':
            self.running = True
            self.get_logger().info('Simulation resumed')
        else:
            self.get_logger().warn(f'Unknown control command: {command}')
    
    def environment_callback(self, msg):
        """Handle environment commands"""
        environment_name = msg.data
        self.load_environment(environment_name)
    
    def timer_callback(self):
        """Update the simulation state"""
        if self.running:
            # Update simulation time
            self.simulation_time += 1.0 / self.update_rate
            
            # Publish status
            status_msg = String()
            status_msg.data = (
                f'running:{self.running},'
                f'time:{self.simulation_time:.2f},'
                f'environment:{self.current_environment}'
            )
            self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    simulation_manager = SimulationManager()
    
    # Use a MultiThreadedExecutor to enable parallel processing
    executor = MultiThreadedExecutor()
    executor.add_node(simulation_manager)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        simulation_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()