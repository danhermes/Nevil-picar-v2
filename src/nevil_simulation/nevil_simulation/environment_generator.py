#!/usr/bin/env python3

import os
import yaml
import random
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion, Pose

class EnvironmentGenerator(Node):
    """
    Environment Generator for Nevil-picar v2.0
    
    This node generates and manages virtual environments for the Nevil-picar simulation.
    """
    
    def __init__(self):
        super().__init__('environment_generator')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('environments_path', 'share/nevil_simulation/environments'),
                ('default_environment', 'empty'),
                ('default_size_x', 10.0),
                ('default_size_y', 10.0),
                ('default_obstacle_count', 5),
                ('wall_height', 0.5),
                ('ground_color_r', 0.8),
                ('ground_color_g', 0.8),
                ('ground_color_b', 0.8),
                ('ground_color_a', 1.0),
                ('wall_color_r', 0.7),
                ('wall_color_g', 0.7),
                ('wall_color_b', 0.7),
                ('wall_color_a', 1.0),
                ('obstacle_color_r', 0.9),
                ('obstacle_color_g', 0.2),
                ('obstacle_color_b', 0.2),
                ('obstacle_color_a', 1.0),
            ]
        )
        
        # Get parameters
        self.environments_path = self.get_parameter('environments_path').value
        self.default_environment = self.get_parameter('default_environment').value
        self.default_size_x = self.get_parameter('default_size_x').value
        self.default_size_y = self.get_parameter('default_size_y').value
        self.default_obstacle_count = self.get_parameter('default_obstacle_count').value
        self.wall_height = self.get_parameter('wall_height').value
        
        # Get color parameters
        self.ground_color = [
            self.get_parameter('ground_color_r').value,
            self.get_parameter('ground_color_g').value,
            self.get_parameter('ground_color_b').value,
            self.get_parameter('ground_color_a').value
        ]
        
        self.wall_color = [
            self.get_parameter('wall_color_r').value,
            self.get_parameter('wall_color_g').value,
            self.get_parameter('wall_color_b').value,
            self.get_parameter('wall_color_a').value
        ]
        
        self.obstacle_color = [
            self.get_parameter('obstacle_color_r').value,
            self.get_parameter('obstacle_color_g').value,
            self.get_parameter('obstacle_color_b').value,
            self.get_parameter('obstacle_color_a').value
        ]
        
        # Initialize state
        self.current_environment = None
        self.environment_data = None
        self.environments = {}
        self.next_marker_id = 0
        
        # Create publishers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.markers_pub = self.create_publisher(
            MarkerArray, '/simulation/environment_markers', qos_profile)
        
        self.status_pub = self.create_publisher(
            String, '/simulation/environment_status', qos_profile)
        
        # Create subscribers
        self.command_sub = self.create_subscription(
            String,
            '/simulation/environment_cmd',
            self.command_callback,
            qos_profile
        )
        
        # Load available environments
        self.load_available_environments()
        
        # Load default environment
        self.load_environment(self.default_environment)
        
        self.get_logger().info('Environment Generator initialized')
    
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
    
    def load_environment(self, environment_name):
        """Load an environment"""
        if environment_name in self.environments:
            try:
                env_path = self.environments[environment_name]['path']
                
                # Load environment data
                with open(env_path, 'r') as file:
                    self.environment_data = yaml.safe_load(file)
                
                self.current_environment = environment_name
                
                # Create visualization markers
                self.create_visualization_markers()
                
                # Publish status
                status_msg = String()
                status_msg.data = f'loaded:{environment_name}'
                self.status_pub.publish(status_msg)
                
                self.get_logger().info(f'Loaded environment: {environment_name}')
                return True
            except Exception as e:
                self.get_logger().error(f'Failed to load environment {environment_name}: {str(e)}')
                return False
        elif environment_name == 'random':
            # Generate a random environment
            return self.generate_random_environment(
                self.default_size_x,
                self.default_size_y,
                self.default_obstacle_count
            )
        elif environment_name == 'empty':
            # Generate an empty environment
            return self.generate_empty_environment(
                self.default_size_x,
                self.default_size_y
            )
        else:
            self.get_logger().error(f'Environment not found: {environment_name}')
            return False
    
    def generate_empty_environment(self, size_x, size_y):
        """Generate an empty environment"""
        self.environment_data = {
            'name': 'Empty Environment',
            'description': 'An empty environment with no obstacles',
            'size': [size_x, size_y],
            'obstacles': []
        }
        
        self.current_environment = 'empty'
        
        # Create visualization markers
        self.create_visualization_markers()
        
        # Publish status
        status_msg = String()
        status_msg.data = 'loaded:empty'
        self.status_pub.publish(status_msg)
        
        self.get_logger().info('Generated empty environment')
        return True
    
    def generate_random_environment(self, size_x, size_y, num_obstacles):
        """Generate a random environment"""
        obstacles = []
        
        # Generate random obstacles
        for i in range(num_obstacles):
            # Randomly choose obstacle type
            obstacle_type = random.choice(['box', 'cylinder', 'sphere'])
            
            # Generate random position (keep away from center where robot starts)
            while True:
                x = random.uniform(-size_x/2 + 1.0, size_x/2 - 1.0)
                y = random.uniform(-size_y/2 + 1.0, size_y/2 - 1.0)
                
                # Ensure obstacle is not too close to the center
                if math.sqrt(x**2 + y**2) > 1.5:
                    break
            
            # Generate random properties based on type
            if obstacle_type == 'box':
                width = random.uniform(0.3, 1.0)
                length = random.uniform(0.3, 1.0)
                height = random.uniform(0.3, 1.0)
                
                obstacles.append({
                    'type': obstacle_type,
                    'position': [x, y, 0.0],
                    'dimensions': [width, length, height]
                })
            elif obstacle_type == 'cylinder':
                radius = random.uniform(0.2, 0.5)
                height = random.uniform(0.3, 1.0)
                
                obstacles.append({
                    'type': obstacle_type,
                    'position': [x, y, 0.0],
                    'radius': radius,
                    'height': height
                })
            elif obstacle_type == 'sphere':
                radius = random.uniform(0.2, 0.5)
                
                obstacles.append({
                    'type': obstacle_type,
                    'position': [x, y, 0.0],
                    'radius': radius
                })
        
        # Create environment data
        self.environment_data = {
            'name': 'Random Environment',
            'description': f'Randomly generated environment with {num_obstacles} obstacles',
            'size': [size_x, size_y],
            'obstacles': obstacles
        }
        
        self.current_environment = 'random'
        
        # Create visualization markers
        self.create_visualization_markers()
        
        # Publish status
        status_msg = String()
        status_msg.data = 'loaded:random'
        self.status_pub.publish(status_msg)
        
        self.get_logger().info(f'Generated random environment with {num_obstacles} obstacles')
        return True
    
    def generate_obstacle_course(self, difficulty=3):
        """Generate an obstacle course environment"""
        size_x = 15.0
        size_y = 5.0
        obstacles = []
        
        # Adjust parameters based on difficulty
        num_obstacles = 5 + difficulty * 3
        min_gap = 1.0 - difficulty * 0.1
        if min_gap < 0.3:
            min_gap = 0.3
        
        # Create a zigzag path with obstacles
        for i in range(num_obstacles):
            x = (i / num_obstacles) * size_x - size_x/2 + size_x/(2*num_obstacles)
            
            # Alternate obstacles on top and bottom
            if i % 2 == 0:
                y = random.uniform(min_gap, size_y/2 - 0.5)
                length = random.uniform(0.5, size_y/2 - y - min_gap)
            else:
                y = random.uniform(-size_y/2 + 0.5, -min_gap)
                length = random.uniform(0.5, abs(y) - min_gap)
                y = -y  # Adjust for bottom placement
            
            # Add wall obstacle
            obstacles.append({
                'type': 'box',
                'position': [x, y, 0.0],
                'dimensions': [0.3, length, self.wall_height]
            })
        
        # Create environment data
        self.environment_data = {
            'name': f'Obstacle Course (Level {difficulty})',
            'description': f'Obstacle course with difficulty level {difficulty}',
            'size': [size_x, size_y],
            'obstacles': obstacles
        }
        
        self.current_environment = f'obstacle_course_{difficulty}'
        
        # Create visualization markers
        self.create_visualization_markers()
        
        # Publish status
        status_msg = String()
        status_msg.data = f'loaded:obstacle_course_{difficulty}'
        self.status_pub.publish(status_msg)
        
        self.get_logger().info(f'Generated obstacle course with difficulty {difficulty}')
        return True
    
    def generate_maze(self, size=10, complexity=0.75, density=0.75):
        """Generate a maze environment"""
        # Ensure odd dimensions
        shape = (size, size)
        
        # Adjust complexity and density relative to maze size
        complexity = int(complexity * (5 * (shape[0] + shape[1])))
        density = int(density * ((shape[0] // 2) * (shape[1] // 2)))
        
        # Create empty grid
        Z = np.zeros(shape, dtype=bool)
        
        # Fill borders
        Z[0, :] = Z[-1, :] = 1
        Z[:, 0] = Z[:, -1] = 1
        
        # Make random islands
        for i in range(density):
            x, y = random.randint(0, shape[0] // 2) * 2, random.randint(0, shape[1] // 2) * 2
            Z[x, y] = 1
            
            for j in range(complexity):
                neighbors = []
                if x > 1:
                    neighbors.append((x - 2, y))
                if x < shape[0] - 2:
                    neighbors.append((x + 2, y))
                if y > 1:
                    neighbors.append((x, y - 2))
                if y < shape[1] - 2:
                    neighbors.append((x, y + 2))
                
                if len(neighbors):
                    next_x, next_y = neighbors[random.randint(0, len(neighbors) - 1)]
                    if Z[next_x, next_y] == 0:
                        Z[next_x, next_y] = 1
                        Z[next_x + (x - next_x) // 2, next_y + (y - next_y) // 2] = 1
                        x, y = next_x, next_y
        
        # Convert maze to obstacles
        obstacles = []
        cell_size = 0.5
        maze_size_x = size * cell_size
        maze_size_y = size * cell_size
        
        for i in range(shape[0]):
            for j in range(shape[1]):
                if Z[i, j]:
                    x = i * cell_size - maze_size_x / 2 + cell_size / 2
                    y = j * cell_size - maze_size_y / 2 + cell_size / 2
                    
                    obstacles.append({
                        'type': 'box',
                        'position': [x, y, 0.0],
                        'dimensions': [cell_size, cell_size, self.wall_height]
                    })
        
        # Create environment data
        self.environment_data = {
            'name': f'Maze (Size {size})',
            'description': f'Randomly generated maze with size {size}x{size}',
            'size': [maze_size_x, maze_size_y],
            'obstacles': obstacles
        }
        
        self.current_environment = f'maze_{size}'
        
        # Create visualization markers
        self.create_visualization_markers()
        
        # Publish status
        status_msg = String()
        status_msg.data = f'loaded:maze_{size}'
        self.status_pub.publish(status_msg)
        
        self.get_logger().info(f'Generated maze with size {size}x{size}')
        return True
    
    def save_environment(self, filename):
        """Save the current environment to a YAML file"""
        if self.environment_data is None:
            self.get_logger().error('No environment to save')
            return False
        
        try:
            # Get the package share directory
            package_share_dir = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                self.environments_path
            )
            
            # Create the environments directory if it doesn't exist
            os.makedirs(package_share_dir, exist_ok=True)
            
            # Save environment data
            env_path = os.path.join(package_share_dir, f'{filename}.yaml')
            with open(env_path, 'w') as file:
                yaml.dump(self.environment_data, file, default_flow_style=False)
            
            self.get_logger().info(f'Saved environment to {env_path}')
            
            # Reload available environments
            self.load_available_environments()
            
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save environment: {str(e)}')
            return False
    
    def create_visualization_markers(self):
        """Create visualization markers for the environment"""
        if self.environment_data is None:
            return
        
        markers_array = MarkerArray()
        self.next_marker_id = 0
        
        # Create ground plane marker
        ground_marker = self.create_ground_marker()
        markers_array.markers.append(ground_marker)
        
        # Create wall markers
        wall_markers = self.create_wall_markers()
        for marker in wall_markers:
            markers_array.markers.append(marker)
        
        # Create obstacle markers
        if 'obstacles' in self.environment_data:
            for obstacle in self.environment_data['obstacles']:
                obstacle_marker = self.create_obstacle_marker(obstacle)
                if obstacle_marker:
                    markers_array.markers.append(obstacle_marker)
        
        # Publish markers
        self.markers_pub.publish(markers_array)
    
    def create_ground_marker(self):
        """Create a marker for the ground plane"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'environment'
        marker.id = self.next_marker_id
        self.next_marker_id += 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set position and size
        size_x = self.environment_data['size'][0]
        size_y = self.environment_data['size'][1]
        
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = -0.01  # Slightly below z=0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = size_x
        marker.scale.y = size_y
        marker.scale.z = 0.01
        
        # Set color
        marker.color.r = self.ground_color[0]
        marker.color.g = self.ground_color[1]
        marker.color.b = self.ground_color[2]
        marker.color.a = self.ground_color[3]
        
        return marker
    
    def create_wall_markers(self):
        """Create markers for the environment walls"""
        markers = []
        
        size_x = self.environment_data['size'][0]
        size_y = self.environment_data['size'][1]
        
        # Create four walls
        wall_thickness = 0.1
        
        # North wall
        north_wall = Marker()
        north_wall.header.frame_id = 'map'
        north_wall.header.stamp = self.get_clock().now().to_msg()
        north_wall.ns = 'environment'
        north_wall.id = self.next_marker_id
        self.next_marker_id += 1
        north_wall.type = Marker.CUBE
        north_wall.action = Marker.ADD
        
        north_wall.pose.position.x = 0.0
        north_wall.pose.position.y = size_y / 2 + wall_thickness / 2
        north_wall.pose.position.z = self.wall_height / 2
        north_wall.pose.orientation.x = 0.0
        north_wall.pose.orientation.y = 0.0
        north_wall.pose.orientation.z = 0.0
        north_wall.pose.orientation.w = 1.0
        
        north_wall.scale.x = size_x + 2 * wall_thickness
        north_wall.scale.y = wall_thickness
        north_wall.scale.z = self.wall_height
        
        north_wall.color.r = self.wall_color[0]
        north_wall.color.g = self.wall_color[1]
        north_wall.color.b = self.wall_color[2]
        north_wall.color.a = self.wall_color[3]
        
        markers.append(north_wall)
        
        # South wall
        south_wall = Marker()
        south_wall.header.frame_id = 'map'
        south_wall.header.stamp = self.get_clock().now().to_msg()
        south_wall.ns = 'environment'
        south_wall.id = self.next_marker_id
        self.next_marker_id += 1
        south_wall.type = Marker.CUBE
        south_wall.action = Marker.ADD
        
        south_wall.pose.position.x = 0.0
        south_wall.pose.position.y = -size_y / 2 - wall_thickness / 2
        south_wall.pose.position.z = self.wall_height / 2
        south_wall.pose.orientation.x = 0.0
        south_wall.pose.orientation.y = 0.0
        south_wall.pose.orientation.z = 0.0
        south_wall.pose.orientation.w = 1.0
        
        south_wall.scale.x = size_x + 2 * wall_thickness
        south_wall.scale.y = wall_thickness
        south_wall.scale.z = self.wall_height
        
        south_wall.color.r = self.wall_color[0]
        south_wall.color.g = self.wall_color[1]
        south_wall.color.b = self.wall_color[2]
        south_wall.color.a = self.wall_color[3]
        
        markers.append(south_wall)
        
        # East wall
        east_wall = Marker()
        east_wall.header.frame_id = 'map'
        east_wall.header.stamp = self.get_clock().now().to_msg()
        east_wall.ns = 'environment'
        east_wall.id = self.next_marker_id
        self.next_marker_id += 1
        east_wall.type = Marker.CUBE
        east_wall.action = Marker.ADD
        
        east_wall.pose.position.x = size_x / 2 + wall_thickness / 2
        east_wall.pose.position.y = 0.0
        east_wall.pose.position.z = self.wall_height / 2
        east_wall.pose.orientation.x = 0.0
        east_wall.pose.orientation.y = 0.0
        east_wall.pose.orientation.z = 0.0
        east_wall.pose.orientation.w = 1.0
        
        east_wall.scale.x = wall_thickness
        east_wall.scale.y = size_y
        east_wall.scale.z = self.wall_height
        
        east_wall.color.r = self.wall_color[0]
        east_wall.color.g = self.wall_color[1]
        east_wall.color.b = self.wall_color[2]
        east_wall.color.a = self.wall_color[3]
        
        markers.append(east_wall)
        
        # West wall
        west_wall = Marker()
        west_wall.header.frame_id = 'map'
        west_wall.header.stamp = self.get_clock().now().to_msg()
        west_wall.ns = 'environment'
        west_wall.id = self.next_marker_id
        self.next_marker_id += 1
        west_wall.type = Marker.CUBE
        west_wall.action = Marker.ADD
        
        west_wall.pose.position.x = -size_x / 2 - wall_thickness / 2
        west_wall.pose.position.y = 0.0
        west_wall.pose.position.z = self.wall_height / 2
        west_wall.pose.orientation.x = 0.0
        west_wall.pose.orientation.y = 0.0
        west_wall.pose.orientation.z = 0.0
        west_wall.pose.orientation.w = 1.0
        
        west_wall.scale.x = wall_thickness
        west_wall.scale.y = size_y
        west_wall.scale.z = self.wall_height
        
        west_wall.color.r = self.wall_color[0]
        west_wall.color.g = self.wall_color[1]
        west_wall.color.b = self.wall_color[2]
        west_wall.color.a = self.wall_color[3]
        
        markers.append(west_wall)
        
        return markers
    
    def create_obstacle_marker(self, obstacle):
        """Create a marker for an obstacle"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'environment'
        marker.id = self.next_marker_id
        self.next_marker_id += 1
        marker.action = Marker.ADD
        
        # Set position
        position = obstacle.get('position', [0.0, 0.0, 0.0])
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Set color
        marker.color.r = self.obstacle_color[0]
        marker.color.g = self.obstacle_color[1]
        marker.color.b = self.obstacle_color[2]
        marker.color.a = self.obstacle_color[3]
        
        # Set type and scale based on obstacle type
        obstacle_type = obstacle.get('type', 'box')
        
        if obstacle_type == 'box':
            marker.type = Marker.CUBE
            dimensions = obstacle.get('dimensions', [1.0, 1.0, 1.0])
            marker.scale.x = dimensions[0]
            marker.scale.y = dimensions[1]
            marker.scale.z = dimensions[2]
        elif obstacle_type == 'cylinder':
            marker.type = Marker.CYLINDER
            radius = obstacle.get('radius', 0.5)
            height = obstacle.get('height', 1.0)
            marker.scale.x = radius * 2
            marker.scale.y = radius * 2
            marker.scale.z = height
        elif obstacle_type == 'sphere':
            marker.type = Marker.SPHERE
            radius = obstacle.get('radius', 0.5)
            marker.scale.x = radius * 2
            marker.scale.y = radius * 2
            marker.scale.z = radius * 2
        else:
            self.get_logger().warn(f'Unknown obstacle type: {obstacle_type}')
            return None
        
        return marker
    
    def command_callback(self, msg):
        """Handle environment commands"""
        command = msg.data.lower()
        
        if command.startswith('load:'):
            # Load environment
            environment_name = command[5:]
            self.load_environment(environment_name)
        elif command == 'random':
            # Generate random environment
            self.generate_random_environment(
                self.default_size_x,
                self.default_size_y,
                self.default_obstacle_count
            )
        elif command == 'empty':
            # Generate empty environment
            self.generate_empty_environment(
                self.default_size_x,
                self.default_size_y
            )
        elif command.startswith('obstacle_course:'):
            # Generate obstacle course
            try:
                difficulty = int(command[16:])
                self.generate_obstacle_course(difficulty)
            except ValueError:
                self.get_logger().error(f'Invalid difficulty: {command[16:]}')
        elif command.startswith('maze:'):
            # Generate maze
            try:
                size = int(command[5:])
                self.generate_maze(size)
            except ValueError:
                self.get_logger().error(f'Invalid maze size: {command[5:]}')
        elif command.startswith('save:'):
            # Save environment
            filename = command[5:]
            self.save_environment(filename)
        else:
            # Try to load as environment name
            self.load_environment(command)

def main(args=None):
    rclpy.init(args=args)
    
    environment_generator = EnvironmentGenerator()
    
    try:
        rclpy.spin(environment_generator)
    except KeyboardInterrupt:
        pass
    finally:
        environment_generator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()