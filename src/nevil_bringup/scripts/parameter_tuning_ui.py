#!/usr/bin/env python3

"""
Parameter Tuning UI for Nevil-picar v2.0

This script provides a simple UI for tuning system parameters during development.
"""

import os
import sys
import time
import yaml
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters, ListParameters
from ament_index_python.packages import get_package_share_directory

class ParameterTuningUI(Node):
    """Parameter tuning UI node for Nevil-picar v2.0."""
    
    def __init__(self):
        """Initialize the parameter tuning UI node."""
        super().__init__('parameter_tuning_ui')
        
        # Declare parameters
        self.declare_parameter('config_file', '')
        
        # Get parameters
        config_file = self.get_parameter('config_file').value
        
        # Get package directories
        self.nevil_bringup_dir = get_package_share_directory('nevil_bringup')
        
        # Load configuration
        self.config = self.load_config(config_file)
        
        # Create clients for parameter services
        self.get_parameters_clients = {}
        self.set_parameters_clients = {}
        self.list_parameters_clients = {}
        
        # Discover nodes
        self.discover_nodes()
        
        # Start command-line UI
        self.start_cli()
        
        self.get_logger().info('Parameter tuning UI initialized')
    
    def load_config(self, config_file):
        """Load configuration from file."""
        if not config_file:
            config_file = os.path.join(self.nevil_bringup_dir, 'config', 'default_config.yaml')
        
        try:
            with open(config_file, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load configuration: {e}')
            return {}
    
    def discover_nodes(self):
        """Discover nodes in the system."""
        # In a real system, this would discover nodes dynamically
        # For now, we just hardcode some nodes
        nodes = [
            "system_manager",
            "navigation_node",
            "obstacle_detection",
            "camera_vision",
            "speech_recognition_node",
            "speech_synthesis_node",
            "text_command_processor",
            "rt_config_manager"
        ]
        
        # Create clients for each node
        for node in nodes:
            self.get_parameters_clients[node] = self.create_client(
                GetParameters,
                f'/{node}/get_parameters'
            )
            self.set_parameters_clients[node] = self.create_client(
                SetParameters,
                f'/{node}/set_parameters'
            )
            self.list_parameters_clients[node] = self.create_client(
                ListParameters,
                f'/{node}/list_parameters'
            )
    
    def get_parameter(self, node, name):
        """Get a parameter from a node."""
        client = self.get_parameters_clients.get(node)
        if client is None or not client.wait_for_service(timeout_sec=0.1):
            print(f"Node {node} not available")
            return None
        
        request = GetParameters.Request()
        request.names = [name]
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        
        if future.result() is not None:
            response = future.result()
            if len(response.values) > 0:
                return response.values[0]
        
        return None
    
    def set_parameter(self, node, name, value):
        """Set a parameter on a node."""
        client = self.set_parameters_clients.get(node)
        if client is None or not client.wait_for_service(timeout_sec=0.1):
            print(f"Node {node} not available")
            return False
        
        request = SetParameters.Request()
        parameter = Parameter()
        parameter.name = name
        
        # Set parameter value based on type
        parameter_value = ParameterValue()
        if isinstance(value, bool):
            parameter_value.type = ParameterType.PARAMETER_BOOL
            parameter_value.bool_value = value
        elif isinstance(value, int):
            parameter_value.type = ParameterType.PARAMETER_INTEGER
            parameter_value.integer_value = value
        elif isinstance(value, float):
            parameter_value.type = ParameterType.PARAMETER_DOUBLE
            parameter_value.double_value = value
        elif isinstance(value, str):
            parameter_value.type = ParameterType.PARAMETER_STRING
            parameter_value.string_value = value
        elif isinstance(value, list):
            if all(isinstance(x, int) for x in value):
                parameter_value.type = ParameterType.PARAMETER_INTEGER_ARRAY
                parameter_value.integer_array_value = value
            elif all(isinstance(x, float) for x in value):
                parameter_value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                parameter_value.double_array_value = value
            elif all(isinstance(x, str) for x in value):
                parameter_value.type = ParameterType.PARAMETER_STRING_ARRAY
                parameter_value.string_array_value = value
        
        parameter.value = parameter_value
        request.parameters = [parameter]
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        
        if future.result() is not None:
            response = future.result()
            if len(response.results) > 0 and response.results[0].successful:
                print(f"Parameter {name} set to {value} on {node}")
                return True
        
        print(f"Failed to set parameter {name} on {node}")
        return False
    
    def list_parameters(self, node):
        """List parameters of a node."""
        client = self.list_parameters_clients.get(node)
        if client is None or not client.wait_for_service(timeout_sec=0.1):
            print(f"Node {node} not available")
            return []
        
        request = ListParameters.Request()
        request.depth = 0  # Get all parameters
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        
        if future.result() is not None:
            response = future.result()
            return response.names
        
        return []
    
    def start_cli(self):
        """Start the command-line UI."""
        print("\nNevil-picar v2.0 Parameter Tuning UI")
        print("===================================\n")
        
        while True:
            print("\nAvailable commands:")
            print("  1. List nodes")
            print("  2. List parameters of a node")
            print("  3. Get parameter value")
            print("  4. Set parameter value")
            print("  5. Exit")
            
            choice = input("\nEnter your choice (1-5): ")
            
            if choice == '1':
                self.cmd_list_nodes()
            elif choice == '2':
                self.cmd_list_parameters()
            elif choice == '3':
                self.cmd_get_parameter()
            elif choice == '4':
                self.cmd_set_parameter()
            elif choice == '5':
                print("Exiting...")
                break
            else:
                print("Invalid choice. Please try again.")
            
            # Process ROS events
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def cmd_list_nodes(self):
        """List available nodes."""
        print("\nAvailable nodes:")
        for node in self.get_parameters_clients.keys():
            print(f"  - {node}")
    
    def cmd_list_parameters(self):
        """List parameters of a node."""
        node = input("\nEnter node name: ")
        
        if node not in self.list_parameters_clients:
            print(f"Node {node} not found")
            return
        
        parameters = self.list_parameters(node)
        
        if not parameters:
            print(f"No parameters found for node {node}")
            return
        
        print(f"\nParameters of node {node}:")
        for param in parameters:
            print(f"  - {param}")
    
    def cmd_get_parameter(self):
        """Get parameter value."""
        node = input("\nEnter node name: ")
        
        if node not in self.get_parameters_clients:
            print(f"Node {node} not found")
            return
        
        param_name = input("Enter parameter name: ")
        
        param_value = self.get_parameter(node, param_name)
        
        if param_value is None:
            print(f"Parameter {param_name} not found on node {node}")
            return
        
        # Display parameter value based on type
        if param_value.type == ParameterType.PARAMETER_BOOL:
            print(f"Value: {param_value.bool_value} (bool)")
        elif param_value.type == ParameterType.PARAMETER_INTEGER:
            print(f"Value: {param_value.integer_value} (int)")
        elif param_value.type == ParameterType.PARAMETER_DOUBLE:
            print(f"Value: {param_value.double_value} (float)")
        elif param_value.type == ParameterType.PARAMETER_STRING:
            print(f"Value: {param_value.string_value} (string)")
        elif param_value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            print(f"Value: {param_value.integer_array_value} (int[])")
        elif param_value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            print(f"Value: {param_value.double_array_value} (float[])")
        elif param_value.type == ParameterType.PARAMETER_STRING_ARRAY:
            print(f"Value: {param_value.string_array_value} (string[])")
        else:
            print(f"Value: {param_value} (unknown type)")
    
    def cmd_set_parameter(self):
        """Set parameter value."""
        node = input("\nEnter node name: ")
        
        if node not in self.set_parameters_clients:
            print(f"Node {node} not found")
            return
        
        param_name = input("Enter parameter name: ")
        param_type = input("Enter parameter type (bool, int, float, string, int[], float[], string[]): ")
        param_value_str = input("Enter parameter value: ")
        
        # Parse parameter value based on type
        try:
            if param_type == 'bool':
                param_value = param_value_str.lower() in ['true', 't', 'yes', 'y', '1']
            elif param_type == 'int':
                param_value = int(param_value_str)
            elif param_type == 'float':
                param_value = float(param_value_str)
            elif param_type == 'string':
                param_value = param_value_str
            elif param_type == 'int[]':
                param_value = [int(x.strip()) for x in param_value_str.split(',')]
            elif param_type == 'float[]':
                param_value = [float(x.strip()) for x in param_value_str.split(',')]
            elif param_type == 'string[]':
                param_value = [x.strip() for x in param_value_str.split(',')]
            else:
                print(f"Unsupported parameter type: {param_type}")
                return
        except ValueError:
            print(f"Invalid value for type {param_type}: {param_value_str}")
            return
        
        # Set parameter
        self.set_parameter(node, param_name, param_value)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    parameter_tuning_ui = ParameterTuningUI()
    
    try:
        rclpy.spin(parameter_tuning_ui)
    except KeyboardInterrupt:
        pass
    finally:
        parameter_tuning_ui.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()