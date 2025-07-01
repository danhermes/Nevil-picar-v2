#!/usr/bin/env python3
"""
Real-time Navigation Monitoring Dashboard for Nevil Robot

This script provides a real-time terminal interface to monitor all navigation-related
topics from the Nevil robot. It displays formatted, color-coded information about
robot movement, goals, system status, and AI commands.

Usage:
    python3 navigation_monitor.py [--log-file LOG_FILE] [--refresh-rate RATE]
    
Arguments:
    --log-file: Optional file to log all messages for later analysis
    --refresh-rate: Display refresh rate in milliseconds (default: 100)

Topics Monitored:
    - /cmd_vel: Movement commands (linear.x, angular.z)
    - /goal_pose: Navigation goals (position x,y,z)
    - /system_mode: Active/standby status
    - /nevil/action_command: AI action commands
    - /planned_path: Path planning output

Controls:
    - Ctrl+C: Clean shutdown
    - 'q': Quit (when using rich interface)
    - 'r': Reset message counters
    - 'p': Pause/resume monitoring

Author: Nevil Navigation Team
Version: 1.0
"""

import sys
import os
import argparse
import signal
import threading
import time
from datetime import datetime
from typing import Dict, Any, Optional, List
import json
import subprocess
import psutil

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from rcl_interfaces.msg import Log

# Try to import AI command message
try:
    from nevil_interfaces_ai_msgs.msg import AICommand
    AI_COMMAND_AVAILABLE = True
except ImportError:
    AI_COMMAND_AVAILABLE = False
    print("Warning: AI command messages not available")

# Try to import rich for better terminal formatting
try:
    from rich.console import Console
    from rich.table import Table
    from rich.live import Live
    from rich.panel import Panel
    from rich.text import Text
    from rich.layout import Layout
    RICH_AVAILABLE = True
except ImportError:
    RICH_AVAILABLE = False
    # Create dummy classes for type hints when Rich is not available
    class Table:
        pass
    class Console:
        pass
    print("Warning: Rich library not available, using basic terminal output")


class CriticalAlert:
    """Container for critical system alerts"""
    
    def __init__(self, alert_type: str, message: str, severity: str = "error"):
        self.alert_type = alert_type
        self.message = message
        self.severity = severity  # fatal, error, warning
        self.timestamp = datetime.now()
        self.count = 1
        
    def update(self):
        """Update alert count and timestamp"""
        self.count += 1
        self.timestamp = datetime.now()


class TopicData:
    """Container for topic monitoring data"""
    
    def __init__(self, topic_name: str):
        self.topic_name = topic_name
        self.latest_value = "No data"
        self.message_count = 0
        self.last_update = None
        self.status = "inactive"  # inactive, active, warning, error, fatal
        self.error_count = 0
        
    def update(self, value: str, is_error: bool = False):
        """Update topic data with new value"""
        self.latest_value = value
        self.message_count += 1
        self.last_update = datetime.now()
        
        if is_error:
            self.error_count += 1
            self.status = "fatal" if self.error_count > 5 else "error"
        else:
            self.status = "active"
        
    def get_status_color(self) -> str:
        """Get color code for status"""
        if not RICH_AVAILABLE:
            return ""
        
        color_map = {
            "active": "green",
            "inactive": "red",
            "warning": "yellow",
            "error": "red bold",
            "fatal": "red bold blink"
        }
        return color_map.get(self.status, "white")
        
    def get_age_seconds(self) -> float:
        """Get age of last message in seconds"""
        if self.last_update is None:
            return float('inf')
        return (datetime.now() - self.last_update).total_seconds()


class NavigationMonitor(Node):
    """ROS2 node for monitoring navigation topics and critical system health"""
    
    def __init__(self, log_file: Optional[str] = None):
        super().__init__('navigation_monitor')
        
        # Initialize topic data containers
        self.topics = {
            '/cmd_vel': TopicData('/cmd_vel'),
            '/goal_pose': TopicData('/goal_pose'),
            '/system_mode': TopicData('/system_mode'),
            '/nevil/action_command': TopicData('/nevil/action_command'),
            '/planned_path': TopicData('/planned_path')
        }
        
        # Critical monitoring
        self.critical_alerts: Dict[str, CriticalAlert] = {}
        self.node_health: Dict[str, str] = {}  # node_name -> status
        self.expected_nodes = [
            'navigation_node',
            'ai_interface_node',
            'dialog_manager_node',
            'hardware_bridge_node',
            'rt_motor_control_node'
        ]
        
        # Monitoring state
        self.monitoring_active = True
        self.log_file = log_file
        self.log_handle = None
        self.fatal_error_count = 0
        
        # Setup logging if requested
        if self.log_file:
            try:
                self.log_handle = open(self.log_file, 'w')
                self.log_handle.write(f"# Navigation Monitor Log - Started at {datetime.now()}\n")
                self.log_handle.write("# Format: timestamp,topic,value,count\n")
                self.get_logger().info(f"Logging to file: {self.log_file}")
            except Exception as e:
                self.get_logger().error(f"Failed to open log file: {e}")
        
        # QoS profile for reliable message delivery
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscribers for all monitored topics
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, reliable_qos)
            
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, reliable_qos)
            
        self.system_mode_sub = self.create_subscription(
            String, '/system_mode', self.system_mode_callback, reliable_qos)
            
        self.planned_path_sub = self.create_subscription(
            Path, '/planned_path', self.planned_path_callback, reliable_qos)
        
        # Subscribe to AI action commands if available
        if AI_COMMAND_AVAILABLE:
            self.action_command_sub = self.create_subscription(
                AICommand, '/nevil/action_command', self.action_command_callback, reliable_qos)
        else:
            # Mark as unavailable
            self.topics['/nevil/action_command'].latest_value = "Not available"
            self.topics['/nevil/action_command'].status = "error"
        
        # Subscribe to ROS2 logging for error detection
        self.log_sub = self.create_subscription(
            Log, '/rosout', self.log_callback, reliable_qos)
        
        # Timers for monitoring
        self.stale_check_timer = self.create_timer(1.0, self.check_stale_data)
        self.node_health_timer = self.create_timer(5.0, self.check_node_health)
        self.critical_alert_timer = self.create_timer(2.0, self.check_critical_alerts)
        
        self.get_logger().info('Navigation Monitor with Critical Error Detection initialized')
    
    def cmd_vel_callback(self, msg: Twist):
        """Handle cmd_vel messages"""
        value = f"lin:{msg.linear.x:.2f} ang:{msg.angular.z:.2f}"
        self.topics['/cmd_vel'].update(value)
        self.log_message('/cmd_vel', value)
    
    def goal_pose_callback(self, msg: PoseStamped):
        """Handle goal_pose messages"""
        pos = msg.pose.position
        value = f"x:{pos.x:.2f} y:{pos.y:.2f} z:{pos.z:.2f}"
        self.topics['/goal_pose'].update(value)
        self.log_message('/goal_pose', value)
    
    def system_mode_callback(self, msg: String):
        """Handle system_mode messages"""
        value = msg.data
        self.topics['/system_mode'].update(value)
        
        # Update status based on mode
        if value == "active":
            self.topics['/system_mode'].status = "active"
        elif value == "standby":
            self.topics['/system_mode'].status = "warning"
        else:
            self.topics['/system_mode'].status = "error"
            
        self.log_message('/system_mode', value)
    
    def action_command_callback(self, msg):
        """Handle AI action command messages"""
        if hasattr(msg, 'command_type'):
            value = msg.command_type
            if hasattr(msg, 'command_data') and msg.command_data:
                try:
                    data = json.loads(msg.command_data)
                    if data:
                        value += f" ({list(data.keys())[0] if data else ''})"
                except:
                    pass
        else:
            value = str(msg)[:50]  # Truncate long messages
            
        self.topics['/nevil/action_command'].update(value)
        self.log_message('/nevil/action_command', value)
    
    def planned_path_callback(self, msg: Path):
        """Handle planned_path messages"""
        num_poses = len(msg.poses)
        value = f"{num_poses} waypoints"
        if num_poses > 0:
            start = msg.poses[0].pose.position
            end = msg.poses[-1].pose.position
            value += f" ({start.x:.1f},{start.y:.1f})->({end.x:.1f},{end.y:.1f})"
        
        self.topics['/planned_path'].update(value)
        self.log_message('/planned_path', value)
    
    def log_callback(self, msg: Log):
        """Monitor ROS2 logs for critical errors"""
        try:
            # Check for fatal/error level logs (handle both int and bytes)
            msg_level = msg.level
            if isinstance(msg_level, bytes):
                msg_level = int.from_bytes(msg_level, byteorder='little')
            
            if msg_level >= 40:  # ERROR level (40) and above
                error_msg = f"{msg.name}: {msg.msg}"
                
                # Categorize critical errors
                if any(keyword in msg.msg.lower() for keyword in ['fatal', 'crash', 'segfault', 'abort']):
                    self.add_critical_alert("FATAL_ERROR", error_msg, "fatal")
                    self.fatal_error_count += 1
                elif any(keyword in msg.msg.lower() for keyword in ['failed', 'error', 'exception']):
                    self.add_critical_alert("ERROR", error_msg, "error")
                
                # Log to file if enabled
                if self.log_handle:
                    self.log_handle.write(f"ERROR,{datetime.now().isoformat()},{msg.name},{msg.msg}\n")
                    self.log_handle.flush()
        except Exception as e:
            # Don't let log processing errors crash the monitor
            self.get_logger().debug(f"Error processing log message: {e}")
    
    def add_critical_alert(self, alert_type: str, message: str, severity: str = "error"):
        """Add or update a critical alert"""
        alert_key = f"{alert_type}_{hash(message) % 1000}"
        
        if alert_key in self.critical_alerts:
            self.critical_alerts[alert_key].update()
        else:
            self.critical_alerts[alert_key] = CriticalAlert(alert_type, message, severity)
            
        # Limit alert history to prevent memory issues
        if len(self.critical_alerts) > 50:
            oldest_key = min(self.critical_alerts.keys(),
                           key=lambda k: self.critical_alerts[k].timestamp)
            del self.critical_alerts[oldest_key]
    
    def check_node_health(self):
        """Check health of critical ROS2 nodes"""
        try:
            # Get list of active nodes
            result = subprocess.run(['ros2', 'node', 'list'],
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                active_nodes = [node.strip() for node in result.stdout.split('\n') if node.strip()]
                
                # Check each expected node
                for node_name in self.expected_nodes:
                    node_found = any(node_name in node for node in active_nodes)
                    
                    if node_found:
                        self.node_health[node_name] = "active"
                    else:
                        self.node_health[node_name] = "missing"
                        self.add_critical_alert("NODE_MISSING", f"Critical node {node_name} not found", "error")
            else:
                self.add_critical_alert("ROS2_ERROR", "Failed to query ROS2 nodes", "error")
                
        except subprocess.TimeoutExpired:
            self.add_critical_alert("ROS2_TIMEOUT", "ROS2 node query timed out", "warning")
        except Exception as e:
            self.add_critical_alert("SYSTEM_ERROR", f"Node health check failed: {e}", "error")
    
    def check_critical_alerts(self):
        """Check for critical system conditions"""
        # Check system resources
        try:
            # Memory usage
            memory = psutil.virtual_memory()
            if memory.percent > 90:
                self.add_critical_alert("HIGH_MEMORY", f"Memory usage: {memory.percent:.1f}%", "warning")
            
            # CPU usage
            cpu_percent = psutil.cpu_percent(interval=1)
            if cpu_percent > 95:
                self.add_critical_alert("HIGH_CPU", f"CPU usage: {cpu_percent:.1f}%", "warning")
            
            # Check for zombie processes
            zombie_count = len([p for p in psutil.process_iter(['status']) if p.info['status'] == 'zombie'])
            if zombie_count > 0:
                self.add_critical_alert("ZOMBIE_PROCESSES", f"{zombie_count} zombie processes detected", "warning")
                
        except Exception as e:
            self.add_critical_alert("SYSTEM_MONITOR_ERROR", f"System monitoring failed: {e}", "error")
    
    def check_stale_data(self):
        """Check for stale data and update status accordingly"""
        current_time = datetime.now()
        
        for topic_data in self.topics.values():
            if topic_data.last_update is None:
                continue
                
            age = topic_data.get_age_seconds()
            
            # Mark as warning if no update for 5 seconds, error if 10 seconds, fatal if 30 seconds
            if age > 30:
                topic_data.status = "fatal"
                self.add_critical_alert("TOPIC_TIMEOUT", f"Topic {topic_data.topic_name} silent for {age:.1f}s", "fatal")
            elif age > 10:
                topic_data.status = "error"
                self.add_critical_alert("TOPIC_STALE", f"Topic {topic_data.topic_name} stale for {age:.1f}s", "error")
            elif age > 5:
                topic_data.status = "warning"
            elif topic_data.status in ["warning", "error", "fatal"]:
                topic_data.status = "active"  # Reset to active if receiving data
    
    def log_message(self, topic: str, value: str):
        """Log message to file if logging enabled"""
        if self.log_handle:
            timestamp = datetime.now().isoformat()
            count = self.topics[topic].message_count
            self.log_handle.write(f"{timestamp},{topic},{value},{count}\n")
            self.log_handle.flush()
    
    def reset_counters(self):
        """Reset all message counters"""
        for topic_data in self.topics.values():
            topic_data.message_count = 0
        self.get_logger().info("Message counters reset")
    
    def toggle_monitoring(self):
        """Toggle monitoring on/off"""
        self.monitoring_active = not self.monitoring_active
        status = "resumed" if self.monitoring_active else "paused"
        self.get_logger().info(f"Monitoring {status}")
    
    def shutdown(self):
        """Clean shutdown"""
        if self.log_handle:
            self.log_handle.close()
        self.get_logger().info("Navigation Monitor shutting down")


class TerminalDisplay:
    """Handle terminal display formatting"""
    
    def __init__(self, monitor: NavigationMonitor, refresh_rate: int = 100):
        self.monitor = monitor
        self.refresh_rate = refresh_rate / 1000.0  # Convert to seconds
        self.console = Console() if RICH_AVAILABLE else None
        self.running = True
        
    def create_rich_table(self) -> Table:
        """Create formatted table using Rich library"""
        # Main navigation table
        table = Table(title="🤖 NEVIL NAVIGATION MONITOR 🤖", show_header=True, header_style="bold magenta")
        
        table.add_column("Topic", style="cyan", width=20)
        table.add_column("Latest Value", style="white", width=30)
        table.add_column("Count", justify="right", style="yellow", width=8)
        table.add_column("Last Update", style="blue", width=12)
        table.add_column("Status", justify="center", width=12)
        
        for topic_data in self.monitor.topics.values():
            # Format timestamp
            if topic_data.last_update:
                time_str = topic_data.last_update.strftime("%H:%M:%S")
            else:
                time_str = "Never"
            
            # Status indicator with enhanced critical states
            if topic_data.status == "active":
                status_text = "🟢 Active"
            elif topic_data.status == "warning":
                status_text = "🟡 Warning"
            elif topic_data.status == "error":
                status_text = "🔴 Error"
            elif topic_data.status == "fatal":
                status_text = "💀 FATAL"
            else:
                status_text = "⚫ Inactive"
            
            table.add_row(
                topic_data.topic_name,
                topic_data.latest_value[:28] + "..." if len(topic_data.latest_value) > 28 else topic_data.latest_value,
                str(topic_data.message_count),
                time_str,
                status_text
            )
        
        return table
    
    def create_alerts_table(self) -> Table:
        """Create critical alerts table"""
        alerts_table = Table(title="🚨 CRITICAL ALERTS 🚨", show_header=True, header_style="bold red")
        
        alerts_table.add_column("Severity", style="red", width=8)
        alerts_table.add_column("Type", style="yellow", width=15)
        alerts_table.add_column("Message", style="white", width=40)
        alerts_table.add_column("Count", justify="right", style="cyan", width=6)
        alerts_table.add_column("Last Seen", style="blue", width=10)
        
        # Sort alerts by severity and timestamp
        sorted_alerts = sorted(self.monitor.critical_alerts.values(),
                             key=lambda a: (a.severity == "fatal", a.severity == "error", a.timestamp),
                             reverse=True)
        
        for alert in sorted_alerts[:10]:  # Show only latest 10 alerts
            severity_icon = "💀" if alert.severity == "fatal" else "🔴" if alert.severity == "error" else "🟡"
            time_str = alert.timestamp.strftime("%H:%M:%S")
            
            alerts_table.add_row(
                f"{severity_icon} {alert.severity.upper()}",
                alert.alert_type,
                alert.message[:38] + "..." if len(alert.message) > 38 else alert.message,
                str(alert.count),
                time_str
            )
        
        return alerts_table
    
    def create_node_health_table(self) -> Table:
        """Create node health monitoring table"""
        health_table = Table(title="🏥 NODE HEALTH 🏥", show_header=True, header_style="bold green")
        
        health_table.add_column("Node", style="cyan", width=25)
        health_table.add_column("Status", justify="center", style="white", width=15)
        
        for node_name, status in self.monitor.node_health.items():
            status_icon = "🟢 ACTIVE" if status == "active" else "🔴 MISSING"
            health_table.add_row(node_name, status_icon)
        
        return health_table
    
    def create_basic_display(self) -> str:
        """Create basic text display for terminals without Rich"""
        lines = []
        lines.append("=" * 100)
        lines.append("🤖 NEVIL NAVIGATION MONITOR WITH CRITICAL ALERTS 🤖".center(100))
        lines.append("=" * 100)
        
        # Critical alerts section (most important)
        if self.monitor.critical_alerts:
            lines.append("🚨 CRITICAL ALERTS 🚨".center(100))
            lines.append("-" * 100)
            lines.append(f"{'Severity':<10} | {'Type':<15} | {'Message':<50} | {'Count':<6} | {'Time'}")
            lines.append("-" * 100)
            
            sorted_alerts = sorted(self.monitor.critical_alerts.values(),
                                 key=lambda a: (a.severity == "fatal", a.severity == "error", a.timestamp),
                                 reverse=True)
            
            for alert in sorted_alerts[:5]:  # Show top 5 critical alerts
                time_str = alert.timestamp.strftime("%H:%M:%S")
                severity = alert.severity.upper()
                if alert.severity == "fatal":
                    severity = "💀 FATAL"
                elif alert.severity == "error":
                    severity = "🔴 ERROR"
                else:
                    severity = "🟡 WARN"
                
                message = alert.message[:48] + "..." if len(alert.message) > 48 else alert.message
                lines.append(f"{severity:<10} | {alert.alert_type:<15} | {message:<50} | {alert.count:<6} | {time_str}")
            
            lines.append("=" * 100)
        
        # Navigation topics section
        lines.append("📡 NAVIGATION TOPICS 📡".center(100))
        lines.append("-" * 100)
        lines.append(f"{'Topic':<20} | {'Latest Value':<30} | {'Count':<8} | {'Last Update':<12} | {'Status'}")
        lines.append("-" * 100)
        
        for topic_data in self.monitor.topics.values():
            time_str = topic_data.last_update.strftime("%H:%M:%S") if topic_data.last_update else "Never"
            
            status_map = {
                "active": "🟢 ACTIVE",
                "warning": "🟡 WARN",
                "error": "🔴 ERROR",
                "fatal": "💀 FATAL",
                "inactive": "⚫ INACTIVE"
            }
            status = status_map.get(topic_data.status, "UNKNOWN")
            
            value = topic_data.latest_value[:28] + "..." if len(topic_data.latest_value) > 28 else topic_data.latest_value
            
            lines.append(f"{topic_data.topic_name:<20} | {value:<30} | {topic_data.message_count:<8} | {time_str:<12} | {status}")
        
        # Node health section
        lines.append("-" * 100)
        lines.append("🏥 NODE HEALTH 🏥".center(100))
        lines.append("-" * 100)
        for node_name, status in self.monitor.node_health.items():
            status_icon = "🟢 ACTIVE" if status == "active" else "🔴 MISSING"
            lines.append(f"{node_name:<30} | {status_icon}")
        
        lines.append("-" * 100)
        lines.append(f"Monitoring: {'ACTIVE' if self.monitor.monitoring_active else 'PAUSED'} | "
                    f"Fatal Errors: {self.monitor.fatal_error_count} | "
                    f"Total Alerts: {len(self.monitor.critical_alerts)}")
        lines.append("Controls: Ctrl+C=Quit, r=Reset counters, p=Pause/Resume")
        lines.append("=" * 100)
        
        return "\n".join(lines)
    
    def run_rich_display(self):
        """Run display using Rich library with multiple panels"""
        try:
            from rich.layout import Layout
            from rich.panel import Panel
            
            layout = Layout()
            layout.split_column(
                Layout(name="alerts", size=8),
                Layout(name="main"),
                Layout(name="health", size=6)
            )
            
            with Live(layout, refresh_per_second=1/self.refresh_rate, screen=True) as live:
                while self.running and rclpy.ok():
                    if self.monitor.monitoring_active:
                        # Update alerts panel (highest priority)
                        if self.monitor.critical_alerts:
                            layout["alerts"].update(Panel(self.create_alerts_table(),
                                                        title="🚨 CRITICAL ALERTS",
                                                        border_style="red"))
                        else:
                            layout["alerts"].update(Panel("✅ No Critical Alerts",
                                                        title="🚨 CRITICAL ALERTS",
                                                        border_style="green"))
                        
                        # Update main navigation panel
                        layout["main"].update(Panel(self.create_rich_table(),
                                                  title="📡 Navigation Topics",
                                                  border_style="blue"))
                        
                        # Update node health panel
                        layout["health"].update(Panel(self.create_node_health_table(),
                                                    title="🏥 Node Health",
                                                    border_style="green"))
                    time.sleep(self.refresh_rate)
        except KeyboardInterrupt:
            pass
        except ImportError:
            # Fallback to simple table if Layout not available
            with Live(self.create_rich_table(), refresh_per_second=1/self.refresh_rate, screen=True) as live:
                while self.running and rclpy.ok():
                    if self.monitor.monitoring_active:
                        live.update(self.create_rich_table())
                    time.sleep(self.refresh_rate)
    
    def run_basic_display(self):
        """Run basic terminal display"""
        try:
            while self.running and rclpy.ok():
                if self.monitor.monitoring_active:
                    # Clear screen
                    os.system('clear' if os.name == 'posix' else 'cls')
                    print(self.create_basic_display())
                time.sleep(self.refresh_rate)
        except KeyboardInterrupt:
            pass
    
    def run(self):
        """Start the display"""
        if RICH_AVAILABLE:
            self.run_rich_display()
        else:
            self.run_basic_display()
    
    def stop(self):
        """Stop the display"""
        self.running = False


def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully"""
    print("\nShutting down navigation monitor...")
    sys.exit(0)


def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='Real-time Navigation Monitor for Nevil Robot')
    parser.add_argument('--log-file', type=str, help='File to log messages for analysis')
    parser.add_argument('--refresh-rate', type=int, default=100, 
                       help='Display refresh rate in milliseconds (default: 100)')
    
    args = parser.parse_args()
    
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create monitor node
        monitor = NavigationMonitor(log_file=args.log_file)
        
        # Create display
        display = TerminalDisplay(monitor, args.refresh_rate)
        
        # Run ROS2 spinning in separate thread
        spin_thread = threading.Thread(target=lambda: rclpy.spin(monitor), daemon=True)
        spin_thread.start()
        
        print("Starting Navigation Monitor...")
        print("Press Ctrl+C to quit")
        if args.log_file:
            print(f"Logging to: {args.log_file}")
        
        # Run display (blocking)
        display.run()
        
    except KeyboardInterrupt:
        print("\nShutdown requested...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        try:
            if 'monitor' in locals():
                monitor.shutdown()
                monitor.destroy_node()
        except Exception:
            pass  # Ignore cleanup errors
        
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # Ignore shutdown errors
        
        print("Navigation Monitor stopped.")


if __name__ == '__main__':
    main()