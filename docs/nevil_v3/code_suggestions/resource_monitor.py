# nevil_framework/resource_monitor.py

import psutil
import threading
import time
from typing import Dict, Any, Optional, Callable
from dataclasses import dataclass
from enum import Enum

class ResourceAlert(Enum):
    CPU_HIGH = "cpu_high"
    MEMORY_HIGH = "memory_high"
    DISK_HIGH = "disk_high"
    SWAP_HIGH = "swap_high"
    OPEN_FILES_HIGH = "open_files_high"

@dataclass
class ResourceLimits:
    max_memory_mb: Optional[int] = None
    max_cpu_percent: Optional[float] = None
    max_open_files: Optional[int] = None
    max_disk_usage_percent: Optional[float] = None

    # Alert thresholds (percentage of limits)
    cpu_alert_threshold: float = 80.0
    memory_alert_threshold: float = 85.0
    disk_alert_threshold: float = 90.0

class ResourceMonitor:
    """
    System and process resource monitoring with alerting.

    Monitors CPU, memory, disk, and other system resources
    with configurable limits and alert callbacks.
    """

    def __init__(self, limits: ResourceLimits = None):
        self.limits = limits or ResourceLimits()
        self.monitoring = False
        self.monitor_thread = None
        self.alert_callbacks = {}  # ResourceAlert -> callback

        # Statistics
        self.resource_history = {}  # metric -> deque of recent values
        self.alert_count = {}  # ResourceAlert -> count
        self.last_alert_time = {}  # ResourceAlert -> timestamp

        # Configuration
        self.monitor_interval = 5.0  # seconds
        self.history_length = 100   # number of data points to keep
        self.alert_cooldown = 60.0  # seconds between same alert types

    def start_monitoring(self):
        """Start resource monitoring"""
        if self.monitoring:
            return

        self.monitoring = True
        self.monitor_thread = threading.Thread(
            target=self._monitoring_loop,
            name="ResourceMonitor",
            daemon=True
        )
        self.monitor_thread.start()

    def stop_monitoring(self):
        """Stop resource monitoring"""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)

    def add_alert_callback(self, alert_type: ResourceAlert, callback: Callable):
        """Add callback for resource alerts"""
        self.alert_callbacks[alert_type] = callback

    def _monitoring_loop(self):
        """Main monitoring loop"""
        from collections import deque

        # Initialize history tracking
        metrics = ['cpu_percent', 'memory_mb', 'disk_percent', 'open_files']
        for metric in metrics:
            self.resource_history[metric] = deque(maxlen=self.history_length)

        while self.monitoring:
            try:
                current_time = time.time()

                # Collect system metrics
                cpu_percent = psutil.cpu_percent(interval=1.0)
                memory_info = psutil.virtual_memory()
                memory_mb = memory_info.used / 1024 / 1024
                disk_usage = psutil.disk_usage('/')
                disk_percent = disk_usage.percent

                # Process-specific metrics
                process = psutil.Process()
                process_memory_mb = process.memory_info().rss / 1024 / 1024
                open_files = len(process.open_files()) if hasattr(process, 'open_files') else 0

                # Store in history
                self.resource_history['cpu_percent'].append(cpu_percent)
                self.resource_history['memory_mb'].append(process_memory_mb)
                self.resource_history['disk_percent'].append(disk_percent)
                self.resource_history['open_files'].append(open_files)

                # Check for alerts
                self._check_resource_alerts(current_time, {
                    'cpu_percent': cpu_percent,
                    'memory_mb': process_memory_mb,
                    'disk_percent': disk_percent,
                    'open_files': open_files
                })

                time.sleep(self.monitor_interval)

            except Exception as e:
                print(f"Error in resource monitoring: {e}")
                time.sleep(self.monitor_interval)

    def _check_resource_alerts(self, current_time: float, metrics: Dict[str, float]):
        """Check metrics against thresholds and trigger alerts"""

        # CPU usage alert
        if (self.limits.max_cpu_percent and
            metrics['cpu_percent'] > self.limits.max_cpu_percent * self.limits.cpu_alert_threshold / 100):
            self._trigger_alert(ResourceAlert.CPU_HIGH, current_time, {
                'current': metrics['cpu_percent'],
                'limit': self.limits.max_cpu_percent
            })

        # Memory usage alert
        if (self.limits.max_memory_mb and
            metrics['memory_mb'] > self.limits.max_memory_mb * self.limits.memory_alert_threshold / 100):
            self._trigger_alert(ResourceAlert.MEMORY_HIGH, current_time, {
                'current_mb': metrics['memory_mb'],
                'limit_mb': self.limits.max_memory_mb
            })

        # Disk usage alert
        if metrics['disk_percent'] > self.limits.disk_alert_threshold:
            self._trigger_alert(ResourceAlert.DISK_HIGH, current_time, {
                'current_percent': metrics['disk_percent'],
                'threshold': self.limits.disk_alert_threshold
            })

        # Open files alert
        if (self.limits.max_open_files and
            metrics['open_files'] > self.limits.max_open_files * 0.8):
            self._trigger_alert(ResourceAlert.OPEN_FILES_HIGH, current_time, {
                'current': metrics['open_files'],
                'limit': self.limits.max_open_files
            })

    def _trigger_alert(self, alert_type: ResourceAlert, current_time: float, data: Dict[str, Any]):
        """Trigger resource alert with cooldown"""

        # Check cooldown
        if (alert_type in self.last_alert_time and
            current_time - self.last_alert_time[alert_type] < self.alert_cooldown):
            return

        self.last_alert_time[alert_type] = current_time
        self.alert_count[alert_type] = self.alert_count.get(alert_type, 0) + 1

        print(f"RESOURCE ALERT: {alert_type.value} - {data}")

        # Call registered callback
        if alert_type in self.alert_callbacks:
            try:
                self.alert_callbacks[alert_type](alert_type, data)
            except Exception as e:
                print(f"Error in resource alert callback: {e}")

    def get_current_usage(self) -> Dict[str, Any]:
        """Get current resource usage"""
        try:
            process = psutil.Process()
            memory_info = process.memory_info()

            return {
                'cpu_percent': process.cpu_percent(),
                'memory_mb': memory_info.rss / 1024 / 1024,
                'memory_percent': process.memory_percent(),
                'open_files': len(process.open_files()) if hasattr(process, 'open_files') else 0,
                'threads': process.num_threads(),
                'disk_usage_percent': psutil.disk_usage('/').percent,
                'timestamp': time.time()
            }
        except Exception as e:
            print(f"Error getting resource usage: {e}")
            return {}

    def get_resource_history(self, metric: str, limit: int = None) -> list:
        """Get historical data for a metric"""
        if metric in self.resource_history:
            history = list(self.resource_history[metric])
            return history[-limit:] if limit else history
        return []

    def get_alert_statistics(self) -> Dict[str, Any]:
        """Get alert statistics"""
        return {
            'alert_counts': dict(self.alert_count),
            'last_alert_times': {k.value: v for k, v in self.last_alert_time.items()},
            'monitoring_active': self.monitoring
        }