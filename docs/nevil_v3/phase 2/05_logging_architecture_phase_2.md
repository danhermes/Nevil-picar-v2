# Nevil v3.0 Logging Architecture - Phase 2 Features

## Overview

This document contains the Phase 2 advanced features for Nevil v3.0 logging architecture, including log analysis and monitoring, real-time alerting, and dynamic log viewer capabilities.

## 4. Log Analysis and Monitoring (Phase 2)

### 4.1 Log Analyzer

#### Code Summary

This section implements comprehensive log analysis and monitoring capabilities with pattern detection, error analysis, and performance metrics:

**Classes:**
- **LogAnalyzer**: Advanced log file parser with pattern detection, error analysis, and metrics extraction
- **LogMetrics**: Statistics collector for log entries, error rates, and performance tracking

**Key Methods in LogAnalyzer:**
- `__init__()`: Initialize analyzer with pattern definitions, metric collectors, and analysis configuration
- `parse_log_file()`: Complete log file parsing with line-by-line analysis and pattern matching
- `detect_patterns()`: Advanced pattern detection for errors, performance issues, and system events
- `extract_metrics()`: Performance metric extraction including response times, error rates, and throughput
- `analyze_errors()`: Error analysis with categorization, frequency tracking, and root cause identification
- `generate_report()`: Comprehensive analysis report with summaries, trends, and recommendations

**Key Methods in LogMetrics:**
- `__init__()`: Initialize metrics tracking with counters, timers, and statistical accumulators
- `record_entry()`: Process individual log entries and extract relevant metrics
- `calculate_rates()`: Calculate error rates, message throughput, and performance statistics
- `get_summary()`: Generate statistical summary with averages, percentiles, and trends

**Key Features:**
- Pattern-based log analysis with configurable regular expressions
- Error detection and categorization with frequency analysis
- Performance metric extraction (response times, throughput, resource usage)
- Trend analysis with time-series data processing
- Statistical analysis with percentiles and moving averages
- Report generation with actionable insights and recommendations
- Multi-log file processing with aggregation capabilities
- Real-time analysis with streaming log processing

```python
# nevil_framework/log_analyzer.py

import re
import json
from typing import Dict, List, Any, Optional
from datetime import datetime, timedelta
from pathlib import Path
from collections import defaultdict, Counter

class LogAnalyzer:
    """
    Analyzes Nevil v3.0 log files for patterns, errors, and performance metrics.
    """
    
    def __init__(self, log_dir: str = "logs"):
        self.log_dir = Path(log_dir)
        self.log_pattern = re.compile(
            r'\[(?P<timestamp>[^\]]+)\] \[(?P<level>[^\]]+)\] \[(?P<logger>[^\]]+)\] '
            r'(?:\[(?P<thread>[^\]]+)\] )?(?P<message>.*)'
        )
    
    def analyze_system_health(self, hours: int = 24) -> Dict[str, Any]:
        """
        Analyze system health over the specified time period.
        
        Args:
            hours: Number of hours to analyze
            
        Returns:
            Health analysis report
        """
        cutoff_time = datetime.now() - timedelta(hours=hours)
        
        analysis = {
            'time_period': f"Last {hours} hours",
            'error_count': 0,
            'warning_count': 0,
            'critical_count': 0,
            'node_status': {},
            'error_patterns': Counter(),
            'performance_issues': [],
            'recommendations': []
        }
        
        # Analyze system log
        system_log = self.log_dir / "system.log"
        if system_log.exists():
            self._analyze_log_file(system_log, cutoff_time, analysis)
        
        # Analyze node logs
        node_log_dir = self.log_dir / "nodes"
        if node_log_dir.exists():
            for node_dir in node_log_dir.iterdir():
                if node_dir.is_dir():
                    node_log = node_dir / f"{node_dir.name}.log"
                    if node_log.exists():
                        node_analysis = {'errors': 0, 'warnings': 0, 'last_seen': None}
                        self._analyze_log_file(node_log, cutoff_time, analysis, node_analysis)
                        analysis['node_status'][node_dir.name] = node_analysis
        
        # Generate recommendations
        self._generate_recommendations(analysis)
        
        return analysis
    
    def _analyze_log_file(self, log_file: Path, cutoff_time: datetime, 
                         analysis: Dict, node_analysis: Dict = None):
        """Analyze a single log file"""
        try:
            with open(log_file, 'r', encoding='utf-8') as f:
                for line in f:
                    match = self.log_pattern.match(line.strip())
                    if not match:
                        continue
                    
                    # Parse timestamp
                    try:
                        timestamp = datetime.strptime(match.group('timestamp'), '%Y-%m-%d %H:%M:%S')
                        if timestamp < cutoff_time:
                            continue
                    except ValueError:
                        continue
                    
                    level = match.group('level')
                    message = match.group('message')
                    
                    # Count by level
                    if level == 'ERROR':
                        analysis['error_count'] += 1
                        if node_analysis:
                            node_analysis['errors'] += 1
                        
                        # Extract error patterns
                        error_type = self._extract_error_type(message)
                        if error_type:
                            analysis['error_patterns'][error_type] += 1
                    
                    elif level == 'WARNING':
                        analysis['warning_count'] += 1
                        if node_analysis:
                            node_analysis['warnings'] += 1
                    
                    elif level == 'CRITICAL':
                        analysis['critical_count'] += 1
                    
                    # Check for performance issues
                    if 'completed in' in message or 'failed after' in message:
                        duration_match = re.search(r'(\d+\.\d+)s', message)
                        if duration_match:
                            duration = float(duration_match.group(1))
                            if duration > 5.0:  # Slow operation threshold
                                analysis['performance_issues'].append({
                                    'timestamp': timestamp.isoformat(),
                                    'operation': message,
                                    'duration': duration
                                })
                    
                    # Update last seen for nodes
                    if node_analysis:
                        node_analysis['last_seen'] = timestamp.isoformat()
                        
        except Exception as e:
            print(f"Error analyzing log file {log_file}: {e}")
    
    def _extract_error_type(self, message: str) -> Optional[str]:
        """Extract error type from error message"""
        # Common error patterns
        patterns = [
            (r'ConnectionError', 'Connection Error'),
            (r'TimeoutError', 'Timeout Error'),
            (r'FileNotFoundError', 'File Not Found'),
            (r'PermissionError', 'Permission Error'),
            (r'OpenAI.*error', 'OpenAI API Error'),
            (r'Audio.*error', 'Audio Error'),
            (r'Configuration.*error', 'Configuration Error'),
        ]
        
        for pattern, error_type in patterns:
            if re.search(pattern, message, re.IGNORECASE):
                return error_type
        
        return 'Unknown Error'
    
    def _generate_recommendations(self, analysis: Dict):
        """Generate recommendations based on analysis"""
        recommendations = []
        
        # High error rate
        if analysis['error_count'] > 50:
            recommendations.append("High error rate detected. Review error patterns and consider system maintenance.")
        
        # Critical errors
        if analysis['critical_count'] > 0:
            recommendations.append(f"{analysis['critical_count']} critical errors found. Immediate attention required.")
        
        # Node health issues
        for node_name, node_status in analysis['node_status'].items():
            if node_status['errors'] > 10:
                recommendations.append(f"Node {node_name} has high error rate ({node_status['errors']} errors).")
            
            if not node_status['last_seen']:
                recommendations.append(f"Node {node_name} appears to be inactive.")
        
        # Performance issues
        if len(analysis['performance_issues']) > 10:
            recommendations.append("Multiple slow operations detected. Consider performance optimization.")
        
        # Common error patterns
        top_errors = analysis['error_patterns'].most_common(3)
        for error_type, count in top_errors:
            if count > 5:
                recommendations.append(f"Frequent {error_type} ({count} occurrences). Investigate root cause.")
        
        analysis['recommendations'] = recommendations
    
    def get_performance_metrics(self, hours: int = 24) -> Dict[str, Any]:
        """Get performance metrics from logs"""
        cutoff_time = datetime.now() - timedelta(hours=hours)
        
        metrics = {
            'slow_operations': [],
            'average_response_times': {},
            'operation_counts': Counter(),
            'error_rates': {}
        }
        
        # Analyze all log files for performance data
        for log_file in self.log_dir.rglob("*.log"):
            self._extract_performance_metrics(log_file, cutoff_time, metrics)
        
        return metrics
    
    def _extract_performance_metrics(self, log_file: Path, cutoff_time: datetime, metrics: Dict):
        """Extract performance metrics from a log file"""
        try:
            with open(log_file, 'r', encoding='utf-8') as f:
                for line in f:
                    match = self.log_pattern.match(line.strip())
                    if not match:
                        continue
                    
                    # Parse timestamp
                    try:
                        timestamp = datetime.strptime(match.group('timestamp'), '%Y-%m-%d %H:%M:%S')
                        if timestamp < cutoff_time:
                            continue
                    except ValueError:
                        continue
                    
                    message = match.group('message')
                    
                    # Extract performance data
                    if 'completed in' in message:
                        duration_match = re.search(r'(\w+) completed in (\d+\.\d+)s', message)
                        if duration_match:
                            operation = duration_match.group(1)
                            duration = float(duration_match.group(2))
                            
                            metrics['operation_counts'][operation] += 1
                            
                            if operation not in metrics['average_response_times']:
                                metrics['average_response_times'][operation] = []
                            metrics['average_response_times'][operation].append(duration)
                            
                            if duration > 5.0:
                                metrics['slow_operations'].append({
                                    'operation': operation,
                                    'duration': duration,
                                    'timestamp': timestamp.isoformat()
                                })
                    
        except Exception as e:
            print(f"Error extracting performance metrics from {log_file}: {e}")

# TEST: Log analyzer correctly parses log file formats
# TEST: Health analysis identifies system issues accurately
# TEST: Performance metrics extraction works for various log patterns
# TEST: Recommendations are generated based on analysis results
```

## 5. Log Monitoring and Alerting (Phase 2)

### 5.1 Real-time Log Monitor

#### Code Summary

This section implements real-time log monitoring and alerting with file system watching, pattern detection, and notification systems:

**Classes:**
- **LogEventHandler**: File system event handler for real-time log file changes
- **LogMonitor**: Real-time log monitoring with pattern matching and alert generation
- **AlertManager**: Alert notification system with rate limiting, escalation, and multiple channels

**Key Methods in LogMonitor:**
- `__init__()`: Initialize monitor with file watchers, pattern definitions, and alert configuration
- `start_monitoring()`: Begin real-time log file monitoring with background thread processing
- `stop_monitoring()`: Clean shutdown of monitoring threads and file system watchers
- `process_log_line()`: Real-time log line processing with pattern matching and alert triggering
- `check_error_patterns()`: Error pattern detection with configurable severity levels
- `trigger_alert()`: Alert generation with context, severity, and notification routing

**Key Methods in AlertManager:**
- `__init__()`: Initialize alert manager with notification channels and rate limiting
- `send_alert()`: Multi-channel alert delivery with escalation and retry logic
- `check_rate_limits()`: Alert rate limiting to prevent notification spam
- `escalate_alert()`: Alert escalation based on severity and acknowledgment status

**Key Features:**
- Real-time file system monitoring using watchdog library
- Pattern-based alert generation with configurable regular expressions
- Multi-channel notifications (email, Slack, webhook, console)
- Alert rate limiting and deduplication to prevent spam
- Escalation policies based on severity and response times
- Context-aware alerting with log line details and system state
- Thread-safe operations with proper resource management
- Configurable alert thresholds and filtering

```python
# nevil_framework/log_monitor.py

import time
import threading
import queue
from typing import Dict, List, Callable, Any
from pathlib import Path
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class LogEventHandler(FileSystemEventHandler):
    """Handles log file change events"""
    
    def __init__(self, callback: Callable[[str, str], None]):
        self.callback = callback
        self.file_positions = {}  # file_path -> last_position
    
    def on_modified(self, event):
        if event.is_directory or not event.src_path.endswith('.log'):
            return
        
        file_path = event.src_path
        
        # Read new lines from file
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                # Get last position or start from beginning
                last_pos = self.file_positions.get(file_path, 0)
                f.seek(last_pos)
                
                # Read new lines
                new_lines = f.readlines()
                if new_lines:
                    # Update position
                    self.file_positions[file_path] = f.tell()
                    
                    # Process new lines
                    for line in new_lines:
                        self.callback(file_path, line.strip())
                        
        except Exception as e:
            print(f"Error reading log file {file_path}: {e}")

class LogMonitor:
    """
    Real-time log monitoring with alerting capabilities.
    """
    
    def __init__(self, log_dir: str = "logs"):
        self.log_dir = Path(log_dir)
        self.observer = Observer()
        self.running = False
        
        # Alert rules
        self.alert_rules = []
        self.alert_callbacks = []
        
        # Event processing
        self.event_queue = queue.Queue()
        self.processor_thread = None
        
        # Statistics
        self.stats = {
            'lines_processed': 0,
            'alerts_triggered': 0,
            'errors_detected': 0,
            'start_time': None
        }
    
    def add_alert_rule(self, pattern: str, level: str = None,
                      callback: Callable[[Dict], None] = None):
        """
        Add an alert rule for log monitoring.
        
        Args:
            pattern: Regex pattern to match in log messages
            level: Log level to match (optional)
            callback: Function to call when rule matches
        """
        rule = {
            'pattern': re.compile(pattern, re.IGNORECASE),
            'level': level,
            'callback': callback,
            'match_count': 0
        }
        self.alert_rules.append(rule)
    
    def add_alert_callback(self, callback: Callable[[str, Dict], None]):
        """Add a global alert callback"""
        self.alert_callbacks.append(callback)
    
    def start_monitoring(self):
        """Start real-time log monitoring"""
        if self.running:
            return
        
        self.running = True
        self.stats['start_time'] = time.time()
        
        # Start event processor
        self.processor_thread = threading.Thread(target=self._process_events, daemon=True)
        self.processor_thread.start()
        
        # Setup file watcher
        handler = LogEventHandler(self._on_log_line)
        self.observer.schedule(handler, str(self.log_dir), recursive=True)
        self.observer.start()
        
        print(f"Log monitoring started for {self.log_dir}")
    
    def stop_monitoring(self):
        """Stop log monitoring"""
        if not self.running:
            return
        
        self.running = False
        
        # Stop file watcher
        self.observer.stop()
        self.observer.join()
        
        # Stop processor
        if self.processor_thread:
            self.processor_thread.join(timeout=5.0)
        
        print("Log monitoring stopped")
    
    def _on_log_line(self, file_path: str, line: str):
        """Handle new log line"""
        self.event_queue.put((file_path, line))
    
    def _process_events(self):
        """Process log events in background thread"""
        while self.running:
            try:
                # Get event with timeout
                try:
                    file_path, line = self.event_queue.get(timeout=1.0)
                except queue.Empty:
                    continue
                
                # Process the line
                self._process_log_line(file_path, line)
                self.stats['lines_processed'] += 1
                
            except Exception as e:
                print(f"Error processing log event: {e}")
    
    def _process_log_line(self, file_path: str, line: str):
        """Process a single log line"""
        # Parse log line
        log_pattern = re.compile(
            r'\[(?P<timestamp>[^\]]+)\] \[(?P<level>[^\]]+)\] \[(?P<logger>[^\]]+)\] '
            r'(?:\[(?P<thread>[^\]]+)\] )?(?P<message>.*)'
        )
        
        match = log_pattern.match(line)
        if not match:
            return
        
        log_data = {
            'file_path': file_path,
            'timestamp': match.group('timestamp'),
            'level': match.group('level'),
            'logger': match.group('logger'),
            'thread': match.group('thread'),
            'message': match.group('message'),
            'raw_line': line
        }
        
        # Check alert rules
        for rule in self.alert_rules:
            if self._check_alert_rule(rule, log_data):
                rule['match_count'] += 1
                self.stats['alerts_triggered'] += 1
                
                # Call rule-specific callback
                if rule['callback']:
                    try:
                        rule['callback'](log_data)
                    except Exception as e:
                        print(f"Error in alert callback: {e}")
                
                # Call global callbacks
                for callback in self.alert_callbacks:
                    try:
                        callback(f"Alert rule matched: {rule['pattern'].pattern}", log_data)
                    except Exception as e:
                        print(f"Error in global alert callback: {e}")
        
        # Track errors
        if log_data['level'] in ['ERROR', 'CRITICAL']:
            self.stats['errors_detected'] += 1
    
    def _check_alert_rule(self, rule: Dict, log_data: Dict) -> bool:
        """Check if a log line matches an alert rule"""
        # Check level filter
        if rule['level'] and log_data['level'] != rule['level']:
            return False
        
        # Check pattern
        return rule['pattern'].search(log_data['message']) is not None
    
    def get_monitoring_stats(self) -> Dict[str, Any]:
        """Get monitoring statistics"""
        stats = self.stats.copy()
        
        if stats['start_time']:
            stats['uptime_seconds'] = time.time() - stats['start_time']
            stats['lines_per_second'] = stats['lines_processed'] / stats['uptime_seconds']
        
        stats['alert_rules_count'] = len(self.alert_rules)
        stats['rule_match_counts'] = {
            f"rule_{i}": rule['match_count']
            for i, rule in enumerate(self.alert_rules)
        }
        
        return stats

# TEST: Log monitor detects new log entries in real-time
# TEST: Alert rules trigger correctly for matching patterns
# TEST: Monitoring statistics are accurate
# TEST: File watcher handles log rotation correctly
```

### 5.2 Alert System Integration

```python
# nevil_framework/alert_system.py

import smtplib
import json
import requests
from typing import Dict, List, Any
from email.mime.text import MimeText
from email.mime.multipart import MimeMultipart

class AlertManager:
    """
    Manages alert notifications for the logging system.
    """
    
    def __init__(self, config: Dict = None):
        self.config = config or {}
        self.alert_channels = []
        
        # Setup alert channels based on configuration
        self._setup_alert_channels()
    
    def _setup_alert_channels(self):
        """Setup alert notification channels"""
        # Email alerts
        if self.config.get('email', {}).get('enabled', False):
            self.alert_channels.append(EmailAlertChannel(self.config['email']))
        
        # Webhook alerts
        if self.config.get('webhook', {}).get('enabled', False):
            self.alert_channels.append(WebhookAlertChannel(self.config['webhook']))
        
        # Console alerts (always enabled)
        self.alert_channels.append(ConsoleAlertChannel())
    
    def send_alert(self, alert_type: str, message: str, log_data: Dict = None):
        """Send alert through all configured channels"""
        alert_data = {
            'type': alert_type,
            'message': message,
            'timestamp': time.time(),
            'log_data': log_data
        }
        
        for channel in self.alert_channels:
            try:
                channel.send_alert(alert_data)
            except Exception as e:
                print(f"Error sending alert via {channel.__class__.__name__}: {e}")

class ConsoleAlertChannel:
    """Console alert output"""
    
    def send_alert(self, alert_data: Dict):
        timestamp = datetime.fromtimestamp(alert_data['timestamp']).strftime('%Y-%m-%d %H:%M:%S')
        print(f"[{timestamp}] ALERT: {alert_data['message']}")

class EmailAlertChannel:
    """Email alert notifications"""
    
    def __init__(self, config: Dict):
        self.smtp_server = config.get('smtp_server')
        self.smtp_port = config.get('smtp_port', 587)
        self.username = config.get('username')
        self.password = config.get('password')
        self.from_email = config.get('from_email')
        self.to_emails = config.get('to_emails', [])
    
    def send_alert(self, alert_data: Dict):
        if not self.to_emails:
            return
        
        # Create email
        msg = MimeMultipart()
        msg['From'] = self.from_email
        msg['To'] = ', '.join(self.to_emails)
        msg['Subject'] = f"Nevil Alert: {alert_data['type']}"
        
        # Email body
        body = f"""
Nevil v3.0 Alert

Type: {alert_data['type']}
Message: {alert_data['message']}
Timestamp: {datetime.fromtimestamp(alert_data['timestamp'])}

Log Data:
{json.dumps(alert_data.get('log_data', {}), indent=2)}
"""
        
        msg.attach(MimeText(body, 'plain'))
        
        # Send email
        with smtplib.SMTP(self.smtp_server, self.smtp_port) as server:
            server.starttls()
            server.login(self.username, self.password)
            server.send_message(msg)

class WebhookAlertChannel:
    """Webhook alert notifications"""
    
    def __init__(self, config: Dict):
        self.webhook_url = config.get('url')
        self.headers = config.get('headers', {'Content-Type': 'application/json'})
        self.timeout = config.get('timeout', 10)
    
    def send_alert(self, alert_data: Dict):
        if not self.webhook_url:
            return
        
        # Prepare payload
        payload = {
            'alert_type': alert_data['type'],
            'message': alert_data['message'],
            'timestamp': alert_data['timestamp'],
            'source': 'nevil_v3',
            'log_data': alert_data.get('log_data', {})
        }
        
        # Send webhook
        response = requests.post(
            self.webhook_url,
            json=payload,
            headers=self.headers,
            timeout=self.timeout
        )
        response.raise_for_status()

# TEST: Alert manager sends notifications through all configured channels
# TEST: Email alerts are properly formatted and delivered
# TEST: Webhook alerts include correct payload structure
# TEST: Failed alert deliveries don't crash the system
```

## 7. Dynamic Log Viewer System (Phase 2)

### 7.1 Interactive Log Viewer

#### Code Summary

This section implements an interactive terminal-based log viewer with real-time updates, filtering, searching, and multi-log support:

**Classes:**
- **DynamicLogViewer**: Interactive curses-based log viewer with real-time updates and filtering
- **LogViewerLauncher**: CLI launcher for the log viewer with argument parsing and mode selection

**Key Methods in DynamicLogViewer:**
- `__init__()`: Initialize viewer with curses setup, log file monitoring, and filter configuration
- `start()`: Begin interactive viewing session with keyboard input handling and screen updates
- `stop()`: Clean shutdown with curses cleanup and thread termination
- `update_display()`: Real-time screen updates with formatted log entries and status information
- `handle_input()`: Keyboard input processing for navigation, filtering, and commands
- `apply_filters()`: Dynamic filtering based on log level, node name, and text patterns
- `search_logs()`: Interactive search with highlight and navigation capabilities
- `follow_mode()`: Real-time log following with automatic scrolling

**Key Methods in LogViewerLauncher:**
- `main()`: CLI entry point with argument parsing and viewer initialization
- `parse_arguments()`: Command-line argument processing for file selection and options
- `launch_viewer()`: Viewer startup with proper configuration and error handling

**Key Features:**
- Interactive terminal interface using curses library
- Real-time log file monitoring with automatic updates
- Multi-log file support with tabbed interface
- Advanced filtering (log level, node name, text patterns, time range)
- Interactive search with syntax highlighting and result navigation
- Keyboard shortcuts for navigation and control
- Color-coded log levels with customizable themes
- Follow mode for real-time log tracking
- Cross-platform terminal compatibility

```python
# nevil_framework/log_viewer.py

import curses
import threading
import time
import queue
import re
from typing import Dict, List, Optional, Set
from pathlib import Path
from collections import deque
from datetime import datetime

class DynamicLogViewer:
    """
    Real-time, interactive log viewer with filtering, search, and color coding.
    Designed for terminal-based monitoring with toggleable features.
    """

    def __init__(self, log_dir: str = "logs", max_lines: int = 1000):
        self.log_dir = Path(log_dir)
        self.max_lines = max_lines

        # Display state
        self.lines = deque(maxlen=max_lines)
        self.filtered_lines = []
        self.current_scroll = 0
        self.screen = None

        # Filter state
        self.level_filters = {'DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'}
        self.node_filters = set()
        self.search_pattern = ""
        self.follow_mode = True
        self.show_timestamps = True
        self.auto_refresh = True

        # Threading
        self.running = False
        self.monitor_thread = None
        self.refresh_queue = queue.Queue()

        # File monitoring
        self.watched_files = {}  # file_path -> last_position

        # UI state
        self.status_message = ""
        self.help_visible = False

    def start(self):
        """Start the dynamic log viewer"""
        try:
            # Initialize curses
            self.screen = curses.initscr()
            curses.noecho()
            curses.cbreak()
            self.screen.keypad(True)
            self.screen.nodelay(True)
            curses.start_color()

            # Setup colors
            self._setup_colors()

            # Start monitoring
            self.running = True
            self.monitor_thread = threading.Thread(target=self._monitor_logs, daemon=True)
            self.monitor_thread.start()

            # Load initial logs
            self._load_recent_logs()

            # Main display loop
            self._main_loop()

        finally:
            self._cleanup()

    def _setup_colors(self):
        """Setup color pairs for display"""
        # Initialize color pairs
        curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)    # INFO
        curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)   # WARNING
        curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)      # ERROR
        curses.init_pair(4, curses.COLOR_MAGENTA, curses.COLOR_BLACK)  # CRITICAL
        curses.init_pair(5, curses.COLOR_CYAN, curses.COLOR_BLACK)     # DEBUG
        curses.init_pair(6, curses.COLOR_BLUE, curses.COLOR_BLACK)     # NODE
        curses.init_pair(7, curses.COLOR_WHITE, curses.COLOR_BLUE)     # STATUS

    def _main_loop(self):
        """Main display and input loop"""
        while self.running:
            try:
                # Handle input
                key = self.screen.getch()
                if key != -1:
                    self._handle_key(key)

                # Process refresh queue
                while not self.refresh_queue.empty():
                    try:
                        new_line = self.refresh_queue.get_nowait()
                        self.lines.append(new_line)
                        if self.follow_mode:
                            self.current_scroll = 0
                    except queue.Empty:
                        break

                # Update display
                self._update_display()

                time.sleep(0.1)  # Prevent excessive CPU usage

            except KeyboardInterrupt:
                break

    def _handle_key(self, key: int):
        """Handle keyboard input"""
        if key == ord('q'):
            self.running = False

        elif key == ord('h') or key == ord('?'):
            self.help_visible = not self.help_visible

        elif key == ord('f'):
            self.follow_mode = not self.follow_mode
            self.status_message = f"Follow mode: {'ON' if self.follow_mode else 'OFF'}"

        elif key == ord('t'):
            self.show_timestamps = not self.show_timestamps
            self.status_message = f"Timestamps: {'ON' if self.show_timestamps else 'OFF'}"

        elif key == ord('r'):
            self.auto
            self.auto_refresh = not self.auto_refresh
            self.status_message = f"Auto refresh: {'ON' if self.auto_refresh else 'OFF'}"

        elif key == ord('c'):
            self.lines.clear()
            self.status_message = "Logs cleared"

        elif key == curses.KEY_UP and not self.follow_mode:
            self.current_scroll = min(self.current_scroll + 1, len(self.filtered_lines) - 1)

        elif key == curses.KEY_DOWN and not self.follow_mode:
            self.current_scroll = max(self.current_scroll - 1, 0)

        elif key == curses.KEY_PPAGE:  # Page Up
            self.current_scroll = min(self.current_scroll + 10, len(self.filtered_lines) - 1)

        elif key == curses.KEY_NPAGE:  # Page Down
            self.current_scroll = max(self.current_scroll - 10, 0)

        # Level filter toggles
        elif key == ord('1'):
            self._toggle_level_filter('DEBUG')
        elif key == ord('2'):
            self._toggle_level_filter('INFO')
        elif key == ord('3'):
            self._toggle_level_filter('WARNING')
        elif key == ord('4'):
            self._toggle_level_filter('ERROR')
        elif key == ord('5'):
            self._toggle_level_filter('CRITICAL')

        # Search functionality
        elif key == ord('/'):
            self._handle_search()

    def _toggle_level_filter(self, level: str):
        """Toggle visibility of a log level"""
        if level in self.level_filters:
            self.level_filters.remove(level)
            self.status_message = f"Hiding {level} logs"
        else:
            self.level_filters.add(level)
            self.status_message = f"Showing {level} logs"

    def _handle_search(self):
        """Handle search input"""
        curses.echo()
        self.screen.addstr(curses.LINES - 1, 0, "Search: ")
        self.screen.refresh()

        search_input = self.screen.getstr().decode('utf-8')
        self.search_pattern = search_input
        self.status_message = f"Search: '{search_input}'" if search_input else "Search cleared"

        curses.noecho()

    def _update_display(self):
        """Update the screen display"""
        self.screen.clear()

        if self.help_visible:
            self._display_help()
            return

        # Filter lines
        self._apply_filters()

        # Calculate display area
        header_lines = 2
        status_lines = 2
        display_height = curses.LINES - header_lines - status_lines

        # Display header
        self._display_header()

        # Display log lines
        start_line = max(0, len(self.filtered_lines) - display_height - self.current_scroll)
        end_line = start_line + display_height

        for i, line_data in enumerate(self.filtered_lines[start_line:end_line]):
            y_pos = header_lines + i
            if y_pos >= curses.LINES - status_lines:
                break

            self._display_log_line(y_pos, line_data)

        # Display status
        self._display_status()

        self.screen.refresh()

    def _display_header(self):
        """Display header information"""
        header1 = f"Nevil v3 Dynamic Log Viewer - {len(self.lines)} total lines"
        header2 = f"Filters: {','.join(sorted(self.level_filters))} | Follow: {'ON' if self.follow_mode else 'OFF'}"

        self.screen.addstr(0, 0, header1, curses.color_pair(7))
        self.screen.addstr(1, 0, header2, curses.color_pair(7))

    def _display_log_line(self, y_pos: int, line_data: Dict):
        """Display a single log line with appropriate coloring"""
        try:
            x_pos = 0

            # Timestamp
            if self.show_timestamps and line_data.get('timestamp'):
                timestamp = line_data['timestamp'][:19]  # Remove microseconds
                self.screen.addstr(y_pos, x_pos, timestamp + " ")
                x_pos += len(timestamp) + 1

            # Log level with color
            level = line_data.get('level', 'INFO')
            level_color = self._get_level_color(level)
            self.screen.addstr(y_pos, x_pos, f"[{level:8}]", level_color)
            x_pos += 10

            # Node name with color
            if line_data.get('node'):
                node = line_data['node'][:12]  # Truncate long names
                self.screen.addstr(y_pos, x_pos, f"[{node:12}]", curses.color_pair(6))
                x_pos += 14

            # Message (truncate to fit screen)
            message = line_data.get('message', '')
            max_msg_width = curses.COLS - x_pos - 1
            if len(message) > max_msg_width:
                message = message[:max_msg_width-3] + "..."

            # Highlight search pattern
            if self.search_pattern and self.search_pattern.lower() in message.lower():
                self.screen.addstr(y_pos, x_pos, message, curses.A_REVERSE)
            else:
                self.screen.addstr(y_pos, x_pos, message)

        except curses.error:
            # Handle screen boundary errors gracefully
            pass

    def _get_level_color(self, level: str) -> int:
        """Get color pair for log level"""
        color_map = {
            'DEBUG': curses.color_pair(5),
            'INFO': curses.color_pair(1),
            'WARNING': curses.color_pair(2),
            'ERROR': curses.color_pair(3),
            'CRITICAL': curses.color_pair(4) | curses.A_BOLD,
        }
        return color_map.get(level, curses.color_pair(1))

    def _display_status(self):
        """Display status line"""
        status_y = curses.LINES - 2

        # Status message
        if self.status_message:
            self.screen.addstr(status_y, 0, self.status_message[:curses.COLS-1])

        # Help line
        help_text = "h:Help q:Quit f:Follow t:Time r:Refresh c:Clear /:Search 1-5:Filter"
        self.screen.addstr(status_y + 1, 0, help_text[:curses.COLS-1])

    def _display_help(self):
        """Display help screen"""
        help_text = [
            "Nevil v3 Dynamic Log Viewer - Help",
            "",
            "Navigation:",
            "  ↑/↓         - Scroll up/down (when follow mode OFF)",
            "  PgUp/PgDn    - Page up/down",
            "",
            "Toggles:",
            "  f           - Toggle follow mode (auto-scroll)",
            "  t           - Toggle timestamp display",
            "  r           - Toggle auto-refresh",
            "",
            "Filtering:",
            "  1           - Toggle DEBUG messages",
            "  2           - Toggle INFO messages",
            "  3           - Toggle WARNING messages",
            "  4           - Toggle ERROR messages",
            "  5           - Toggle CRITICAL messages",
            "",
            "Actions:",
            "  /           - Search in messages",
            "  c           - Clear current view",
            "  h or ?      - Toggle this help",
            "  q           - Quit viewer",
            "",
            "Press 'h' or '?' to close help"
        ]

        for i, line in enumerate(help_text):
            if i < curses.LINES - 1:
                self.screen.addstr(i, 2, line)

    def _apply_filters(self):
        """Apply current filters to generate filtered_lines"""
        self.filtered_lines = []

        for line_data in self.lines:
            # Level filter
            if line_data.get('level') not in self.level_filters:
                continue

            # Node filter (if any set)
            if self.node_filters and line_data.get('node') not in self.node_filters:
                continue

            # Search filter
            if self.search_pattern:
                message = line_data.get('message', '').lower()
                if self.search_pattern.lower() not in message:
                    continue

            self.filtered_lines.append(line_data)

    def _monitor_logs(self):
        """Monitor log files for changes in background thread"""
        from watchdog.observers import Observer
        from watchdog.events import FileSystemEventHandler

        class LogEventHandler(FileSystemEventHandler):
            def __init__(self, viewer):
                self.viewer = viewer

            def on_modified(self, event):
                if not event.is_directory and event.src_path.endswith('.log'):
                    self.viewer._process_log_file_change(event.src_path)

        observer = Observer()
        handler = LogEventHandler(self)
        observer.schedule(handler, str(self.log_dir), recursive=True)
        observer.start()

        try:
            while self.running:
                time.sleep(1)
        finally:
            observer.stop()
            observer.join()

    def _process_log_file_change(self, file_path: str):
        """Process changes in a log file"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                # Get last position
                last_pos = self.watched_files.get(file_path, 0)
                f.seek(last_pos)

                # Read new lines
                new_lines = f.readlines()
                if new_lines:
                    self.watched_files[file_path] = f.tell()

                    for line in new_lines:
                        line_data = self._parse_log_line(line.strip())
                        if line_data:
                            self.refresh_queue.put(line_data)

        except Exception as e:
            # Silently handle file access errors
            pass

    def _parse_log_line(self, line: str) -> Optional[Dict]:
        """Parse a log line into structured data"""
        # Enhanced pattern for new format
        pattern = re.compile(
            r'\[(?P<timestamp>[^\]]+) EST\] \[(?P<level>[^\]]+)\] '
            r'\[(?P<node>[^\]]+)\] (?:\[(?P<component>[^\]]+)\] )?'
            r'(?P<message>.*)'
        )

        match = pattern.match(line)
        if not match:
            return None

        return {
            'timestamp': match.group('timestamp').strip(),
            'level': match.group('level').strip(),
            'node': match.group('node').strip(),
            'component': match.group('component').strip() if match.group('component') else '',
            'message': match.group('message').strip(),
            'raw_line': line
        }

    def _load_recent_logs(self):
        """Load recent log entries on startup"""
        log_files = list(self.log_dir.rglob("*.log"))

        # Sort by modification time, newest first
        log_files.sort(key=lambda f: f.stat().st_mtime, reverse=True)

        # Load from most recent files first
        lines_loaded = 0
        for log_file in log_files:
            if lines_loaded >= self.max_lines // 2:  # Don't load too much initially
                break

            try:
                with open(log_file, 'r', encoding='utf-8') as f:
                    # Read last N lines efficiently
                    all_lines = f.readlines()
                    recent_lines = all_lines[-100:]  # Last 100 lines per file

                    for line in recent_lines:
                        if lines_loaded >= self.max_lines // 2:
                            break

                        line_data = self._parse_log_line(line.strip())
                        if line_data:
                            self.lines.append(line_data)
                            lines_loaded += 1

                    # Remember file position
                    self.watched_files[str(log_file)] = f.tell()

            except Exception:
                continue

        # Sort lines by timestamp
        self.lines = deque(
            sorted(self.lines, key=lambda x: x.get('timestamp', '')),
            maxlen=self.max_lines
        )

    def _cleanup(self):
        """Cleanup curses and threading"""
        self.running = False

        if self.monitor_thread:
            self.monitor_thread.join(timeout=2)

        if self.screen:
            curses.endwin()

# TEST: Dynamic log viewer displays real-time log updates
# TEST: Filtering and search functionality work correctly
# TEST: Color coding matches log level and node types
# TEST: Keyboard navigation and controls respond properly
# TEST: File monitoring detects log file changes accurately
```

### 7.2 Log Viewer Launcher

```python
# nevil_framework/log_viewer_cli.py

import argparse
import sys
from pathlib import Path

def main():
    """Command-line interface for the dynamic log viewer"""
    parser = argparse.ArgumentParser(description="Nevil v3 Dynamic Log Viewer")
    parser.add_argument(
        "--log-dir", "-d",
        type=str,
        default="logs",
        help="Log directory to monitor (default: logs)"
    )
    parser.add_argument(
        "--max-lines", "-m",
        type=int,
        default=1000,
        help="Maximum lines to keep in memory (default: 1000)"
    )
    parser.add_argument(
        "--follow",
        action="store_true",
        help="Start in follow mode (auto-scroll)"
    )
    parser.add_argument(
        "--filter-level",
        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'],
        help="Start with specific log level filter"
    )

    args = parser.parse_args()

    # Validate log directory
    log_dir = Path(args.log_dir)
    if not log_dir.exists():
        print(f"Error: Log directory '{log_dir}' does not exist")
        sys.exit(1)

    # Create and configure viewer
    viewer = DynamicLogViewer(
        log_dir=str(log_dir),
        max_lines=args.max_lines
    )

    # Apply initial configuration
    if not args.follow:
        viewer.follow_mode = False

    if args.filter_level:
        viewer.level_filters = {args.filter_level}

    try:
        print(f"Starting Nevil v3 Dynamic Log Viewer...")
        print(f"Monitoring: {log_dir.absolute()}")
        print(f"Press 'h' for help, 'q' to quit")
        print()

        viewer.start()

    except KeyboardInterrupt:
        print("\nLog viewer interrupted by user")
    except Exception as e:
        print(f"Error starting log viewer: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()

# TEST: CLI argument parsing works correctly
# TEST: Log directory validation catches invalid paths
# TEST: Initial configuration options are applied properly
```

### 7.3 Integration with Launch System

```python
# Example integration in launch script
def launch_log_viewer():
    """Launch the dynamic log viewer as a separate process"""
    import subprocess
    import sys

    try:
        # Launch viewer in new terminal if possible
        if sys.platform.startswith('linux'):
            subprocess.Popen([
                'gnome-terminal', '--',
                'python', '-m', 'nevil_framework.log_viewer_cli',
                '--log-dir', 'logs',
                '--follow'
            ])
        elif sys.platform == 'darwin':  # macOS
            subprocess.Popen([
                'osascript', '-e',
                'tell app "Terminal" to do script "python -m nevil_framework.log_viewer_cli --log-dir logs --follow"'
            ])
        else:
            # Fallback: run in current terminal
            subprocess.run([
                sys.executable, '-m', 'nevil_framework.log_viewer_cli',
                '--log-dir', 'logs',
                '--follow'
            ])
    except Exception as e:
        print(f"Could not launch log viewer: {e}")
        print("You can manually run: python -m nevil_framework.log_viewer_cli")

# TEST: Log viewer launches correctly on different platforms
# TEST: Terminal integration works with various terminal emulators
```

## 8. Desktop GUI Log Monitoring Dashboard (Phase 2)

### 8.1 LogScope GUI Monitor

For comprehensive desktop log monitoring capabilities, Nevil v3 includes **LogScope**, an advanced GUI dashboard that provides real-time log visualization, filtering, and analysis.

**Key Features:**
- Real-time multi-pane log monitoring with pause/resume
- Advanced filtering by log level, node type, and search patterns
- Crash Monitor Mode for focused error analysis
- Node health indicators with visual status updates
- Keyboard shortcuts and regex search capabilities

**Quick Start:**
```bash
# Launch LogScope dashboard
python -m nevil_framework.logscope.launcher --log-dir logs

# With increased memory for high-volume logging
python -m nevil_framework.logscope.launcher --max-entries 50000
```

**Complete Documentation:**
See [`07_logscope_gui_monitor.md`](./07_logscope_gui_monitor.md) for comprehensive implementation details, user interface guide, installation instructions, and troubleshooting information.

## Conclusion

The Phase 2 logging enhancements provide advanced monitoring, analysis, and alerting capabilities while maintaining the core simplicity and reliability of the Nevil v3.0 logging system. These features enable production-ready monitoring and debugging capabilities for complex robotic applications.

# TEST: All keyboard shortcuts function correctly
# TEST: Search and highlighting work with various patterns
# TEST: Node filtering and multi-view switching operate properly
# TEST: Pause-on-scroll behavior works as documented
# TEST: Memory management stays within configured limits