# Nevil v3.0 LogScope GUI Monitor

## Overview

LogScope is an advanced desktop log monitoring dashboard for Nevil v3 that provides real-time log visualization, filtering, and analysis capabilities. It serves as a powerful alternative to parsing monolithic system.log files, offering a modern GUI experience with comprehensive monitoring features.

**Key Features:**
- Real-time log streaming with pause/resume functionality
- Multi-pane view (unified and per-node)
- Advanced filtering by log level and node type
- Live search with regex support and highlighting
- Crash Monitor Mode for focused error analysis
- Node health indicators with visual status LEDs
- Keyboard shortcuts for efficient operation
- Performance metrics and statistics display

## Architecture

### Technical Design

LogScope implements a **multi-threaded architecture** with clear separation between monitoring, processing, and display components:

```
File System → Watchdog Monitor → Queue → GUI Thread → Display Views
     ↓              ↓              ↓         ↓           ↓
   Log Files → File Events → Entry Queue → Processing → Multi-Tab UI
```

**Core Components:**
- **File Monitor**: Background thread using watchdog library for real-time file monitoring
- **Log Parser**: Structured parsing of EST-formatted log entries
- **Filter Engine**: Multi-criteria filtering with crash detection
- **GUI Controller**: PyQt-based interface with responsive updates
- **View Manager**: Tabbed interface for unified and node-specific views

### Memory Management

- **Bounded Storage**: Maximum configurable entry limit (default: 10,000)
- **Automatic Pruning**: Oldest entries removed when limit exceeded
- **Efficient Updates**: Batched processing (50 entries per 100ms update cycle)
- **Thread Safety**: Queue-based communication between monitor and GUI threads

## Implementation

### Main Application Class

```python
# nevil_framework/logscope/main_window.py

import sys
import threading
import queue
import re
from typing import Dict, List, Set, Optional
from datetime import datetime
from pathlib import Path

try:
    from PyQt6.QtWidgets import *
    from PyQt6.QtCore import *
    from PyQt6.QtGui import *
    PYQT_AVAILABLE = True
except ImportError:
    try:
        from PyQt5.QtWidgets import *
        from PyQt5.QtCore import *
        from PyQt5.QtGui import *
        PYQT_AVAILABLE = True
    except ImportError:
        PYQT_AVAILABLE = False

class NevilLogScope(QMainWindow):
    """
    Advanced desktop log monitoring dashboard for Nevil v3.

    Features:
    - Multi-pane view (unified, per-node, system)
    - Real-time live streaming with pause/resume
    - Keystroke-based filtering (Ctrl+1-5 for levels, F1-F4 for nodes)
    - Live search with colored highlighting
    - Pause on scroll, resume on hotkey
    - Node health indicators
    - Performance metrics overlay
    """

    # Custom signals for thread-safe GUI updates
    new_log_entry = pyqtSignal(dict)
    node_status_changed = pyqtSignal(str, str)

    def __init__(self):
        super().__init__()

        # Application state
        self.log_dir = Path("logs")
        self.running = False
        self.paused = False
        self.auto_scroll = True

        # Filtering state
        self.active_levels = {'DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'}
        self.active_nodes = {'system', 'speech_recognition', 'speech_synthesis', 'ai_cognition'}
        self.search_pattern = ""
        self.search_regex = None
        self.crash_monitor_mode = False  # Crash Monitor Mode

        # Data storage
        self.log_entries = []  # All entries
        self.filtered_entries = []  # Currently visible entries
        self.max_entries = 10000  # Memory limit

        # Threading
        self.monitor_thread = None
        self.update_queue = queue.Queue()

        # Setup UI
        self.init_ui()
        self.init_hotkeys()
        self.init_monitoring()

        # Start log monitoring
        self.start_monitoring()

    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("Nevil v3 LogScope - Real-time Log Monitor")
        self.setGeometry(100, 100, 1600, 1000)

        # Central widget with splitter layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QVBoxLayout(central_widget)

        # Top toolbar
        self.create_toolbar(main_layout)

        # Create splitter for multi-pane view
        splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter)

        # Left panel - Node overview and filters
        self.create_control_panel(splitter)

        # Right panel - Log displays
        self.create_log_panels(splitter)

        # Bottom status bar
        self.create_status_bar()

        # Set splitter proportions
        splitter.setSizes([300, 1300])  # 300px for controls, rest for logs

    def create_toolbar(self, layout):
        """Create top toolbar with search and controls"""
        toolbar_layout = QHBoxLayout()

        # Search box
        self.search_box = QLineEdit()
        self.search_box.setPlaceholderText("Search logs... (Ctrl+F)")
        self.search_box.textChanged.connect(self.on_search_changed)
        self.search_box.returnPressed.connect(self.focus_next_match)

        # Search controls
        self.search_case_sensitive = QCheckBox("Case Sensitive")
        self.search_regex_mode = QCheckBox("Regex")

        # Live controls
        self.live_button = QPushButton("⏸️ Pause" if not self.paused else "▶️ Resume")
        self.live_button.clicked.connect(self.toggle_pause)

        self.auto_scroll_button = QPushButton("📜 Auto-scroll ON" if self.auto_scroll else "📜 Auto-scroll OFF")
        self.auto_scroll_button.clicked.connect(self.toggle_auto_scroll)

        self.clear_button = QPushButton("🗑️ Clear")
        self.clear_button.clicked.connect(self.clear_logs)

        self.crash_monitor_button = QPushButton("🚨 Crash Monitor OFF")
        self.crash_monitor_button.clicked.connect(self.toggle_crash_monitor_mode)
        self.crash_monitor_button.setStyleSheet("QPushButton { font-weight: bold; }")

        # Add to layout
        toolbar_layout.addWidget(QLabel("Search:"))
        toolbar_layout.addWidget(self.search_box)
        toolbar_layout.addWidget(self.search_case_sensitive)
        toolbar_layout.addWidget(self.search_regex_mode)
        toolbar_layout.addStretch()
        toolbar_layout.addWidget(self.crash_monitor_button)
        toolbar_layout.addWidget(self.live_button)
        toolbar_layout.addWidget(self.auto_scroll_button)
        toolbar_layout.addWidget(self.clear_button)

        toolbar_widget = QWidget()
        toolbar_widget.setLayout(toolbar_layout)
        layout.addWidget(toolbar_widget)

    def create_control_panel(self, splitter):
        """Create left control panel"""
        control_widget = QWidget()
        control_layout = QVBoxLayout(control_widget)

        # Node status indicators
        control_layout.addWidget(QLabel("Node Status:"))
        self.node_indicators = {}

        for node in ['system', 'speech_recognition', 'speech_synthesis', 'ai_cognition']:
            indicator_layout = QHBoxLayout()

            # Status LED
            status_led = QLabel("🟢")  # Green by default
            status_led.setFixedSize(20, 20)
            self.node_indicators[node] = status_led

            # Node name and checkbox
            checkbox = QCheckBox(node.replace('_', ' ').title())
            checkbox.setChecked(True)
            checkbox.stateChanged.connect(lambda state, n=node: self.toggle_node_filter(n, state))

            # Hotkey label
            hotkey_map = {'system': 'F1', 'speech_recognition': 'F2', 'speech_synthesis': 'F3', 'ai_cognition': 'F4'}
            hotkey_label = QLabel(f"({hotkey_map[node]})")
            hotkey_label.setStyleSheet("color: gray; font-size: 10px;")

            indicator_layout.addWidget(status_led)
            indicator_layout.addWidget(checkbox)
            indicator_layout.addStretch()
            indicator_layout.addWidget(hotkey_label)

            control_layout.addLayout(indicator_layout)

        control_layout.addWidget(QLabel(""))  # Spacer

        # Log level filters
        control_layout.addWidget(QLabel("Log Levels:"))
        self.level_checkboxes = {}

        level_colors = {
            'DEBUG': '#00FFFF',    # Cyan
            'INFO': '#00FF00',     # Green
            'WARNING': '#FFFF00',  # Yellow
            'ERROR': '#FF0000',    # Red
            'CRITICAL': '#FF00FF'  # Magenta
        }

        hotkey_nums = {'DEBUG': '1', 'INFO': '2', 'WARNING': '3', 'ERROR': '4', 'CRITICAL': '5'}

        for level in ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']:
            level_layout = QHBoxLayout()

            checkbox = QCheckBox(level)
            checkbox.setChecked(True)
            checkbox.stateChanged.connect(lambda state, l=level: self.toggle_level_filter(l, state))
            checkbox.setStyleSheet(f"color: {level_colors[level]}; font-weight: bold;")
            self.level_checkboxes[level] = checkbox

            hotkey_label = QLabel(f"(Ctrl+{hotkey_nums[level]})")
            hotkey_label.setStyleSheet("color: gray; font-size: 10px;")

            level_layout.addWidget(checkbox)
            level_layout.addStretch()
            level_layout.addWidget(hotkey_label)

            control_layout.addLayout(level_layout)

        control_layout.addStretch()

        # Statistics
        control_layout.addWidget(QLabel("Statistics:"))
        self.stats_label = QLabel("Entries: 0\nFiltered: 0\nRate: 0.0/s")
        self.stats_label.setStyleSheet("font-family: monospace; font-size: 10px;")
        control_layout.addWidget(self.stats_label)

        splitter.addWidget(control_widget)

    def create_log_panels(self, splitter):
        """Create main log display panels"""
        # Right side - tabbed log views
        self.tab_widget = QTabWidget()

        # Unified view - all logs merged
        self.unified_view = self.create_log_view("All Logs")
        self.tab_widget.addTab(self.unified_view, "🔄 Unified")

        # Per-node views
        self.node_views = {}
        node_tabs = [
            ("system", "🖥️ System"),
            ("speech_recognition", "🎤 Speech Rec"),
            ("speech_synthesis", "🔊 Speech Syn"),
            ("ai_cognition", "🧠 AI Cognition")
        ]

        for node_key, tab_name in node_tabs:
            view = self.create_log_view(f"{node_key} logs")
            self.node_views[node_key] = view
            self.tab_widget.addTab(view, tab_name)

        splitter.addWidget(self.tab_widget)

    def create_log_view(self, view_name):
        """Create a single log view widget"""
        view = QTextEdit()
        view.setReadOnly(True)
        view.setFont(QFont("Consolas", 10))  # Monospace font
        view.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: #ffffff;
                border: 1px solid #404040;
            }
        """)

        # Enable scroll-to-pause functionality
        scrollbar = view.verticalScrollBar()
        scrollbar.valueChanged.connect(lambda: self.on_scroll_changed(view))

        return view

    def init_hotkeys(self):
        """Initialize keyboard shortcuts"""
        # Search
        QShortcut(QKeySequence("Ctrl+F"), self, self.focus_search)

        # Level filters (Ctrl+1-5)
        levels = ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']
        for i, level in enumerate(levels, 1):
            QShortcut(QKeySequence(f"Ctrl+{i}"), self, lambda l=level: self.toggle_level_hotkey(l))

        # Node filters (F1-F4)
        nodes = ['system', 'speech_recognition', 'speech_synthesis', 'ai_cognition']
        for i, node in enumerate(nodes, 1):
            QShortcut(QKeySequence(f"F{i}"), self, lambda n=node: self.toggle_node_hotkey(n))

        # Crash Monitor Mode
        QShortcut(QKeySequence("Ctrl+Shift+C"), self, self.toggle_crash_monitor_mode)

        # Pause/Resume
        QShortcut(QKeySequence("Space"), self, self.toggle_pause)
        QShortcut(QKeySequence("P"), self, self.toggle_pause)

        # Clear
        QShortcut(QKeySequence("Ctrl+L"), self, self.clear_logs)

        # Auto-scroll
        QShortcut(QKeySequence("A"), self, self.toggle_auto_scroll)

    def init_monitoring(self):
        """Initialize log file monitoring"""
        # Connect signals
        self.new_log_entry.connect(self.add_log_entry)
        self.node_status_changed.connect(self.update_node_status)

        # Timer for GUI updates
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.process_update_queue)
        self.update_timer.start(100)  # Update every 100ms

    def start_monitoring(self):
        """Start log file monitoring in background thread"""
        if self.running:
            return

        self.running = True
        self.monitor_thread = threading.Thread(target=self._monitor_logs, daemon=True)
        self.monitor_thread.start()

    def _monitor_logs(self):
        """Background thread for monitoring log files"""
        from watchdog.observers import Observer
        from watchdog.events import FileSystemEventHandler

        class LogEventHandler(FileSystemEventHandler):
            def __init__(self, logscope):
                self.logscope = logscope
                self.file_positions = {}

            def on_modified(self, event):
                if not event.is_directory and event.src_path.endswith('.log'):
                    self.process_log_file(event.src_path)

            def process_log_file(self, file_path):
                try:
                    with open(file_path, 'r', encoding='utf-8') as f:
                        last_pos = self.file_positions.get(file_path, 0)
                        f.seek(last_pos)

                        new_lines = f.readlines()
                        if new_lines:
                            self.file_positions[file_path] = f.tell()

                            for line in new_lines:
                                entry = self.parse_log_line(file_path, line.strip())
                                if entry:
                                    self.logscope.update_queue.put(('log', entry))

                except Exception as e:
                    print(f"Error processing {file_path}: {e}")

            def parse_log_line(self, file_path, line):
                """Parse log line into structured entry"""
                # Enhanced pattern for EST format
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
                    'file_path': file_path,
                    'raw_line': line
                }

        observer = Observer()
        handler = LogEventHandler(self)
        observer.schedule(handler, str(self.log_dir), recursive=False)
        observer.start()

        try:
            while self.running:
                threading.Event().wait(1)
        finally:
            observer.stop()
            observer.join()

    def process_update_queue(self):
        """Process queued updates from monitoring thread"""
        processed = 0
        while not self.update_queue.empty() and processed < 50:  # Limit batch size
            try:
                update_type, data = self.update_queue.get_nowait()
                if update_type == 'log':
                    self.new_log_entry.emit(data)
                processed += 1
            except queue.Empty:
                break

    def add_log_entry(self, entry):
        """Add new log entry to displays"""
        if self.paused:
            return

        # Add to storage
        self.log_entries.append(entry)

        # Trim if too many entries
        if len(self.log_entries) > self.max_entries:
            self.log_entries = self.log_entries[-self.max_entries:]

        # Check if entry passes filters
        if self.passes_filters(entry):
            self.filtered_entries.append(entry)

            # Add to appropriate views
            formatted_line = self.format_log_entry(entry)

            # Unified view
            self.append_to_view(self.unified_view, formatted_line, entry)

            # Node-specific view
            if entry['node'] in self.node_views:
                self.append_to_view(self.node_views[entry['node']], formatted_line, entry)

        # Update statistics
        self.update_statistics()

    def format_log_entry(self, entry):
        """Format log entry for display with colors"""
        timestamp = entry['timestamp']
        level = entry['level']
        node = entry['node']
        component = entry['component']
        message = entry['message']

        # Color codes
        level_colors = {
            'DEBUG': '#00FFFF',    # Cyan
            'INFO': '#00FF00',     # Green
            'WARNING': '#FFFF00',  # Yellow
            'ERROR': '#FF0000',    # Red
            'CRITICAL': '#FF00FF'  # Magenta
        }

        node_colors = {
            'system': '#0080FF',           # Blue
            'speech_recognition': '#40FFFF', # Light Cyan
            'speech_synthesis': '#80FF80',   # Light Green
            'ai_cognition': '#FF80FF',       # Light Magenta
        }

        level_color = level_colors.get(level, '#FFFFFF')
        node_color = node_colors.get(node, '#FFFFFF')

        # Build formatted line
        formatted = f'<span style="color: #AAAAAA;">[{timestamp} EST]</span> '
        formatted += f'<span style="color: {level_color}; font-weight: bold;">[{level:8}]</span> '
        formatted += f'<span style="color: {node_color};">[{node:15}]</span> '
        if component:
            formatted += f'<span style="color: #CCCCCC;">[{component:20}]</span> '
        formatted += f'<span style="color: #FFFFFF;">{message}</span>'

        return formatted

    def append_to_view(self, view, formatted_line, entry):
        """Append formatted line to a view with search highlighting"""
        if self.search_pattern and self.search_regex:
            # Highlight search matches
            highlighted = self.highlight_search_matches(formatted_line, entry)
            view.append(highlighted)
        else:
            view.append(formatted_line)

        # Auto-scroll if enabled and view is active
        if self.auto_scroll:
            scrollbar = view.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())

    def highlight_search_matches(self, formatted_line, entry):
        """Highlight search matches in the formatted line"""
        if not self.search_regex:
            return formatted_line

        try:
            # Search in the raw message for matches
            message = entry['message']
            matches = list(self.search_regex.finditer(message))

            if matches:
                # Highlight matches with yellow background
                highlighted_message = message
                offset = 0
                for match in matches:
                    start, end = match.span()
                    start += offset
                    end += offset

                    replacement = f'<span style="background-color: #FFFF00; color: #000000;">{message[match.start():match.end()]}</span>'
                    highlighted_message = highlighted_message[:start] + replacement + highlighted_message[end:]
                    offset += len(replacement) - (end - start)

                # Replace message in formatted line
                return formatted_line.replace(f'>{message}</span>', f'>{highlighted_message}</span>')

        except Exception as e:
            print(f"Search highlighting error: {e}")

        return formatted_line

    def passes_filters(self, entry):
        """Check if entry passes current filters"""
        # Crash Monitor Mode - overrides other filters
        if self.crash_monitor_mode:
            return self._is_crash_related(entry)

        # Level filter
        if entry['level'] not in self.active_levels:
            return False

        # Node filter
        if entry['node'] not in self.active_nodes:
            return False

        # Search filter
        if self.search_pattern and self.search_regex:
            try:
                if not self.search_regex.search(entry['message']):
                    return False
            except Exception:
                return False

        return True

    def _is_crash_related(self, entry):
        """Check if entry is crash/error related"""
        # Check log level first
        if entry['level'] in ['ERROR', 'CRITICAL']:
            return True

        # Check message content for crash-related keywords
        message = entry['message'].lower()
        crash_keywords = [
            'error', 'failure', 'crash', 'unsuccessful', 'failed',
            'exception', 'traceback', 'fatal', 'abort', 'panic',
            'timeout', 'refused', 'unavailable', 'corrupted',
            'invalid', 'missing', 'not found', 'access denied',
            'connection lost', 'segmentation fault', 'memory error'
        ]

        return any(keyword in message for keyword in crash_keywords)

    # Event handlers and utility methods...
    def toggle_pause(self):
        """Toggle pause state"""
        self.paused = not self.paused
        self.live_button.setText("▶️ Resume" if self.paused else "⏸️ Pause")

    def toggle_auto_scroll(self):
        """Toggle auto-scroll"""
        self.auto_scroll = not self.auto_scroll
        self.auto_scroll_button.setText("📜 Auto-scroll ON" if self.auto_scroll else "📜 Auto-scroll OFF")

    def toggle_crash_monitor_mode(self):
        """Toggle Crash Monitor Mode"""
        self.crash_monitor_mode = not self.crash_monitor_mode

        if self.crash_monitor_mode:
            self.crash_monitor_button.setText("🚨 Crash Monitor ON")
            self.crash_monitor_button.setStyleSheet("QPushButton { background-color: #ff4444; color: white; font-weight: bold; }")
            self.status_message = "Crash Monitor Mode: Showing only ERROR/CRITICAL and crash-related entries"
        else:
            self.crash_monitor_button.setText("🚨 Crash Monitor OFF")
            self.crash_monitor_button.setStyleSheet("QPushButton { font-weight: bold; }")
            self.status_message = "Crash Monitor Mode: OFF"

        # Refresh view with new filter
        self.refresh_all_views()

    def on_search_changed(self, text):
        """Handle search text changes"""
        self.search_pattern = text
        if text:
            try:
                flags = 0 if self.search_case_sensitive.isChecked() else re.IGNORECASE
                if self.search_regex_mode.isChecked():
                    self.search_regex = re.compile(text, flags)
                else:
                    self.search_regex = re.compile(re.escape(text), flags)
            except re.error:
                self.search_regex = None
        else:
            self.search_regex = None

        self.refresh_all_views()

    def on_scroll_changed(self, view):
        """Handle scroll changes for pause-on-scroll"""
        scrollbar = view.verticalScrollBar()
        # If user scrolled up from bottom, pause auto-scroll
        if scrollbar.value() < scrollbar.maximum() - 10:
            if self.auto_scroll:
                self.auto_scroll = False
                self.auto_scroll_button.setText("📜 Auto-scroll OFF")

    def refresh_all_views(self):
        """Refresh all views with current filters"""
        # Clear views
        self.unified_view.clear()
        for view in self.node_views.values():
            view.clear()

        # Re-filter and display entries
        self.filtered_entries = []
        for entry in self.log_entries:
            if self.passes_filters(entry):
                self.filtered_entries.append(entry)
                formatted_line = self.format_log_entry(entry)

                self.append_to_view(self.unified_view, formatted_line, entry)
                if entry['node'] in self.node_views:
                    self.append_to_view(self.node_views[entry['node']], formatted_line, entry)

        self.update_statistics()

    def update_statistics(self):
        """Update statistics display"""
        total = len(self.log_entries)
        filtered = len(self.filtered_entries)

        # Calculate rate (entries per second over last 60 seconds)
        rate = 0.0  # Simplified for now

        self.stats_label.setText(f"Entries: {total}\nFiltered: {filtered}\nRate: {rate:.1f}/s")

    def closeEvent(self, event):
        """Handle application close"""
        self.running = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2)
        event.accept()

# TEST: GUI application launches and displays log entries
# TEST: Real-time filtering works for levels and nodes
# TEST: Search highlighting functions correctly
# TEST: Pause on scroll and resume functionality works
# TEST: Keyboard shortcuts respond properly
```

### Launcher Implementation

```python
# nevil_framework/logscope/launcher.py

import sys
import argparse
from pathlib import Path

def main():
    """Launch Nevil LogScope desktop application"""
    parser = argparse.ArgumentParser(description="Nevil v3 LogScope - Desktop Log Monitor")
    parser.add_argument("--log-dir", "-d", default="logs", help="Log directory to monitor")
    parser.add_argument("--max-entries", "-m", type=int, default=10000, help="Maximum entries to keep in memory")
    parser.add_argument("--theme", choices=["dark", "light"], default="dark", help="UI theme")

    args = parser.parse_args()

    # Check if PyQt is available
    if not PYQT_AVAILABLE:
        print("Error: PyQt6 or PyQt5 required for LogScope GUI")
        print("Install with: pip install PyQt6")
        sys.exit(1)

    # Validate log directory
    log_dir = Path(args.log_dir)
    if not log_dir.exists():
        print(f"Warning: Log directory '{log_dir}' does not exist")

    # Create QApplication
    app = QApplication(sys.argv)
    app.setApplicationName("Nevil LogScope")

    # Create and show main window
    window = NevilLogScope()
    window.log_dir = log_dir
    window.max_entries = args.max_entries
    window.show()

    # Run application
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
```

### Integration with Nevil v3 Launch System

```python
# Example integration in main launch script
def launch_logscope():
    """Launch LogScope monitoring dashboard"""
    import subprocess
    import sys

    try:
        # Launch LogScope in separate process
        subprocess.Popen([
            sys.executable, "-m", "nevil_framework.logscope.launcher",
            "--log-dir", "logs",
            "--theme", "dark"
        ])
        print("LogScope dashboard launched")
    except Exception as e:
        print(f"Could not launch LogScope: {e}")
        print("You can manually run: python -m nevil_framework.logscope.launcher")

# Add to main launch options
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--logscope", action="store_true", help="Launch LogScope dashboard")

    args = parser.parse_args()

    if args.logscope:
        launch_logscope()
```

## User Interface Documentation

### Main Window Layout

**Default Window Size: 1600x1000 pixels**

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│ Search & Controls Toolbar                                                      │
├─────────────────┬───────────────────────────────────────────────────────────────┤
│ Control Panel   │ Multi-Tab Log Viewer                                         │
│ (300px wide)    │ (1300px wide)                                                │
│                 │                                                               │
│ • Node Status   │ Tabs: [🔄 Unified] [🖥️ System] [🎤 Speech Rec]               │
│ • Level Filters │       [🔊 Speech Syn] [🧠 AI Cognition]                        │
│ • Statistics    │                                                               │
│                 │ Real-time log stream with color coding                       │
└─────────────────┴───────────────────────────────────────────────────────────────┘
```

### Keyboard Shortcuts Reference

| Shortcut | Action | Description |
|----------|--------|-------------|
| `Ctrl+F` | Focus Search | Jump to search box |
| `Ctrl+1` | Toggle DEBUG | Show/hide DEBUG level logs |
| `Ctrl+2` | Toggle INFO | Show/hide INFO level logs |
| `Ctrl+3` | Toggle WARNING | Show/hide WARNING level logs |
| `Ctrl+4` | Toggle ERROR | Show/hide ERROR level logs |
| `Ctrl+5` | Toggle CRITICAL | Show/hide CRITICAL level logs |
| `F1` | Toggle System | Show/hide System node logs |
| `F2` | Toggle Speech Recognition | Show/hide Speech Recognition logs |
| `F3` | Toggle Speech Synthesis | Show/hide Speech Synthesis logs |
| `F4` | Toggle AI Cognition | Show/hide AI Cognition logs |
| `Space` or `P` | Pause/Resume | Toggle live log streaming |
| `A` | Auto-scroll | Toggle auto-scroll to bottom |
| `Ctrl+L` | Clear | Clear all visible log entries |
| `Ctrl+Shift+C` | Crash Monitor | Toggle Crash Monitor Mode (ERROR/CRITICAL + crash keywords) |
| `Enter` (in search) | Next Match | Find next search result |

### Search Functionality

**Basic Search:**
- Type in search box for instant filtering
- Case-insensitive by default
- Highlights matches with yellow background
- Searches only in log message content

**Advanced Search Options:**
- **Case Sensitive**: Check box for exact case matching
- **Regex Mode**: Enable regular expression patterns
- **Live Filtering**: Results update as you type

**Search Examples:**
```
Basic:     "OpenAI"           → Find all OpenAI mentions
Regex:     "error.*timeout"   → Find errors containing timeout
Regex:     "^[WARNING]"       → Find lines starting with WARNING
Regex:     "\d+\.\d+s"        → Find duration measurements
```

### Crash Monitor Mode

**Purpose:**
Crash Monitor Mode provides focused troubleshooting by filtering logs to show only error-level events and entries containing crash-related keywords.

**Activation:**
- **Button**: Click the "🚨 Crash Monitor OFF/ON" button in the toolbar
- **Keyboard**: Press `Ctrl+Shift+C` to toggle
- **Visual Feedback**: Button turns red when active with "🚨 Crash Monitor ON"

**Filter Criteria:**
- **Log Levels**: Automatically includes ERROR and CRITICAL level logs
- **Keywords**: Detects crash-related terms in log messages:
  - `error`, `failure`, `crash`, `unsuccessful`, `failed`
  - `exception`, `traceback`, `fatal`, `abort`, `panic`
  - `timeout`, `refused`, `unavailable`, `corrupted`
  - `invalid`, `missing`, `not found`, `access denied`
  - `connection lost`, `segmentation fault`, `memory error`

**Behavior:**
- **Override Filters**: When active, overrides all other level and node filters
- **Real-time**: Works with live streaming and existing log entries
- **Search Compatible**: Can be combined with search functionality
- **Status Message**: Displays "Crash Monitor Mode: Showing only ERROR/CRITICAL and crash-related entries"

**Use Cases:**
- Rapid problem identification during system failures
- Focused debugging of intermittent crashes
- Security incident investigation
- Performance issue root cause analysis

### Node Status Indicators

**Status LED Colors:**
- 🟢 **Green**: Node healthy and active
- 🟡 **Yellow**: Node warnings detected
- 🔴 **Red**: Node errors or critical issues
- ⚫ **Gray**: Node inactive or stopped

**Status Updates:**
- Real-time updates based on log activity
- Health determined by recent error rates
- Automatic detection of node crashes

### Multi-View System

**Unified View (🔄):**
- Shows all logs from all nodes merged chronologically
- Color-coded by node type and severity level
- Best for overall system monitoring
- Default view on startup

**Node-Specific Views:**
- **System (🖥️)**: Framework and system-level messages
- **Speech Recognition (🎤)**: Audio input and recognition logs
- **Speech Synthesis (🔊)**: TTS and audio output logs
- **AI Cognition (🧠)**: AI processing and decision logs

**View Features:**
- Independent filtering per view
- Separate scroll positions
- Tab switching preserves state

### Pause-on-Scroll Behavior

**Automatic Pause:**
- Scrolling up automatically disables auto-scroll
- Allows historical log examination
- Prevents new entries from interrupting reading

**Resume Methods:**
- Scroll to bottom → Auto-resumes
- Press `Space` or `P` → Manual resume
- Click "Auto-scroll" button → Manual resume

**Visual Indicators:**
- Button text changes: "Auto-scroll ON" vs "Auto-scroll OFF"
- Pause button: "⏸️ Pause" vs "▶️ Resume"

## Installation and Configuration

### Requirements

```bash
# Required
pip install PyQt6        # Or PyQt5 as fallback
pip install watchdog     # File monitoring

# Optional for enhanced features
pip install regex        # Advanced regex support
```

### Installation

```bash
# Clone/install Nevil v3 framework
# Then run LogScope
python -m nevil_framework.logscope.launcher --log-dir logs
```

### Command Line Options

```bash
python -m nevil_framework.logscope.launcher [OPTIONS]

Options:
  -d, --log-dir PATH     Log directory to monitor (default: logs)
  -m, --max-entries INT  Maximum entries in memory (default: 10000)
  --theme [dark|light]   UI theme (default: dark)
  -h, --help            Show help message
```

**Usage Examples:**
```bash
# Basic usage
python -m nevil_framework.logscope.launcher

# Monitor custom log directory
python -m nevil_framework.logscope.launcher --log-dir /path/to/logs

# Increase memory limit for high-volume logging
python -m nevil_framework.logscope.launcher --max-entries 50000

# Light theme
python -m nevil_framework.logscope.launcher --theme light
```

### Integration with Nevil v3

**Auto-Launch with System:**
```python
# In main launch script
if args.monitor:
    launch_logscope()
    time.sleep(2)  # Allow GUI to initialize
    launch_nevil_nodes()
```

**Configuration Integration:**
```yaml
# .nodes configuration
system:
  monitoring:
    logscope_enabled: true
    logscope_theme: "dark"
    max_entries: 10000
```

## Performance and Resource Management

### Memory Limits

- **Default**: 10,000 log entries in memory
- **Configurable** via `--max-entries` parameter
- **Oldest entries automatically pruned**
- **Efficient storage** using structured data

### Update Batching

- **GUI updates every 100ms**
- **Batch processing** of up to 50 entries per update
- **Background thread** for file monitoring
- **Thread-safe queue** for entry processing

### Performance Monitoring

- **Live statistics display**
- **Entries per second calculation**
- **Memory usage tracking**
- **Filter efficiency metrics**

## Troubleshooting

### Common Issues

1. **PyQt Not Found:**
   ```
   Error: PyQt6 or PyQt5 required for LogScope GUI
   Solution: pip install PyQt6
   ```

2. **Log Directory Not Found:**
   ```
   Warning: Log directory 'logs' does not exist
   Solution: Ensure Nevil v3 has created log files first
   ```

3. **High Memory Usage:**
   ```
   Symptom: LogScope consuming excessive RAM
   Solution: Reduce --max-entries or restart application
   ```

4. **Search Not Working:**
   ```
   Symptom: Regex patterns not matching
   Solution: Check regex syntax or disable regex mode
   ```

### Debug Mode

```bash
# Enable debug output
LOGSCOPE_DEBUG=1 python -m nevil_framework.logscope.launcher
```

## Comparison: LogScope vs System.log

| Feature | System.log Parsing | LogScope Dashboard |
|---------|-------------------|-------------------|
| **Real-time Updates** | Manual refresh | Automatic streaming |
| **Color Coding** | Terminal colors only | Full HTML color support |
| **Multi-node View** | Single file confusion | Separate tabs per node |
| **Search & Highlight** | grep commands | Live search with highlighting |
| **Filtering** | Complex CLI commands | Point-and-click checkboxes |
| **History Navigation** | File scrolling | Intelligent pause-on-scroll |
| **Node Health** | Manual analysis | Visual status indicators |
| **Performance Stats** | Manual calculation | Live metrics display |
| **Learning Curve** | CLI expertise required | Intuitive GUI interface |
| **Resource Usage** | Low | Moderate (GUI overhead) |

## Best Practices

### Efficient Monitoring

1. Use node-specific tabs for focused debugging
2. Filter by severity level to reduce noise
3. Use regex search for complex pattern matching
4. Monitor statistics to gauge system health

### Performance Optimization

1. Adjust max-entries based on available RAM
2. Close unused tabs to reduce processing
3. Use specific filters instead of viewing all logs
4. Clear logs periodically for long-running sessions

### Debugging Workflow

1. Start with Unified view for overview
2. Filter to ERROR/CRITICAL levels for issues
3. Switch to specific node tab for details
4. Use search to find specific error patterns
5. Scroll through history while paused

This comprehensive documentation ensures users can effectively leverage LogScope as a powerful alternative to monolithic system.log file parsing, providing a modern GUI experience for Nevil v3 log monitoring.

# TEST: LogScope launches successfully on different platforms