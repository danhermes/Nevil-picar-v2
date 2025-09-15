
# Nevil v3.0 Logging Architecture

## Overview

The Nevil v3.0 logging system provides comprehensive, structured logging across all framework components and nodes. It supports multiple log levels, automatic rotation, and both console and file output while maintaining simplicity and performance.

## Detailed Technical Design Specification

### Logging Architecture Philosophy

The Nevil v3.0 logging system is designed around three fundamental principles:

1. **Bounded Resource Usage**: Strict limits on log file growth and storage consumption
2. **Readability-First Design**: Human-readable timestamps, color coding, and clear source identification
3. **Zero-Configuration Operation**: Intelligent defaults that work out-of-the-box with minimal setup

### Multi-Tier Logging Architecture

The logging system implements a **hierarchical logging architecture** with multiple abstraction layers that provide both flexibility and performance optimization.

#### Logging Data Flow Architecture

```
Application Layer → Log Manager → Formatter → Handler → Storage → Rotation → Cleanup
     ↓                ↓            ↓          ↓         ↓         ↓         ↓
Node Loggers → Structured → EST Format → File/Console → Disk → Numbered → Auto-Prune
```

Each layer has specific responsibilities and can be independently configured or replaced without affecting other layers.

### Structured Logging Framework

The system implements a **context-aware structured logging model** that automatically enriches log entries with relevant metadata and maintains consistency across all components.

#### Log Entry Structure Design

**Core Metadata Fields**:
- **Timestamp**: EST timezone with microsecond precision for debugging
- **Level**: Severity classification with color coding
- **Node**: Source node identification with visual differentiation
- **Component**: Sub-component or module identification
- **Thread**: Thread ID for concurrency debugging
- **Context**: Request/operation correlation IDs

**Enhanced Context Injection**:
- **Performance Metrics**: Automatic duration tracking for operations
- **Resource Usage**: Memory and CPU utilization at log time
- **Error Context**: Stack traces and error correlation
- **Business Context**: Domain-specific metadata injection

#### Intelligent Log Correlation

The system provides **automatic log correlation** across distributed components:

**Correlation Strategies**:
- **Request Tracing**: End-to-end request tracking across nodes
- **Session Correlation**: User session tracking across interactions
- **Error Propagation**: Error cause-and-effect relationship tracking
- **Performance Correlation**: Related performance events grouping

### File Rotation and Retention Architecture

The logging system implements an **aggressive retention policy** designed to prevent unbounded log growth while maintaining sufficient debugging history.

#### Strict Retention Policy Framework

**KEEP_LOG_FILES=3 Enforcement**:
- **Current File**: Active log file receiving new entries
- **Backup .1**: Previous rotation (most recent historical data)
- **Backup .2**: Oldest retained file (deleted on next rotation)
- **Total Limit**: Maximum 12 files system-wide (4 log types × 3 files)

#### Rotation Trigger Mechanisms

**Size-Based Rotation**:
- **File Size Threshold**: 20MB maximum per log file
- **Immediate Rotation**: Atomic file movement without service interruption
- **Overflow Protection**: Emergency rotation for rapid log generation

**Time-Based Rotation**:
- **Daily Rotation**: Automatic rotation at midnight EST
- **Startup Rotation**: Fresh logs for each system restart
- **Manual Rotation**: Administrative rotation capability

#### Storage Optimization Framework

**Disk Usage Prediction**:
- **Growth Rate Analysis**: Historical log growth pattern analysis
- **Predictive Rotation**: Proactive rotation based on growth trends
- **Resource Monitoring**: Available disk space monitoring and alerts

**Compression Strategy**:
- **No Compression**: Simple numbered files for fast access
- **Trade-off Analysis**: Simplicity vs. storage efficiency
- **Access Pattern Optimization**: Recent logs prioritized for speed

### Real-Time Log Processing Architecture

The system implements **stream-based log processing** that enables real-time analysis and alerting without impacting application performance.

#### Asynchronous Processing Pipeline

**Log Event Stream**:
- **Non-Blocking Writes**: Asynchronous log entry processing
- **Buffered Output**: Intelligent buffering for performance optimization
- **Backpressure Handling**: Graceful degradation under high load

**Processing Stages**:
1. **Capture**: Log event creation and initial formatting
2. **Enrich**: Context injection and metadata enhancement
3. **Filter**: Level-based and pattern-based filtering
4. **Format**: Final formatting for output destinations
5. **Dispatch**: Delivery to configured handlers
6. **Archive**: Long-term storage and rotation management

#### Real-Time Analysis Framework

**Pattern Detection**:
- **Error Pattern Recognition**: Automatic detection of recurring error patterns
- **Performance Anomaly Detection**: Statistical analysis of performance metrics
- **Security Event Detection**: Suspicious activity pattern identification

**Alert Generation**:
- **Threshold-Based Alerts**: Configurable thresholds for various metrics
- **Pattern-Based Alerts**: Complex pattern matching for sophisticated alerting
- **Escalation Policies**: Multi-tier alerting with escalation paths

### Color-Coded Visual Architecture

The logging system implements a **comprehensive visual coding system** that enables rapid visual parsing of log information.

#### Color Coding Strategy

**Severity Level Colors**:
- **DEBUG**: Cyan - Development and troubleshooting information
- **INFO**: Green - Normal operational information
- **WARNING**: Yellow - Attention-required but non-critical events
- **ERROR**: Red - Error conditions requiring investigation
- **CRITICAL**: Magenta + Bold - System-threatening conditions

**Node Type Colors**:
- **System**: Blue - Framework and infrastructure logs
- **Speech Recognition**: Light Cyan - Audio input processing
- **Speech Synthesis**: Light Green - Audio output processing
- **AI Cognition**: Light Magenta - AI processing and decision making

#### Visual Consistency Framework

**Cross-Platform Compatibility**:
- **Terminal Support**: ANSI color code compatibility across terminals
- **GUI Integration**: Color mapping for graphical log viewers
- **Accessibility**: High contrast options for visual accessibility

**Formatting Standards**:
- **Fixed-Width Columns**: Consistent alignment for easy scanning
- **Timestamp Formatting**: Standardized EST format across all logs
- **Message Truncation**: Intelligent truncation for long messages

### Performance and Scalability Design

The logging system is designed for **high-throughput operation** with minimal impact on application performance.

#### Performance Optimization Framework

**Write Performance**:
- **Asynchronous I/O**: Non-blocking log writes to prevent application delays
- **Batch Processing**: Intelligent batching of log entries for efficiency
- **Buffer Management**: Adaptive buffer sizing based on load patterns

**Memory Management**:
- **Bounded Buffers**: Fixed-size buffers to prevent memory leaks
- **Garbage Collection Optimization**: Minimal object allocation in hot paths
- **Resource Pooling**: Reusable objects for high-frequency operations

#### Scalability Architecture

**Horizontal Scaling**:
- **Distributed Logging**: Support for distributed log aggregation
- **Load Balancing**: Intelligent distribution of logging load
- **Partition Strategy**: Log partitioning for parallel processing

**Vertical Scaling**:
- **Resource Adaptation**: Dynamic resource allocation based on load
- **Performance Monitoring**: Real-time performance metrics collection
- **Bottleneck Detection**: Automatic identification of performance bottlenecks

### Error Handling and Recovery Architecture

The logging system implements **fault-tolerant design** that ensures logging continues to function even under adverse conditions.

#### Failure Mode Analysis

**Disk Space Exhaustion**:
- **Emergency Cleanup**: Automatic cleanup of oldest logs
- **Degraded Mode**: Reduced logging to essential events only
- **Alert Generation**: Immediate notification of storage issues

#### Recovery Mechanisms

### Security and Compliance Architecture

The logging system implements **security-first design** with comprehensive protection for sensitive information and audit trail integrity.


This comprehensive technical design ensures that the Nevil v3.0 logging system provides robust, scalable, and maintainable logging capabilities while maintaining the simplicity and reliability required for robotic applications.

## 1. Logging Architecture

### 1.1 System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Logging System (KEEP_LOG_FILES=3)       │
│  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐│
│  │ Log Manager     │ │ Log Formatter   │ │ File Rotator    ││
│  │                 │ │ (EST + Colors)  │ │ (Strict Limit)  ││
│  └─────────────────┘ └─────────────────┘ └─────────────────┘│
│  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐│
│  │ Console Handler │ │ File Handler    │ │ Cleanup Monitor ││
│  │ (Color Coded)   │ │ (Numbered)      │ │ (Auto Prune)    ││
│  └─────────────────┘ └─────────────────┘ └─────────────────┘│
└─────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────┐
│              Strict Log Storage (Max 12 Files)             │
│  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐│
│  │ System Logs     │ │ Node Logs       │ │ Numbered Backups││
│  │ current + .1+.2 │ │ current + .1+.2 │ │ Auto-pruned     ││
│  └─────────────────┘ └─────────────────┘ └─────────────────┘│
└─────────────────────────────────────────────────────────────┘
```

### 1.2 Core Design Principles

- **KEEP_LOG_FILES=3**: Strict limit of current file + 2 numbered backups per log type
- **Readability First**: EST timestamps, color coding, clear node identification
- **Bounded Storage**: Maximum 12 total files across entire system
- **Simple Rotation**: Numbered files (.1, .2) without compression or archiving
- **Auto Pruning**: Background cleanup enforces file limits automatically

### 1.2 Log Levels

Nevil v3.0 uses five standard log levels with specific purposes:

| Level    | Value | Purpose | Console | File | Examples |
|----------|-------|---------|---------|------|----------|
| DEBUG    | 10    | Detailed diagnostic information | No | Yes | Variable values, function entry/exit |
| INFO     | 20    | General information | Yes | Yes | Node startup, configuration loaded |
| WARNING  | 30    | Warning conditions | Yes | Yes | Deprecated features, recoverable errors |
| ERROR    | 40    | Error conditions | Yes | Yes | Failed operations, exceptions |
| CRITICAL | 50    | Critical system failures | Yes | Yes | System shutdown, unrecoverable errors |

### 1.3 Log Directory Structure (KEEP_LOG_FILES=3 Enforcement)

The file formats for log files are here: [node_log_name][date created:month.day].log[.# of backup log(current log doesn't have this suffix)]

```
logs/
├── system.log                     # Current system log
├── system.log.1                   # Previous system log
├── system.log.2                   # Oldest system log (deleted on next rotation)
├── speech_recognition.8.14.log         # Current speech recognition log
├── speech_recognition.8.12.log.1       # Previous speech recognition log
├── speech_recognition.8.13.log.2       # Oldest speech recognition log
├── speech_synthesis.8.14.log           # Current speech synthesis log
├── speech_synthesis.8.12.log.1         # Previous speech synthesis log
├── speech_synthesis.8.10.log.2         # Oldest speech synthesis log
├── ai_cognition.8.14.log              # Current AI cognition log
├── ai_cognition.8.12.log.1            # Previous AI cognition log
└── ai_cognition.8.13.log.2            # Oldest AI cognition log

# STRICT LIMIT: Maximum 12 files total (3 per log type × 4 types)
# ROTATION FLOW: .log → .log.1 → .log.2 → DELETE
# ENFORCEMENT: KEEP_LOG_FILES=3 constant prevents unbounded growth
```

### 1.4 Strict File Retention Policy (KEEP_LOG_FILES=3)

The Nevil v3.0 logging system implements an aggressive file retention policy to prevent log bloat and ensure predictable disk usage.

#### 1.4.1 File Limit Enforcement
- **Constant**: `KEEP_LOG_FILES = 3` (hardcoded in NevilLogManager)
- **Per Log Type**: Current file + maximum 2 numbered backups
- **System Total**: Maximum 12 files (4 log types × 3 files each)
- **Auto Cleanup**: Background thread continuously enforces limits

#### 1.4.2 Rotation Behavior
```python
# Rotation sequence for each log type:
system.log          # Current active log
system.log.1        # Previous log (after rotation)
system.log.2        # Oldest log (deleted on next rotation)

# When system.log reaches size limit:
# 1. system.log.2 → DELETED
# 2. system.log.1 → system.log.2
# 3. system.log   → system.log.1
# 4. New system.log created
```

#### 1.4.3 Storage Guarantees
- **Bounded Growth**: Never more than 12 total log files
- **Predictable Size**: Each log ≤ 100MB, total ≤ 1.2GB maximum
- **No Runaway Logs**: Automatic cleanup prevents disk exhaustion
- **Fast Rotation**: Simple file moves, no compression overhead

#### 1.4.4 Implementation Details
```python
class NevilLogManager:
    def __init__(self):
        # STRICT LOG FILE LIMIT - Keep only current + last 2 files
        self.KEEP_LOG_FILES = 3

    def _enforce_file_limit(self, base_log_file: Path):
        """Enforce KEEP_LOG_FILES limit for a specific log file"""
        # Automatically remove files beyond .2 suffix
        # Called by background cleanup thread every hour
```

## 2. Log Manager Implementation

### 2.1 Core Log Manager

#### Code Summary

This section implements the NevilLogManager class for centralized logging management with KEEP_LOG_FILES=3 enforcement and comprehensive log handling:

**Classes:**
- **NevilLogManager**: Central logging coordinator with automatic rotation, EST timezone handling, and structured logging setup

**Key Methods in NevilLogManager:**
- `__init__()`: Initialize log manager with configuration, logger registry, and handler management
- `setup_logger()`: Create configured logger with formatters, handlers, and rotation policies
- `create_file_handler()`: Set up file handlers with automatic rotation and KEEP_LOG_FILES=3 enforcement
- `create_console_handler()`: Configure console output with color coding and readability formatting
- `setup_rotation()`: Implement size-based and time-based log rotation with strict file limits
- `cleanup_old_logs()`: Enforce KEEP_LOG_FILES=3 by removing excess backup files
- `get_logger()`: Retrieve or create logger instance with automatic configuration
- `shutdown()`: Clean shutdown of all handlers and loggers

**Key Features:**
- KEEP_LOG_FILES=3 strict enforcement with automatic cleanup
- EST timezone handling for consistent timestamps
- Multiple handler types (file, console, rotating file)
- Automatic log rotation based on size and time
- Thread-safe operations with proper locking
- Configurable log levels and formatting
- Centralized logger registry and management

```python
# nevil_framework/log_manager.py

import os
import logging
import logging.handlers
import threading
import time
import gzip
import shutil
from typing import Dict, Optional, List
from datetime import datetime, timedelta
from pathlib import Path

class NevilLogManager:
    """
    Centralized logging manager for Nevil v3.0.
    
    Provides structured logging with automatic rotation, archiving,
    and configurable output destinations.
    """
    
    def __init__(self, config: Dict = None):
        self.config = config or {}
        self.loggers = {}  # logger_name -> logger
        self.handlers = {}  # handler_name -> handler
        self.lock = threading.RLock()
        
        # Configuration
        self.log_level = self.config.get('log_level', 'INFO')
        self.max_log_size_mb = self.config.get('max_log_size_mb', 100)
        self.log_retention_days = self.config.get('log_retention_days', 30)
        self.console_level = self.config.get('console_level', 'WARNING')
        self.mask_secrets = self.config.get('mask_secrets', True)

        # STRICT LOG FILE LIMIT - Keep only current + last 2 files
        self.KEEP_LOG_FILES = 3
        
        # Paths
        self.log_dir = Path("logs")
        # No separate archive directory - numbered rotation in place
        
        # Create directories
        self._create_log_directories()
        
        # Setup formatters
        self._setup_formatters()
        
        # Start cleanup thread
        self._start_cleanup_thread()
    
    def _create_log_directories(self):
        """Create necessary log directories"""
        self.log_dir.mkdir(exist_ok=True)
        # Only main logs directory - no subdirectories needed
    
    def _setup_formatters(self):
        """Setup log formatters with readability-first design"""
        # Enhanced EST timezone formatter for file logs
        self.file_formatter = ReadabilityFormatter(
            fmt='[%(est_timestamp)s EST] [%(colored_level)s] [%(node_info)s] [%(component)s] %(message)s',
            use_colors=False
        )

        # Color-coded console formatter for readability
        self.console_formatter = ReadabilityFormatter(
            fmt='[%(est_timestamp)s EST] [%(colored_level)s] [%(node_info)s] %(message)s',
            use_colors=True
        )

        # Debug formatter with enhanced context
        self.debug_formatter = ReadabilityFormatter(
            fmt='[%(est_timestamp)s EST] [%(colored_level)s] [%(node_info)s] [%(component)s] '
            '[%(filename)s:%(lineno)d] [T:%(thread)d] %(message)s',
            use_colors=False,
            include_microseconds=True
        )
    
    def get_logger(self, name: str, node_name: str = None) -> logging.Logger:
        """
        Get or create a logger for a component.

        Args:
            name: Logger name (e.g., 'speech_recognition', 'system')
            node_name: Node name for node-specific loggers

        Returns:
            Configured logger instance
        """
        with self.lock:
            if name in self.loggers:
                return self.loggers[name]

            # Create logger
            logger = logging.getLogger(name)
            logger.setLevel(getattr(logging, self.log_level.upper()))

            # Prevent duplicate handlers
            logger.handlers.clear()
            logger.propagate = False

            # Add structured filter to inject node context
            if node_name:
                filter_obj = StructuredLogFilter(node_name=node_name, component=name)
                logger.addFilter(filter_obj)

            # Add handlers
            if node_name:
                self._add_node_handlers(logger, node_name)
            else:
                self._add_system_handlers(logger, name)

            self.loggers[name] = logger
            return logger

    def get_system_logger(self, source_node: str = None) -> logging.Logger:
        """
        Get system logger with optional node source identification.

        Args:
            source_node: Source node for color coding (e.g., 'speech_recognition')

        Returns:
            System logger with node context for color coding
        """
        logger_name = "system"
        if source_node:
            logger_name = f"system.{source_node}"

        return self.get_logger(logger_name, node_name=source_node)
    
    def _add_system_handlers(self, logger: logging.Logger, name: str):
        """Add handlers for system-level loggers"""
        # Console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(getattr(logging, self.console_level.upper()))
        console_handler.setFormatter(self.console_formatter)
        logger.addHandler(console_handler)
        
        # File handler for main log
        file_path = self.log_dir / f"{name}.log"
        file_handler = self._create_rotating_handler(file_path)
        file_handler.setLevel(logging.INFO)
        file_handler.setFormatter(self.file_formatter)
        logger.addHandler(file_handler)
        
        # Debug file handler
        debug_path = self.log_dir / f"{name}.debug.log"
        debug_handler = self._create_rotating_handler(debug_path)
        debug_handler.setLevel(logging.DEBUG)
        debug_handler.setFormatter(self.debug_formatter)
        logger.addHandler(debug_handler)
    
    def _add_node_handlers(self, logger: logging.Logger, node_name: str):
        """Add handlers for node-specific loggers"""
        # Create node log directory
        node_dir = self.node_log_dir / node_name
        node_dir.mkdir(exist_ok=True)
        
        # Console handler (only for warnings and errors)
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.WARNING)
        console_handler.setFormatter(self.console_formatter)
        logger.addHandler(console_handler)
        
        # Node file handler
        file_path = node_dir / f"{node_name}.log"
        file_handler = self._create_rotating_handler(file_path)
        file_handler.setLevel(logging.INFO)
        file_handler.setFormatter(self.file_formatter)
        logger.addHandler(file_handler)
        
        # Node debug handler
        debug_path = node_dir / f"{node_name}.debug.log"
        debug_handler = self._create_rotating_handler(debug_path)
        debug_handler.setLevel(logging.DEBUG)
        debug_handler.setFormatter(self.debug_formatter)
        logger.addHandler(debug_handler)
    
    def _create_rotating_handler(self, file_path: Path) -> logging.handlers.RotatingFileHandler:
        """Create a rotating file handler with strict 3-file limit"""
        max_bytes = self.max_log_size_mb * 1024 * 1024  # Convert MB to bytes

        handler = logging.handlers.RotatingFileHandler(
            filename=str(file_path),
            maxBytes=max_bytes,
            backupCount=self.KEEP_LOG_FILES - 1,  # -1 because current file counts as 1
            encoding='utf-8'
        )

        # Custom rotation behavior for strict file limit enforcement
        handler.rotator = self._strict_rotate_file
        handler.namer = self._simple_numbered_filename

        return handler
    
    def _strict_rotate_file(self, source: str, dest: str):
        """Simple file rotation without compression - enforces KEEP_LOG_FILES limit"""
        try:
            # Just move the file - no compression to keep it simple and fast
            shutil.move(source, dest)
        except Exception as e:
            print(f"Error rotating log file {source}: {e}")

    def _simple_numbered_filename(self, default_name: str) -> str:
        """Generate simple numbered filename for rotated logs"""
        # Extract base path and add simple .1, .2, etc. suffix
        path = Path(default_name)

        # Simple numbered rotation: log.txt → log.txt.1
        return f"{default_name}.1"
    
    def _start_cleanup_thread(self):
        """Start background thread for log cleanup"""
        def cleanup_loop():
            while True:
                try:
                    self._cleanup_old_logs()
                    time.sleep(3600)  # Run every hour
                except Exception as e:
                    print(f"Error in log cleanup: {e}")
                    time.sleep(3600)
        
        cleanup_thread = threading.Thread(target=cleanup_loop, daemon=True)
        cleanup_thread.start()
    
    def _cleanup_old_logs(self):
        """Enforce strict KEEP_LOG_FILES=3 limit for all log files"""
        try:
            # Find all log files in main directory
            for log_file in self.log_dir.glob("*.log"):
                self._enforce_file_limit(log_file)
        except Exception as e:
            print(f"Error in log cleanup: {e}")

    def _enforce_file_limit(self, base_log_file: Path):
        """Enforce KEEP_LOG_FILES limit for a specific log file"""
        try:
            # Check for numbered backups (e.g., system.log.1, system.log.2, etc.)
            base_name = base_log_file.name
            numbered_files = []

            # Find all numbered backup files for this log
            for i in range(1, 10):  # Check up to .9 just in case
                backup_file = self.log_dir / f"{base_name}.{i}"
                if backup_file.exists():
                    numbered_files.append((i, backup_file))

            # Sort by number (highest numbers are oldest)
            numbered_files.sort(key=lambda x: x[0], reverse=True)

            # Remove files beyond our limit
            # Keep current file + (KEEP_LOG_FILES - 1) numbered files
            max_numbered_files = self.KEEP_LOG_FILES - 1
            for i, (num, file_path) in enumerate(numbered_files):
                if i >= max_numbered_files:
                    file_path.unlink()
                    print(f"Removed excess log file: {file_path}")

        except Exception as e:
            print(f"Error enforcing file limit for {base_log_file}: {e}")
    
    def mask_sensitive_data(self, message: str) -> str:
        """Mask sensitive information in log messages"""
        if not self.mask_secrets:
            return message
        
        # Mask API keys
        import re
        
        patterns = [
            (r'sk-[a-zA-Z0-9]{48}', 'sk-***MASKED***'),
            (r'asst_[a-zA-Z0-9]{24}', 'asst_***MASKED***'),
            (r'"api_key":\s*"[^"]*"', '"api_key": "***MASKED***"'),
            (r'password["\']?\s*[:=]\s*["\']?[^"\'\s]+', 'password: ***MASKED***'),
        ]
        
        masked_message = message
        for pattern, replacement in patterns:
            masked_message = re.sub(pattern, replacement, masked_message, flags=re.IGNORECASE)
        
        return masked_message
    
    def get_log_stats(self) -> Dict[str, any]:
        """Get logging system statistics"""
        stats = {
            'total_loggers': len(self.loggers),
            'log_directories': [],
            'total_log_size_mb': 0,
            'oldest_log': None,
            'newest_log': None
        }
        
        # Scan log directories
        for log_path in self.log_dir.rglob("*.log"):
            try:
                size_mb = log_path.stat().st_size / (1024 * 1024)
                stats['total_log_size_mb'] += size_mb
                
                mtime = datetime.fromtimestamp(log_path.stat().st_mtime)
                if not stats['oldest_log'] or mtime < stats['oldest_log']:
                    stats['oldest_log'] = mtime
                if not stats['newest_log'] or mtime > stats['newest_log']:
                    stats['newest_log'] = mtime
                    
            except Exception:
                continue
        
        return stats

# TEST: Log manager creates proper directory structure
# TEST: Log rotation works correctly at size limits
# TEST: Sensitive data is properly masked
# TEST: Old logs are cleaned up according to retention policy
# TEST: Multiple loggers can be created without conflicts
```

### 2.2 Enhanced Readability Formatter

#### Code Summary

This section implements the ReadabilityFormatter class for human-readable log formatting with EST timestamps, color coding, and structured presentation:

**Classes:**
- **ReadabilityFormatter**: Advanced log formatter with EST timezone handling, color coding, and node identification

**Key Methods in ReadabilityFormatter:**
- `__init__()`: Initialize formatter with timezone, color settings, and formatting configuration
- `format()`: Main formatting method with EST timestamp conversion, color application, and structured layout
- `formatTime()`: Convert UTC timestamps to EST format with proper timezone handling
- `_apply_colors()`: Apply color coding based on log level and component type
- `_format_node_info()`: Extract and format node identification information
- `_truncate_message()`: Intelligent message truncation with preservation of important content
- `_format_exception()`: Enhanced exception formatting with stack trace and context

**Key Features:**
- EST timezone conversion for all timestamps (UTC to EST automatically)
- Color-coded output based on log levels (ERROR=red, WARNING=yellow, INFO=green, DEBUG=cyan)
- Node type identification with consistent formatting
- Component tracking and source identification
- Intelligent message truncation for readability
- Thread and process information inclusion
- Exception formatting with enhanced context
- Fixed-width column alignment for easy scanning
- Cross-platform color compatibility

```python
# nevil_framework/readability_formatter.py

import logging
import time
from datetime import datetime
from typing import Dict, Optional
import pytz

class ReadabilityFormatter(logging.Formatter):
    """
    Enhanced formatter for readability-first logging with EST timestamps,
    color coding, and clear source identification.
    """

    # Color codes for different severity levels
    LEVEL_COLORS = {
        'DEBUG': '\033[36m',      # Cyan
        'INFO': '\033[32m',       # Green
        'WARNING': '\033[33m',    # Yellow
        'ERROR': '\033[31m',      # Red
        'CRITICAL': '\033[35;1m', # Magenta + Bold
    }

    # Color codes for different node types
    NODE_COLORS = {
        'system': '\033[94m',           # Blue
        'speech_recognition': '\033[96m', # Light Cyan
        'speech_synthesis': '\033[92m',   # Light Green
        'ai_cognition': '\033[95m',       # Light Magenta
        'audio_interface': '\033[93m',    # Light Yellow
        'sensor_interface': '\033[97m',   # Light White
        'actuator_interface': '\033[91m', # Light Red
    }

    RESET_CODE = '\033[0m'  # Reset color
    BOLD_CODE = '\033[1m'   # Bold text

    def __init__(self, fmt: str = None, use_colors: bool = True, include_microseconds: bool = False):
        super().__init__()
        self.use_colors = use_colors
        self.include_microseconds = include_microseconds
        self.est_tz = pytz.timezone('US/Eastern')
        self._fmt = fmt or '[%(est_timestamp)s EST] [%(colored_level)s] [%(node_info)s] %(message)s'

    def format(self, record: logging.LogRecord) -> str:
        """Format log record with enhanced readability features"""
        # Add EST timestamp
        record.est_timestamp = self._get_est_timestamp(record.created)

        # Add colored level
        record.colored_level = self._get_colored_level(record.levelname)

        # Add node information with color coding
        record.node_info = self._get_node_info(record)

        # Add component information
        record.component = self._get_component_info(record)

        # Format the message
        formatted = self._fmt % record.__dict__

        # Clean up any remaining color codes if colors disabled
        if not self.use_colors:
            formatted = self._strip_colors(formatted)

        return formatted

    def _get_est_timestamp(self, timestamp: float) -> str:
        """Get EST formatted timestamp"""
        # Convert UTC timestamp to EST
        utc_dt = datetime.fromtimestamp(timestamp, tz=pytz.UTC)
        est_dt = utc_dt.astimezone(self.est_tz)

        if self.include_microseconds:
            return est_dt.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # Trim to milliseconds
        else:
            return est_dt.strftime('%Y-%m-%d %H:%M:%S')

    def _get_colored_level(self, level_name: str) -> str:
        """Get color-coded log level"""
        if not self.use_colors:
            return f"{level_name:8}"  # Fixed width without colors

        color = self.LEVEL_COLORS.get(level_name, '')
        if level_name in ['ERROR', 'CRITICAL']:
            # Make errors bold
            return f"{color}{self.BOLD_CODE}{level_name:8}{self.RESET_CODE}"
        else:
            return f"{color}{level_name:8}{self.RESET_CODE}"

    def _get_node_info(self, record: logging.LogRecord) -> str:
        """Get color-coded node information"""
        node_name = getattr(record, 'node_name', 'system')

        # For system logs, extract node information from logger name or message
        if node_name == 'system' and hasattr(record, 'name'):
            # Check if logger name contains node information
            if 'speech_recognition' in record.name.lower():
                node_name = 'speech_recognition'
            elif 'speech_synthesis' in record.name.lower():
                node_name = 'speech_synthesis'
            elif 'ai_cognition' in record.name.lower():
                node_name = 'ai_cognition'
            elif 'audio_interface' in record.name.lower():
                node_name = 'audio_interface'
            elif 'sensor_interface' in record.name.lower():
                node_name = 'sensor_interface'
            elif 'actuator_interface' in record.name.lower():
                node_name = 'actuator_interface'

        # Also check message content for node references in system logs
        if node_name == 'system' and hasattr(record, 'getMessage'):
            message = record.getMessage().lower()
            if any(node in message for node in ['speech_recognition', 'speech recognition']):
                node_name = 'speech_recognition'
            elif any(node in message for node in ['speech_synthesis', 'speech synthesis']):
                node_name = 'speech_synthesis'
            elif any(node in message for node in ['ai_cognition', 'ai cognition']):
                node_name = 'ai_cognition'
            elif any(node in message for node in ['audio_interface', 'audio interface']):
                node_name = 'audio_interface'

        if not self.use_colors:
            return f"{node_name:15}"  # Fixed width without colors

        color = self.NODE_COLORS.get(node_name, self.NODE_COLORS['system'])
        return f"{color}{node_name:15}{self.RESET_CODE}"

    def _get_component_info(self, record: logging.LogRecord) -> str:
        """Get component/module information"""
        # Use the logger name, but clean it up for readability
        component = getattr(record, 'name', 'unknown')

        # Shorten long module paths
        if '.' in component:
            parts = component.split('.')
            if len(parts) > 2:
                component = f"{parts[0]}...{parts[-1]}"

        return f"{component:20}"  # Fixed width

    def _strip_colors(self, text: str) -> str:
        """Remove color codes from text"""
        import re
        # Remove ANSI color codes
        ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
        return ansi_escape.sub('', text)

# TEST: Readability formatter produces consistent EST timestamps
# TEST: Color coding works correctly for different severity levels
# TEST: Node differentiation colors are applied properly
# TEST: Color stripping works for file output
# TEST: Fixed-width formatting maintains column alignment
```

### 2.2 Structured Logging Filter

#### Code Summary

This section implements structured logging filters for adding metadata and context to log records:

**Classes:**
- **StructuredLogFilter**: Adds structured metadata (node name, uptime, thread ID) to log records
- **PerformanceLogFilter**: Filters performance logs by duration threshold and adds timing context
- **ErrorContextFilter**: Enriches error logs with stack trace context and system state

**Key Methods in StructuredLogFilter:**
- `__init__()`: Initialize filter with node name, component identification, and start time tracking
- `filter()`: Add structured fields to log records including uptime, thread info, and performance timing

**Key Methods in PerformanceLogFilter:**
- `__init__()`: Initialize with duration threshold and performance tracking settings
- `filter()`: Filter logs based on duration threshold and add performance context

**Key Methods in ErrorContextFilter:**
- `__init__()`: Initialize with context collection settings and system state tracking
- `filter()`: Enrich error logs with stack traces, system metrics, and debugging context

**Key Features:**
- Structured metadata addition (node name, component, uptime, thread ID)
- Performance threshold filtering with timing information
- Error context enrichment with stack traces and system state
- Thread-safe operation with proper timing tracking
- Configurable filtering based on log content and performance metrics
- Integration with logging pipeline for automatic metadata injection

```python
# nevil_framework/log_filters.py

import logging
import json
import time
from typing import Dict, Any

class StructuredLogFilter(logging.Filter):
    """
    Filter that adds structured data to log records.
    """
    
    def __init__(self, node_name: str = None, component: str = None):
        super().__init__()
        self.node_name = node_name
        self.component = component
        self.start_time = time.time()
    
    def filter(self, record: logging.LogRecord) -> bool:
        # Add structured fields
        record.node_name = self.node_name or "system"
        record.component = self.component or record.name
        record.uptime = time.time() - self.start_time
        record.thread_id = record.thread
        
        # Add performance timing if available
        if hasattr(record, 'duration'):
            record.performance = f"{record.duration:.3f}s"
        
        return True

class PerformanceLogFilter(logging.Filter):
    """
    Filter for performance-related logging.
    """
    
    def __init__(self, slow_threshold: float = 1.0):
        super().__init__()
        self.slow_threshold = slow_threshold
    
    def filter(self, record: logging.LogRecord) -> bool:
        # Only log performance records that exceed threshold
        if hasattr(record, 'duration'):
            return record.duration >= self.slow_threshold
        return True

class ErrorContextFilter(logging.Filter):
    """
    Filter that adds context information to error logs.
    """
    
    def filter(self, record: logging.LogRecord) -> bool:
        if record.levelno >= logging.ERROR:
            # Add stack trace context
            if record.exc_info:
                record.error_context = {
                    'exception_type': record.exc_info[0].__name__,
                    'exception_message': str(record.exc_info[1]),
                    'module': record.module,
                    'function': record.funcName,
                    'line': record.lineno
                }
        return True

# TEST: Structured filter adds correct metadata to log records
# TEST: Performance filter only passes slow operations
# TEST: Error context filter enriches error logs with debugging info
```

## 3. Logging Utilities and Decorators

### 3.1 Performance Logging Decorator

#### Code Summary

This section implements logging utilities and decorators for function execution timing, exception handling, and structured context logging:

**Decorators:**
- **@log_performance**: Times function execution and logs duration with configurable threshold filtering
- **@log_exceptions**: Catches and logs exceptions with full context and stack traces
- **@log_execution_time**: Simple execution timing decorator with automatic duration logging

**Classes:**
- **ContextLogger**: Structured logging context manager with consistent field formatting and nested context support

**Key Features in @log_performance:**
- Configurable execution time threshold for filtering
- Automatic duration calculation and logging
- Function argument and return value logging (optional)
- Integration with performance monitoring systems

**Key Features in @log_exceptions:**
- Complete exception capture with stack traces
- Function context preservation (arguments, local variables)
- Automatic error recovery and re-raising
- Integration with error tracking systems

**Key Features in ContextLogger:**
- Structured context management with nested scopes
- Consistent field formatting across log entries
- Automatic context cleanup and resource management
- Thread-safe context tracking

**Key Features:**
- Function-level performance monitoring with decorators
- Comprehensive exception handling and context preservation
- Structured logging with consistent formatting
- Thread-safe operations with proper context management
- Configurable thresholds and filtering options
- Integration with existing logging infrastructure

```python
# nevil_framework/log_decorators.py

import time
import functools
import logging
from typing import Callable, Any

def log_performance(logger: logging.Logger = None, 
                   threshold: float = 1.0,
                   level: int = logging.INFO):
    """
    Decorator to log function performance.
    
    Args:
        logger: Logger to use (defaults to function's module logger)
        threshold: Only log if duration exceeds threshold (seconds)
        level: Log level to use
    """
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs) -> Any:
            start_time = time.time()
            
            try:
                result = func(*args, **kwargs)
                duration = time.time() - start_time
                
                # Get logger
                log = logger or logging.getLogger(func.__module__)
                
                # Log if exceeds threshold
                if duration >= threshold:
                    log.log(level, f"{func.__name__} completed in {duration:.3f}s", 
                           extra={'duration': duration, 'function': func.__name__})
                
                return result
                
            except Exception as e:
                duration = time.time() - start_time
                log = logger or logging.getLogger(func.__module__)
                log.error(f"{func.__name__} failed after {duration:.3f}s: {e}",
                         extra={'duration': duration, 'function': func.__name__})
                raise
        
        return wrapper
    return decorator

def log_entry_exit(logger: logging.Logger = None, 
                  level: int = logging.DEBUG):
    """
    Decorator to log function entry and exit.
    
    Args:
        logger: Logger to use
        level: Log level to use
    """
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs) -> Any:
            log = logger or logging.getLogger(func.__module__)
            
            # Log entry
            log.log(level, f"Entering {func.__name__}")
            
            try:
                result = func(*args, **kwargs)
                log.log(level, f"Exiting {func.__name__}")
                return result
            except Exception as e:
                log.log(level, f"Exiting {func.__name__} with exception: {e}")
                raise
        
        return wrapper
    return decorator

def log_errors(logger: logging.Logger = None,
               reraise: bool = True,
               default_return: Any = None):
    """
    Decorator to log and optionally handle errors.
    
    Args:
        logger: Logger to use
        reraise: Whether to reraise the exception
        default_return: Default value to return if not reraising
    """
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs) -> Any:
            try:
                return func(*args, **kwargs)
            except Exception as e:
                log = logger or logging.getLogger(func.__module__)
                log.error(f"Error in {func.__name__}: {e}", exc_info=True)
                
                if reraise:
                    raise
                else:
                    return default_return
        
        return wrapper
    return decorator

# TEST: Performance decorator logs slow functions correctly
# TEST: Entry/exit decorator provides proper function tracing
# TEST: Error decorator handles exceptions according to configuration
```

### 3.2 Context Manager for Logging

```python
# nevil_framework/log_context.py

import logging
import time
import threading
from contextlib import contextmanager
from typing import Dict, Any, Optional

class LogContext:
    """
    Thread-local context for structured logging.
    """
    
    def __init__(self):
        self._local = threading.local()
    
    def set_context(self, **kwargs):
        """Set context variables for current thread"""
        if not hasattr(self._local, 'context'):
            self._local.context = {}
        self._local.context.update(kwargs)
    
    def get_context(self) -> Dict[str, Any]:
        """Get context variables for current thread"""
        if not hasattr(self._local, 'context'):
            self._local.context = {}
        return self._local.context.copy()
    
    def clear_context(self):
        """Clear context for current thread"""
        if hasattr(self._local, 'context'):
            self._local.context.clear()

# Global context instance
log_context = LogContext()

@contextmanager
def logging_context(**kwargs):
    """
    Context manager for temporary logging context.
    
    Usage:
        with logging_context(request_id="123", user="admin"):
            logger.info("Processing request")  # Will include context
    """
    # Save current context
    old_context = log_context.get_context()
    
    try:
        # Set new context
        log_context.set_context(**kwargs)
        yield
    finally:
        # Restore old context
        log_context.clear_context()
        log_context.set_context(**old_context)

@contextmanager
def timed_operation(logger: logging.Logger, operation_name: str, level: int = logging.INFO):
    """
    Context manager for timing operations.
    
    Usage:
        with timed_operation(logger, "database_query"):
            result = db.query(...)
    """
    start_time = time.time()
    logger.log(level, f"Starting {operation_name}")
    
    try:
        yield
        duration = time.time() - start_time
        logger.log(level, f"Completed {operation_name} in {duration:.3f}s",
                  extra={'duration': duration, 'operation': operation_name})
    except Exception as e:
        duration = time.time() - start_time
        logger.error(f"Failed {operation_name} after {duration:.3f}s: {e}",
                    extra={'duration': duration, 'operation': operation_name})
        raise

# TEST: Log context maintains thread-local state correctly
# TEST: Context manager properly saves and restores context
# TEST: Timed operation context logs duration accurately
```

## 4. Log Analysis and Monitoring

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

## 5. Log Monitoring and Alerting

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

## 6. Log Configuration and Setup

### 6.1 Logging Configuration

```yaml
# Example logging configuration in .nodes file
system:
  log_level: "INFO"
  max_log_size_mb: 100
  log_retention_days: 30
  console_level: "WARNING"
  mask_secrets: true

logging:
  # File rotation settings
  rotation:
    max_size_mb: 100
    backup_count: 5
    compress: true
  
  # Log levels per component
  levels:
    system: "INFO"
    speech_recognition: "INFO"
    speech_synthesis: "INFO"
    ai_cognition: "DEBUG"
  
  # Alert configuration
  alerts:
    enabled: true
    email:
      enabled: false
      smtp_server: "smtp.gmail.com"
      smtp_port: 587
      username: "alerts@example.com"
      password: "${EMAIL_PASSWORD}"
      from_email: "nevil@example.com"
      to_emails: ["admin@example.com"]
    
    webhook:
      enabled: false
      url: "https://hooks.slack.com/services/..."
      headers:
        Content-Type: "application/json"
    
    rules:
      - pattern: "CRITICAL|FATAL"
        level: null
        description: "Critical system errors"
      
      - pattern: "OpenAI.*error"
        level: "ERROR"
        description: "OpenAI API errors"
      
      - pattern: "completed in [5-9]\d+\.\d+s"
        level: null
        description: "Slow operations (>50s)"
```

### 6.2 Setup Utilities

#### Code Summary

This section implements logging setup utilities and configuration functions for easy one-line logger initialization:

**Functions:**
- **setup_logging()**: Main setup function with configuration validation and logger manager creation
- **get_logger()**: One-line logger retrieval with automatic EST formatting and file rotation
- **configure_console_logging()**: Console logging setup with color coding and level filtering
- **configure_file_logging()**: File logging setup with rotation and KEEP_LOG_FILES=3 enforcement

**Key Features in setup_logging():**
- Configuration validation and default value application
- Logger manager initialization with proper handler setup
- Environment variable integration for configuration
- Error handling with fallback to default configuration

**Key Features in get_logger():**
- One-line logger retrieval with automatic configuration
- EST timezone formatting with proper handler setup
- File rotation with KEEP_LOG_FILES=3 enforcement
- Thread-safe logger creation and caching

**Key Features:**
- Simple one-line logger setup for rapid development
- Automatic EST timezone handling without manual configuration
- KEEP_LOG_FILES=3 enforcement built into all file handlers
- Console and file logging with proper formatting
- Configuration-driven setup with environment variable support
- Thread-safe operations with proper logger caching
- Error handling with graceful fallback to defaults

```python
# nevil_framework/logging_setup.py

def setup_logging(config: Dict = None) -> NevilLogManager:
    """
    Setup logging for Nevil v3.0 system.
    
    Args:
        config: Logging configuration dictionary
        
    Returns:
        Configured log manager instance
    """
    # Load configuration
    if not config:
        from nevil_framework.config_loader import ConfigLoader
        system_config = ConfigLoader().load_nodes_config()
        config = system_config.get('logging', {})
    
    # Create log manager
    log_manager = NevilLogManager(config)
    
    # Setup monitoring if enabled
    if config.get('alerts', {}).get('enabled', False):
        monitor = LogMonitor()
        alert_manager = AlertManager(config.get('alerts', {}))
        
        # Add alert rules
        for rule_config in config.get('alerts', {}).get('rules', []):
            monitor.add_alert_rule(
                pattern=rule_config['pattern'],
                level=rule_config.get('level'),
                callback=lambda log_data, desc=rule_config.get('description', ''):
                    alert_manager.send_alert('log_pattern', desc, log_data)
            )
        
        monitor.start_monitoring()
    
    return log_manager

def get_node_logger(node_name: str, log_manager: NevilLogManager = None) -> logging.Logger:
    """
    Get a logger for a specific node.
    
    Args:
        node_name: Name of the node
        log_manager: Log manager instance (optional)
        
    Returns:
        Configured logger for the node
    """
    if not log_manager:
        log_manager = setup_logging()
    
    return log_manager.get_logger(node_name, node_name)

# TEST: Logging setup creates proper directory structure
# TEST: Node loggers are configured with correct handlers
# TEST: Alert monitoring starts correctly when enabled
# TEST: Configuration validation catches invalid settings

def get_enhanced_node_logger(node_name: str, log_manager: NevilLogManager = None) -> Dict[str, logging.Logger]:
    """
    Get both node-specific and system loggers for a node with proper color coding.

    Args:
        node_name: Name of the node (e.g., 'speech_recognition')
        log_manager: Log manager instance (optional)

    Returns:
        Dictionary with 'node' and 'system' loggers
    """
    if not log_manager:
        log_manager = setup_logging()

    return {
        'node': log_manager.get_logger(node_name, node_name),
        'system': log_manager.get_system_logger(source_node=node_name)
    }

# Usage example in a node:
# loggers = get_enhanced_node_logger('speech_recognition')
# loggers['node'].info("Audio processing started")  # Goes to speech_recognition.log
# loggers['system'].info("Node startup completed")  # Goes to system.log with speech_recognition color coding
```

## Conclusion

The Nevil v3.0 logging architecture provides a comprehensive, scalable logging solution that supports:

- **Structured logging** with consistent formatting across all components
- **Automatic log rotation** and archiving to manage disk space
- **Real-time monitoring** with configurable alert rules
- **Performance tracking** and analysis capabilities
- **Security features** including sensitive data masking
- **Multiple output destinations** (console, files, remote systems)
- **Easy configuration** through YAML files
- **Thread-safe operation** for concurrent node execution

This logging system ensures that Nevil v3.0 provides excellent observability and debugging capabilities while maintaining the simplicity and reliability that are core to the framework's design philosophy.

## Summary of Key Requirements Implemented

### ✅ Readability First Logging
- **EST Timestamps**: All logs show Eastern Standard Time with clear "EST" suffix
- **Color Coding**: Severity levels and node types have distinct colors
- **Clear Source ID**: Every log entry clearly identifies originating node/component
- **Fixed Width Formatting**: Consistent column alignment for easy scanning

### ✅ System Log Node Type Color Coding
- **Intelligent Detection**: Automatically identifies node source in system.log entries
- **Consistent Colors**: Same color scheme across node logs and system logs
- **Visual Differentiation**: Instant identification of log entry source

### ✅ Strict Log File Retention (KEEP_LOG_FILES=3)
- **Hard Limit**: `KEEP_LOG_FILES = 3` constant enforces strict file limit
- **Per Log Type**: Current file + maximum 2 numbered backups (.1, .2)
- **Total System Limit**: Maximum 12 files total (4 types × 3 files each)
- **Auto Enforcement**: Background cleanup thread continuously prunes excess files
- **Simple Rotation**: Numbered sequence without compression or complex archiving
- **Bounded Storage**: Predictable disk usage ≤ 1.2GB maximum

### ✅ Enhanced Architecture
- **Centralized Logging**: All logs in single `/logs` directory
- **No Subdirectories**: Simplified flat structure for easy management
- **Dynamic Log Viewer**: Real-time monitoring with filtering and search
- **Thread-Safe**: Concurrent operation across multiple nodes
- **Performance Optimized**: Fast rotation and cleanup operations

The logging system successfully balances comprehensive observability with strict resource management, ensuring Nevil v3.0 remains lean and maintainable while providing excellent debugging capabilities.

# TEST: Complete logging system handles all log levels correctly
# TEST: Log rotation and archiving work under various conditions
# TEST: Real-time monitoring detects issues promptly
# TEST: Performance analysis provides actionable insights
# TEST: Alert system delivers notifications reliably

## 7. Dynamic Log Viewer System

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

## 8. Desktop GUI Log Monitoring Dashboard

## 8. LogScope GUI Monitor

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
See [LogScope GUI Monitor Documentation](./07_logscope_gui_monitor.md) for comprehensive implementation details, user interface guide, installation instructions, and troubleshooting information.
# TEST: All keyboard shortcuts function correctly
# TEST: Search and highlighting work with various patterns
# TEST: Node filtering and multi-view switching operate properly
# TEST: Pause-on-scroll behavior works as documented
# TEST: Memory management stays within configured limits