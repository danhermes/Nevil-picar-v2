# Nevil v3.0 Framework Core Components Pseudocode - Phase 2 Features

## Overview

This document contains the Phase 2 advanced features for Nevil v3.0 framework core components, including dependency management, advanced error handling, performance optimization, scalability enhancements, and security features.

## Dependency Management Framework (Phase 2)

**Dependency Types:**
- **Hard Dependencies**: Required services that must be available
- **Soft Dependencies**: Optional services that enhance functionality
- **Circular Dependencies**: Detection and resolution of circular references

## Error Handling Technical Architecture - Phase 2 Features

### Error Context Enrichment (Phase 2)
- **Stack Trace Analysis**: Automatic stack trace capture and analysis
- **System State Capture**: Snapshot of relevant system state at error time
- **Correlation Analysis**: Linking related errors across components
- **Performance Impact**: Assessment of error impact on system performance

### Recovery Strategy Framework (Phase 2)

**Automatic Recovery Patterns:**
- **Retry with Backoff**: Exponential backoff for transient failures
- **Circuit Breaker**: Temporary service isolation for persistent failures
- **Fallback Mechanisms**: Alternative execution paths for failed operations
- **Resource Cleanup**: Automatic cleanup of failed operations

**Recovery Decision Matrix (Phase 2):**
| Error Type | Severity | Recovery Action | Escalation Threshold |
|------------|----------|-----------------|---------------------|
| Transient | Low | Retry with backoff | 3 consecutive failures |
| Transient | High | Immediate retry + alert | 1 failure |
| Persistent | Low | Circuit breaker | 5 failures in 5 minutes |
| Persistent | High | Service restart | 2 failures |
| Critical | Any | System shutdown | Immediate |

### Error Reporting and Analytics (Phase 2)

**Error Aggregation:**
- **Pattern Detection**: Automatic detection of error patterns and trends
- **Root Cause Analysis**: Correlation analysis to identify root causes
- **Impact Assessment**: Analysis of error impact on system functionality
- **Trend Analysis**: Historical analysis of error patterns and frequencies

**Alert Generation Framework:**
- **Threshold-Based Alerts**: Configurable thresholds for error rates and patterns
- **Anomaly Detection**: Statistical analysis to detect unusual error patterns
- **Escalation Policies**: Multi-tier escalation based on error severity and frequency
- **Alert Suppression**: Intelligent suppression of duplicate and related alerts

## Performance and Scalability Architecture (Phase 2)

The framework core is designed for **high-performance operation** with predictable scalability characteristics.

### Performance Optimization Framework (Phase 2)

**Memory Management:**
- **Object Pooling**: Reuse of expensive objects to reduce garbage collection
- **Lazy Initialization**: Deferred initialization of expensive resources
- **Memory Monitoring**: Real-time monitoring of memory usage patterns
- **Garbage Collection Optimization**: Tuned garbage collection for low latency

**CPU Optimization:**
- **Thread Pool Management**: Intelligent thread pool sizing and management
- **Lock-Free Data Structures**: Use of lock-free structures where possible
- **CPU Affinity**: Strategic CPU affinity for performance-critical components
- **Batch Processing**: Batching of operations to reduce overhead

### Scalability Architecture (Phase 2)

**Horizontal Scaling:**
- **Stateless Design**: Components designed to be stateless where possible
- **Load Distribution**: Intelligent load distribution across component instances
- **Dynamic Scaling**: Automatic scaling based on load and performance metrics
- **Resource Partitioning**: Logical partitioning of resources for parallel processing

**Vertical Scaling:**
- **Resource Adaptation**: Dynamic resource allocation based on demand
- **Performance Monitoring**: Real-time performance metrics and optimization
- **Bottleneck Detection**: Automatic identification and mitigation of bottlenecks
- **Capacity Planning**: Predictive capacity planning based on usage patterns

## Security and Isolation Architecture (Phase 2)

The framework implements **defense-in-depth security** with multiple layers of protection and isolation.

### Component Isolation Framework (Phase 2)

**Process Isolation:**
- **Separate Address Spaces**: Components run in separate processes where appropriate
- **Resource Limits**: Strict resource limits to prevent resource exhaustion
- **Capability-Based Security**: Fine-grained capability control for component access
- **Sandbox Execution**: Sandboxed execution for untrusted or experimental components

**Communication Security (Phase 2):**
- **Message Authentication**: Cryptographic authentication of inter-component messages
- **Encryption**: Optional encryption for sensitive message content
- **Access Control**: Fine-grained access control for message topics and operations
- **Audit Logging**: Comprehensive audit logging of security-relevant events

### Data Protection Framework (Phase 2)

**Sensitive Data Handling:**
- **Data Classification**: Automatic classification of sensitive data types
- **Encryption at Rest**: Encryption of sensitive data in configuration and logs
- **Secure Transmission**: Encrypted transmission of sensitive data between components
- **Data Minimization**: Minimization of sensitive data retention and exposure

**Privacy Protection:**
- **PII Detection**: Automatic detection of personally identifiable information
- **Data Anonymization**: Anonymization capabilities for privacy protection
- **Consent Management**: Integration with consent management frameworks
- **Data Retention**: Configurable data retention policies for privacy compliance

## Advanced Error Handler Implementation (Phase 2)

```python
# nevil_framework/advanced_error_handler.py

import traceback
import threading
import time
import json
import hashlib
from typing import Dict, List, Callable, Any, Optional
from dataclasses import dataclass, field
from enum import Enum
from collections import defaultdict, deque

class ErrorPattern:
    """Represents a detected error pattern"""
    
    def __init__(self, pattern_id: str, description: str):
        self.pattern_id = pattern_id
        self.description = description
        self.occurrences = []
        self.first_seen = None
        self.last_seen = None
        self.frequency = 0.0
        self.severity_distribution = defaultdict(int)

class AdvancedErrorHandler:
    """
    Advanced error handling with pattern detection, root cause analysis,
    and predictive failure prevention.
    """
    
    def __init__(self):
        self.error_patterns = {}  # pattern_id -> ErrorPattern
        self.error_correlations = {}  # error_id -> list of related error_ids
        self.system_state_snapshots = deque(maxlen=100)
        self.performance_baselines = {}
        
        # Pattern detection
        self.pattern_window_hours = 24
        self.min_pattern_occurrences = 3
        self.correlation_time_window = 300  # 5 minutes
        
        # Predictive analysis
        self.failure_predictors = []
        self.prediction_models = {}
        
    def analyze_error_patterns(self, hours: int = 24) -> Dict[str, Any]:
        """
        Analyze error patterns over the specified time period.
        
        Returns:
            Analysis report with detected patterns and recommendations
        """
        cutoff_time = time.time() - (hours * 3600)
        
        # Collect recent errors
        recent_errors = [
            error for error in self.errors.values()
            if error.timestamp >= cutoff_time
        ]
        
        # Group errors by similarity
        error_groups = self._group_similar_errors(recent_errors)
        
        # Detect patterns
        patterns = []
        for group in error_groups:
            if len(group) >= self.min_pattern_occurrences:
                pattern = self._create_error_pattern(group)
                patterns.append(pattern)
        
        # Analyze correlations
        correlations = self._analyze_error_correlations(recent_errors)
        
        # Generate recommendations
        recommendations = self._generate_pattern_recommendations(patterns, correlations)
        
        return {
            'analysis_period': f"Last {hours} hours",
            'total_errors': len(recent_errors),
            'detected_patterns': len(patterns),
            'patterns': [self._serialize_pattern(p) for p in patterns],
            'correlations': correlations,
            'recommendations': recommendations,
            'risk_assessment': self._assess_system_risk(patterns, correlations)
        }
    
    def predict_failures(self, node_name: str = None) -> Dict[str, Any]:
        """
        Predict potential failures based on current trends and patterns.
        
        Args:
            node_name: Specific node to analyze (None for system-wide)
            
        Returns:
            Failure prediction report
        """
        predictions = {
            'high_risk_nodes': [],
            'predicted_failures': [],
            'recommended_actions': [],
            'confidence_scores': {}
        }
        
        # Analyze error trends
        for node, errors in self._group_errors_by_node().items():
            if node_name and node != node_name:
                continue
                
            risk_score = self._calculate_node_risk_score(node, errors)
            predictions['confidence_scores'][node] = risk_score
            
            if risk_score > 0.7:  # High risk threshold
                predictions['high_risk_nodes'].append({
                    'node': node,
                    'risk_score': risk_score,
                    'primary_risk_factors': self._identify_risk_factors(node, errors)
                })
        
        # Generate specific failure predictions
        predictions['predicted_failures'] = self._generate_failure_predictions()
        
        # Generate recommended actions
        predictions['recommended_actions'] = self._generate_preventive_actions(predictions)
        
        return predictions
    
    def _group_similar_errors(self, errors: List[ErrorInfo]) -> List[List[ErrorInfo]]:
        """Group similar errors together"""
        groups = defaultdict(list)
        
        for error in errors:
            # Create signature for grouping
            signature = self._create_error_signature(error)
            groups[signature].append(error)
        
        return list(groups.values())
    
    def _create_error_signature(self, error: ErrorInfo) -> str:
        """Create a signature for error grouping"""
        # Combine error type, node, and key parts of message
        key_parts = [
            error.error_type,
            error.node_name,
            self._extract_error_key(error.message)
        ]
        
        signature_text = "|".join(key_parts)
        return hashlib.md5(signature_text.encode()).hexdigest()[:16]
    
    def _extract_error_key(self, message: str) -> str:
        """Extract key parts of error message for pattern matching"""
        import re
        
        # Remove variable parts (numbers, timestamps, IDs)
        cleaned = re.sub(r'\d+', 'N', message)
        cleaned = re.sub(r'\b[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}\b', 'UUID', cleaned)
        cleaned = re.sub(r'\b\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\b', 'TIMESTAMP', cleaned)
        
        # Take first 100 characters
        return cleaned[:100]
    
    def _analyze_error_correlations(self, errors: List[ErrorInfo]) -> List[Dict[str, Any]]:
        """Analyze correlations between errors"""
        correlations = []
        
        # Sort errors by timestamp
        sorted_errors = sorted(errors, key=lambda e: e.timestamp)
        
        # Look for errors that occur close together
        for i, error1 in enumerate(sorted_errors):
            for j, error2 in enumerate(sorted_errors[i+1:], i+1):
                time_diff = error2.timestamp - error1.timestamp
                
                if time_diff > self.correlation_time_window:
                    break  # Too far apart
                
                # Check for correlation indicators
                correlation_strength = self._calculate_correlation_strength(error1, error2)
                
                if correlation_strength > 0.5:
                    correlations.append({
                        'error1_id': error1.error_id,
                        'error2_id': error2.error_id,
                        'time_difference': time_diff,
                        'correlation_strength': correlation_strength,
                        'correlation_type': self._identify_correlation_type(error1, error2)
                    })
        
        return correlations
    
    def _calculate_correlation_strength(self, error1: ErrorInfo, error2: ErrorInfo) -> float:
        """Calculate correlation strength between two errors"""
        strength = 0.0
        
        # Same node increases correlation
        if error1.node_name == error2.node_name:
            strength += 0.3
        
        # Similar error types
        if error1.error_type == error2.error_type:
            strength += 0.4
        
        # Related error types (e.g., ConnectionError -> TimeoutError)
        related_types = {
            'ConnectionError': ['TimeoutError', 'NetworkError'],
            'TimeoutError': ['ConnectionError', 'ResponseError'],
            'MemoryError': ['ResourceError', 'AllocationError']
        }
        
        if error1.error_type in related_types:
            if error2.error_type in related_types[error1.error_type]:
                strength += 0.3
        
        # Message similarity
        message_similarity = self._calculate_message_similarity(error1.message, error2.message)
        strength += message_similarity * 0.3
        
        return min(strength, 1.0)
    
    def _calculate_message_similarity(self, msg1: str, msg2: str) -> float:
        """Calculate similarity between error messages"""
        # Simple word-based similarity
        words1 = set(msg1.lower().split())
        words2 = set(msg2.lower().split())
        
        if not words1 or not words2:
            return 0.0
        
        intersection = words1.intersection(words2)
        union = words1.union(words2)
        
        return len(intersection) / len(union)
    
    def _generate_pattern_recommendations(self, patterns: List[ErrorPattern], 
                                        correlations: List[Dict]) -> List[str]:
        """Generate recommendations based on detected patterns"""
        recommendations = []
        
        # High-frequency patterns
        for pattern in patterns:
            if pattern.frequency > 0.1:  # More than 10% of errors
                recommendations.append(
                    f"High-frequency error pattern detected: {pattern.description}. "
                    f"Consider implementing specific handling for this pattern."
                )
        
        # Correlated errors
        if len(correlations) > 5:
            recommendations.append(
                "Multiple correlated errors detected. Investigate potential root causes "
                "that may be triggering cascading failures."
            )
        
        # Node-specific issues
        node_error_counts = defaultdict(int)
        for pattern in patterns:
            for occurrence in pattern.occurrences:
                node_error_counts[occurrence.node_name] += 1
        
        for node, count in node_error_counts.items():
            if count > 10:
                recommendations.append(
                    f"Node {node} has high error frequency ({count} pattern occurrences). "
                    f"Consider node-specific debugging and optimization."
                )
        
        return recommendations
    
    def _assess_system_risk(self, patterns: List[ErrorPattern], 
                           correlations: List[Dict]) -> Dict[str, Any]:
        """Assess overall system risk based on error analysis"""
        risk_factors = {
            'pattern_diversity': len(patterns),
            'correlation_density': len(correlations),
            'critical_error_rate': 0.0,
            'node_failure_risk': {},
            'overall_risk_level': 'LOW'
        }
        
        # Calculate critical error rate
        total_errors = sum(len(p.occurrences) for p in patterns)
        critical_errors = sum(
            p.severity_distribution.get('CRITICAL', 0) for p in patterns
        )
        
        if total_errors > 0:
            risk_factors['critical_error_rate'] = critical_errors / total_errors
        
        # Assess overall risk level
        if risk_factors['critical_error_rate'] > 0.1:
            risk_factors['overall_risk_level'] = 'CRITICAL'
        elif risk_factors['critical_error_rate'] > 0.05 or len(correlations) > 10:
            risk_factors['overall_risk_level'] = 'HIGH'
        elif len(patterns) > 5 or len(correlations) > 5:
            risk_factors['overall_risk_level'] = 'MEDIUM'
        
        return risk_factors

# TEST: Error pattern detection identifies recurring issues
# TEST: Correlation analysis finds related errors accurately
# TEST: Risk assessment provides actionable insights
# TEST: Failure prediction models improve over time
```

## Performance Monitoring Framework (Phase 2)

```python
# nevil_framework/performance_monitor.py

import time
import threading
import psutil
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, field
from collections import deque, defaultdict

@dataclass
class PerformanceMetrics:
    timestamp: float
    node_name: str
    cpu_usage: float
    memory_usage: float
    thread_count: int
    message_throughput: float
    response_time: float
    error_rate: float
    custom_metrics: Dict[str, float] = field(default_factory=dict)

class PerformanceMonitor:
    """
    Advanced performance monitoring with trend analysis,
    anomaly detection, and optimization recommendations.
    """
    
    def __init__(self, collection_interval: float = 5.0):
        self.collection_interval = collection_interval
        self.metrics_history = defaultdict(lambda: deque(maxlen=1000))
        self.baselines = {}
        self.anomaly_thresholds = {}
        self.optimization_rules = []
        
        # Monitoring state
        self.running = False
        self.monitor_thread = None
        self.lock = threading.RLock()
        
        # Performance targets
        self.performance_targets = {
            'max_cpu_usage': 80.0,
            'max_memory_usage': 512.0,  # MB
            'max_response_time': 1.0,   # seconds
            'max_error_rate': 0.05      # 5%
        }
    
    def start_monitoring(self):
        """Start performance monitoring"""
        if self.running:
            return
        
        self.running = True
        self.monitor_thread = threading.Thread(
            target=self._monitoring_loop,
            name="PerformanceMonitor",
            daemon=True
        )
        self.monitor_thread.start()
    
    def stop_monitoring(self):
        """Stop performance monitoring"""
        self.running = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=5.0)
    
    def record_metrics(self, node_name: str, metrics: PerformanceMetrics):
        """Record performance metrics for a node"""
        with self.lock:
            self.metrics_history[node_name].append(metrics)
            
            # Update baselines
            self._update_baselines(node_name, metrics)
            
            # Check for anomalies
            anomalies = self._detect_anomalies(node_name, metrics)
            if anomalies:
                self._handle_performance_anomalies(node_name, anomalies)
    
    def get_performance_summary(self, node_name: str = None, 
                              hours: int = 1) -> Dict[str, Any]:
        """Get performance summary for a node or all nodes"""
        cutoff_time = time.time() - (hours * 3600)
        
        if node_name:
            return self._get_node_performance_summary(node_name, cutoff_time)
        else:
            return self._get_system_performance_summary(cutoff_time)
    
    def _monitoring_loop(self):
        """Main monitoring loop"""
        while self.running:
            try:
                # Collect system-wide metrics
                system_metrics = self._collect_system_metrics()
                self.record_metrics('system', system_metrics)
                
                # Sleep until next collection
                time.sleep(self.collection_interval)
                
            except Exception as e:
                print(f"Error in performance monitoring: {e}")
                time.sleep(self.collection_interval)
    
    def _collect_system_metrics(self) -> PerformanceMetrics:
        """Collect system-wide performance metrics"""
        return PerformanceMetrics(
            timestamp=time.time(),
            node_name='system',
            cpu_usage=psutil.cpu_percent(),
            memory_usage=psutil.virtual_memory().used / (1024 * 1024),  # MB
            thread_count=threading.active_count(),
            message_throughput=0.0,  # Would be calculated from message bus
            response_time=0.0,       # Would be calculated from request tracking
            error_rate=0.0           # Would be calculated from error handler
        )
    
    def _detect_anomalies(self, node_name: str, metrics: PerformanceMetrics) -> List[str]:
        """Detect performance anomalies"""
        anomalies = []
        
        # Check against performance targets
        if metrics.cpu_usage > self.performance_targets['max_cpu_usage']:
            anomalies.append(f"High CPU usage: {metrics.cpu_usage:.1f}%")
        
        if metrics.memory_usage > self.performance_targets['max_memory_usage']:
            anomalies.append(f"High memory usage: {metrics.memory_usage:.1f}MB")
        
        if metrics.response_time > self.performance_targets['max_response_time']:
            anomalies.append(f"High response time: {metrics.response_time:.3f}s")
        
        if metrics.error_rate > self.performance_targets['max_error_rate']:
            anomalies.append(f"High error rate: {metrics.error_rate:.1%}")
        
        # Check against baselines (if available)
        if node_name in self.baselines:
            baseline = self.baselines[node_name]
            
            # CPU usage anomaly (>2x baseline)
            if metrics.cpu_usage > baseline['cpu_usage'] * 2:
                anomalies.append(f"CPU usage spike: {metrics.cpu_usage:.1f}% (baseline: {baseline['cpu_usage']:.1f}%)")
            
            # Memory usage anomaly (>1.5x baseline)
            if metrics.memory_usage > baseline['memory_usage'] * 1.5:
                anomalies.append(f"Memory usage spike: {metrics.memory_usage:.1f}MB (baseline: {baseline['memory_usage']:.1f}MB)")
        
        return anomalies

# TEST: Performance monitoring collects accurate metrics
# TEST: Anomaly detection identifies performance issues
# TEST: Baseline calculation adapts to normal operation patterns
# TEST: Performance recommendations are actionable
```

## Resource Management Framework (Phase 2)

```python
# nevil_framework/resource_manager.py

import threading
import time
import psutil
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass
from enum import Enum

class ResourceType(Enum):
    CPU = "cpu"
    MEMORY = "memory"
    DISK = "disk"
    NETWORK = "network"
    GPU = "gpu"

@dataclass
class ResourceLimit:
    resource_type: ResourceType
    soft_limit: float
    hard_limit: float
    unit: str
    enforcement_action: str = "throttle"

@dataclass
class ResourceUsage:
    timestamp: float
    node_name: str
    resource_type: ResourceType
    current_usage: float
    peak_usage: float
    average_usage: float
    unit: str

class ResourceManager:
    """
    Advanced resource management with quotas, throttling,
    and intelligent allocation.
    """
    
    def __init__(self):
        self.resource_limits = {}  # node_name -> {resource_type -> ResourceLimit}
        self.resource_usage = {}   # node_name -> {resource_type -> ResourceUsage}
        self.resource_pools = {}   # resource_type -> available_amount
        self.allocation_policies = {}
        
        # Monitoring
        self.monitoring_enabled = True
        self.monitor_thread = None
        self.running = False
        self.lock = threading.RLock()
        
        # Enforcement
        self.enforcement_callbacks = {}  # resource_type -> callback
        
        # Initialize system resource pools
        self._initialize_resource_pools()
    
    def set_resource_limit(self, node_name: str, resource_type: ResourceType,
                          soft_limit: float, hard_limit: float, unit: str = ""):
        """Set resource limits for a node"""
        with self.lock:
            if node_name not in self.resource_limits:
                self.resource_limits[node_name] = {}
            
            self.resource_limits[node_name][resource_type] = ResourceLimit(
                resource_type=resource_type,
                soft_limit=soft_limit,
                hard_limit=hard_limit,
                unit=unit
            )
    
    def allocate_resources(self, node_name: str, 
                          resource_requests: Dict[ResourceType, float]) -> Dict[ResourceType, float]:
        """Allocate resources to a node based on availability and policies"""
        allocated = {}
        
        with self.lock:
            for resource_type, requested_amount in resource_requests.items():
                available = self.resource_pools.get(resource_type, 0.0)
                
                # Apply allocation policy
                policy = self.allocation_policies.get(resource_type, 'first_come_first_served')
                
                if policy == 'proportional':
                    allocated_amount = self._proportional_allocation(
                        node_name, resource_type, requested_amount, available
                    )
                elif policy == 'priority_based':
                    allocated_amount = self._priority_based_allocation(
                        node_name, resource_type, requested_amount, available
                    )
                else:  # first_come_first_served
                    allocated_amount = min(requested_amount, available)
                
                allocated[resource_type] = allocated_amount
                
                # Update resource pool
                if resource_type in self.resource_pools:
                    self.resource_pools[resource_type] -= allocated_amount
        
        return allocated
    
    def monitor_resource_usage(self, node_name: str) -> Dict[ResourceType, ResourceUsage]:
        """Monitor current resource usage for a node"""
        usage = {}
        
        try:
            # Get process info (would need actual process tracking)
            # This is a simplified example
            current_time = time.time()
            
            # CPU usage
            cpu_usage = psutil.cpu_percent(interval=0.1)
            usage[ResourceType.CPU] = ResourceUsage(
                timestamp=current_time,
                node_name=node_name,
                resource_type=ResourceType.CPU,
                current_usage=cpu_usage,
                peak_usage=cpu_usage,  # Would track actual peak
                average_usage=cpu_usage,  # Would calculate actual average
                unit="percent"
            )
            
            # Memory usage
            memory_info = psutil.virtual_memory()
            memory_usage = memory_info.used / (1024 * 1024)  # MB
            usage[ResourceType.MEMORY] = ResourceUsage(
                timestamp=current_time,
                node_name=node_name,
                resource_type=ResourceType.MEMORY,
                current_usage=memory_usage,
                peak_usage=memory_usage,
                average_usage=memory_usage,
                unit="MB"
            )
            
            # Store usage history
            with self.lock:
                if node_name not in self.resource_usage:
                    self.resource_usage[node_name] = {}
                self.resource_usage[node_name].update(usage)
            
            # Check limits and enforce
            self._enforce_resource_limits(node_name, usage)
            
        except Exception as e:
            print(f"Error monitoring resource usage for {node_name}: {e}")
        
        return usage
    
    def _enforce_resource_limits(self, node_name: str, 
                               usage: Dict[ResourceType, ResourceUsage]):
        """Enforce resource limits for a node"""
        if node_name not in self.resource_limits:
            return
        
        for resource_type, resource_usage in usage.items():
            if resource_type not in self.resource_limits[node_name]:
                continue
            
            limit = self.resource_limits[node_name][resource_type]
            current_usage = resource_usage.current_usage
            
            # Check soft limit
            if current_usage > limit.soft_limit:
                print(f"Node {node_name} exceeded soft limit for {resource_type.value}: "
                      f"{current_usage} > {limit.soft_limit}")
                
                # Apply throttling or other soft enforcement
                if resource_type in self.enforcement_callbacks:
                    self.enforcement_callbacks[resource_type](
                        node_name, 'soft_limit_exceeded', current_usage, limit
                    )
            
            # Check hard limit
            if current_usage > limit.hard_limit:
                print(f"Node {node_name} exceeded hard limit for {resource_type.value}: "
                      f"{current_usage} > {limit.hard_limit}")
                
                # Apply hard enforcement (kill, restart, etc.)
                if resource_type in self.enforcement_callbacks:
                    self.enforcement_callbacks[resource_type](
                        node_name, 'hard_limit_exceeded', current_usage, limit
                    )

# TEST: Resource allocation respects limits and policies
# TEST: Resource monitoring accurately tracks usage
# TEST: Limit enforcement triggers appropriate actions
# TEST: Resource pools are managed correctly
```

## Conclusion

The Phase 2 framework core component enhancements provide advanced capabilities for production robotic applications including:

- **Advanced Error Handling**: Pattern detection, correlation analysis, and predictive failure prevention
- **Performance Optimization**: Real-time monitoring, anomaly detection, and optimization recommendations
- **Resource Management**: Intelligent allocation, quota enforcement, and usage optimization
- **Security Enhancements**: Component isolation, secure communication, and data protection
- **Scalability Features**: Horizontal and vertical scaling capabilities with load distribution

These enhancements maintain the core simplicity of the Nevil v3.0 framework while adding enterprise-grade reliability and performance capabilities.

# TEST: All Phase 2 features integrate seamlessly with core framework
# TEST: Performance monitoring provides actionable insights
# TEST: Resource management prevents system overload
# TEST: Security features protect against common threats
# TEST: Scalability enhancements handle increased load gracefully