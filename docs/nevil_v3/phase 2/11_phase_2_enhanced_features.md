# Nevil v3.0 Phase 2 Enhanced Features Design

## Overview

This document defines the design specifications for Phase 2 enhancements to the Nevil v3.0 framework. These features extend the core system with wake word detection, enhanced security, resource management, and message validation while maintaining the fundamental "simple architecture = working robot" philosophy.

## Design Principles for Phase 2

1. **Preserve v1.0 Integration**: All enhancements must not interfere with v1.0 audio pipeline preservation
2. **Minimal Overhead**: New features should add <10% performance overhead
3. **Optional by Default**: Enhanced features can be disabled for simpler deployments
4. **Fail Gracefully**: System continues operating if enhanced features fail to initialize
5. **Configuration-Driven**: All features controlled through .nodes configuration file

## 1. Wake Word Detection System

### 1.1 Architecture Requirements

**Purpose**: Provide always-listening capability with low power consumption while preserving exact v1.0 audio pipeline for speech recognition.

**Design Approach**:
- Parallel wake word detection using lightweight Porcupine engine
- State-based switching between "wake word mode" and "active listening mode"
- Complete independence from v1.0 speech recognition pipeline
- Automatic fallback to continuous listening if wake word fails

### 1.2 System States

```
┌─────────────────┐    Wake Word     ┌──────────────────┐
│   Wake Word     │    Detected      │  Active          │
│   Detection     │ ────────────────▶│  Listening       │
│   (Low Power)   │                  │  (v1.0 Pipeline) │
└─────────────────┘                  └──────────────────┘
         ▲                                      │
         │              Timeout                 │
         └──────────────────────────────────────┘
```

### 1.3 Integration Specifications

**Audio Hardware Sharing**:
- Wake word detector uses same USB microphone as v1.0 (device index 1)
- Audio stream sharing handled through PyAudio device management
- No interference with v1.0 audio initialization sequence

**Message Interface**:
```yaml
# New message types for wake word system
wake_word_detected:
  keyword: string
  confidence: float
  timestamp: float

system_mode:
  mode: string  # "idle", "listening", "speaking", "thinking"
  source: string  # "wake_word", "timeout", "manual"
  timestamp: float
```

**Configuration Integration**:
```yaml
# Addition to .nodes file
nodes:
  speech_recognition:
    environment:
      WAKE_WORD_ENABLED: "true"
      WAKE_WORD_SENSITIVITY: "0.7"
      ACTIVE_LISTENING_TIMEOUT: "10.0"
      PICOVOICE_ACCESS_KEY: "${PICOVOICE_ACCESS_KEY}"
```

### 1.4 Performance Requirements

- Wake word detection latency: <500ms from speech to activation
- CPU overhead: <5% additional load during wake word monitoring
- Memory overhead: <50MB additional RAM usage
- False positive rate: <2% in typical home environment
- Battery impact: Negligible (designed for always-on operation)

## 2. Enhanced Message Validation System

### 2.1 Design Architecture

**Purpose**: Provide runtime validation of messages against schemas defined in .messages files with minimal performance impact.

**Validation Levels**:
- **Development Mode**: Strict validation with detailed error reporting
- **Production Mode**: Warning-only validation to maintain system stability
- **Disabled Mode**: No validation for maximum performance

### 2.2 Schema Integration Strategy

**Schema Loading**:
- Schemas loaded from existing .messages files at startup
- JSON Schema conversion from simple YAML format
- Schema caching for performance optimization

**Validation Pipeline**:
```
Message → Schema Lookup → Validation → Statistics → Publish/Warning
```

**Performance Specifications**:
- Validation time: <1ms per message for typical schemas
- Schema loading: <100ms total at system startup
- Memory overhead: <10MB for all schemas combined
- Error handling: Never blocks message publication in production mode

### 2.3 Configuration Interface

```yaml
# Addition to .nodes file
messaging:
  validation_enabled: true
  strict_validation: false  # true for development
  validation_timeout: 0.001  # 1ms timeout
  cache_schemas: true
```

## 3. Resource Management and Monitoring

### 3.1 Architecture Design

**Purpose**: Prevent system failures through proactive resource monitoring and alerting while maintaining lightweight operation.

**Monitoring Scope**:
- **Process-Level**: Memory, CPU, open files, threads per node
- **System-Level**: Disk usage, swap usage, overall system health
- **Application-Level**: Message queue depths, error rates, response times

### 3.2 Alert System Design

**Alert Types and Thresholds**:
```yaml
resource_alerts:
  memory_high: 85%        # of configured limit
  cpu_high: 80%           # of configured limit
  disk_full: 90%          # of total disk space
  open_files_high: 80%    # of system limit
  message_queue_full: 90% # of queue capacity
```

**Alert Actions**:
- **Logging**: All alerts logged with structured format
- **Callbacks**: Configurable callback functions for custom responses
- **Throttling**: Cooldown periods to prevent alert spam
- **Escalation**: Critical alerts can trigger emergency shutdown

### 3.3 Integration with Node Management

**Resource Limits per Node**:
```yaml
nodes:
  speech_recognition:
    resources:
      max_memory_mb: 200
      max_cpu_percent: 25
      max_threads: 10
      max_open_files: 100
```

**Automatic Actions**:
- Warning logs at 75% of limits
- Process restart at 95% of limits (if configured)
- Emergency shutdown at 100% of critical resources

### 3.4 Performance Requirements

- Monitoring overhead: <1% CPU usage
- Collection interval: 5 seconds (configurable)
- Alert response time: <1 second from threshold breach
- History retention: Last 100 data points per metric

## 4. Enhanced Security Features

### 4.1 Configuration Security Design

**Purpose**: Protect sensitive configuration data (API keys, credentials) through masking, validation, and optional encryption.

**Security Layers**:
1. **Environment Variable Validation**: Ensure required credentials are present
2. **Sensitive Data Masking**: Hide credentials in logs and debug output
3. **Configuration Encryption**: Optional encryption for stored credentials
4. **Access Path Restrictions**: Limit file system access for nodes

### 4.2 Credential Management

**Sensitive Data Identification**:
- Pattern-based detection (keys, tokens, passwords, secrets)
- Configurable sensitive field patterns
- Length-based validation for API keys

**Masking Strategy**:
```
Original: sk-1234567890abcdef1234567890abcdef
Masked:   sk-**************************ef
```

**Environment Variable Security**:
```yaml
security:
  required_env_vars:
    - OPENAI_API_KEY
    - PICOVOICE_ACCESS_KEY
  validate_env_vars: true
  mask_secrets: true
```

### 4.3 Runtime Security

**Path Restrictions**:
```yaml
security:
  allowed_paths:
    - "./nodes/"
    - "./audio/"
    - "./logs/"
    - "/tmp/"
```

**Configuration Validation**:
- Startup validation of all required credentials
- Runtime validation of configuration changes
- Secure logging with automatic credential masking

## 5. Enhanced Configuration Architecture

### 5.1 Backward Compatibility

**v1.0 Parameter Preservation**:
All existing v1.0 parameters maintained exactly:
```yaml
nodes:
  speech_recognition:
    environment:
      # Exact v1.0 preservation
      ENERGY_THRESHOLD: "300"
      PAUSE_THRESHOLD: "0.5"
      DYNAMIC_ENERGY_DAMPING: "0.1"
      DYNAMIC_ENERGY_RATIO: "1.2"
      OPERATION_TIMEOUT: "18"
```

**Configuration Expansion**:
```yaml
# Phase 2 additions (all optional)
system:
  resource_monitoring:
    enabled: true
    monitor_interval: 5.0

security:
  validate_env_vars: true
  mask_secrets: true

messaging:
  validation_enabled: true
  strict_validation: false
```

### 5.2 Hot-Reloading Support

**Configuration Changes**:
- Resource limits can be updated without restart
- Alert thresholds adjustable at runtime
- Validation settings modifiable during operation
- Wake word sensitivity tunable without restart

## 6. System Integration Design

### 6.1 Startup Sequence Enhancement

```
1. Load base configuration (.nodes file)
2. Initialize security manager
3. Validate environment variables
4. Start resource monitoring
5. Initialize message validation
6. Start core nodes (unchanged v1.0 integration)
7. Initialize wake word detection
8. System ready
```

### 6.2 Failure Handling Strategy

**Graceful Degradation**:
- Wake word failure → Fall back to continuous listening
- Resource monitoring failure → Continue without monitoring
- Validation failure → Continue without validation
- Security failure → Log warning, continue with basic security

**Critical Failures**:
- Missing required API keys → Startup failure
- Audio hardware unavailable → Startup failure
- Node restart loop → System shutdown

## 7. Performance and Resource Specifications

### 7.1 Overall System Impact

**CPU Usage**:
- Base system (v1.0 compatibility): 15-25% average
- Phase 2 enhancements: +3-7% additional
- Total target: <35% CPU usage under normal load

**Memory Usage**:
- Base system: 150-200MB
- Phase 2 enhancements: +50-100MB
- Total target: <300MB total RAM usage

**Disk Usage**:
- Enhanced logging: +10-20MB per day
- Configuration caching: <1MB
- Wake word models: 5-10MB

### 7.2 Network and API Impact

**Additional API Dependencies**:
- Picovoice API key required for wake word detection
- No additional network calls during normal operation
- Wake word processing is local-only

**Existing API Usage**:
- No changes to OpenAI API usage patterns
- Preserved v1.0 API call frequency and patterns

## 8. Testing and Validation Strategy

### 8.1 Integration Testing Approach

**v1.0 Compatibility Testing**:
- All existing v1.0 functionality must pass unchanged
- Audio quality regression testing with reference recordings
- Performance baseline validation against v1.0 metrics

**Phase 2 Feature Testing**:
- Wake word detection accuracy in various noise conditions
- Resource monitoring alert trigger validation
- Security masking effectiveness verification
- Message validation performance benchmarking

### 8.2 Success Criteria Definition

**Functional Requirements**:
- [ ] Wake word detection: <2 second activation time, <5% false positive rate
- [ ] Resource monitoring: Prevents system failures, <1% CPU overhead
- [ ] Message validation: Catches 95% of schema violations, <10ms latency
- [ ] Security features: 100% credential masking, no sensitive data in logs

**Performance Requirements**:
- [ ] Total system overhead: <10% additional CPU/memory vs Phase 1
- [ ] v1.0 functionality: No degradation in audio quality or response time
- [ ] System stability: >99% uptime over 24 hours with all features enabled

## 9. Implementation Priority and Phasing

### 9.1 Phase 2A (Core Features - Week 3)
1. Wake word detection system architecture
2. Resource monitoring foundation
3. Basic security enhancements (credential masking)
4. Enhanced configuration loading

### 9.2 Phase 2B (Integration - Week 4)
1. Wake word integration with speech recognition node
2. Message validation system implementation
3. Resource limit enforcement
4. Security validation improvements

### 9.3 Phase 2C (Optimization - Week 5)
1. Performance optimization and tuning
2. Error handling and graceful degradation
3. Configuration hot-reloading
4. End-to-end integration testing

## 10. Configuration Migration Strategy

### 10.1 Backward Compatibility

**Existing .nodes Files**:
- Phase 1 configurations work unchanged
- New features disabled by default
- Gradual feature enablement through configuration

**Migration Path**:
```bash
# Automatic migration script
./scripts/migrate_phase1_to_phase2.sh
```

### 10.2 Feature Adoption Strategy

**Conservative Approach**:
1. Deploy with all Phase 2 features disabled initially
2. Enable resource monitoring first (lowest risk)
3. Enable security features second
4. Enable wake word detection third
5. Enable message validation last (development environments first)

## 11. Deployment and Operations

### 11.1 Production Deployment

**Staged Rollout**:
1. **Development Environment**: All features enabled, strict validation
2. **Testing Environment**: Production configuration, comprehensive testing
3. **Production Environment**: Conservative configuration, monitoring enabled

**Rollback Strategy**:
- Phase 2 features can be disabled via configuration
- Complete rollback to Phase 1 possible without data loss
- Automated rollback triggers for critical failures

### 11.2 Monitoring and Observability

**Enhanced Logging**:
```
[2024-01-15 10:30:00] [INFO] [wake_word] Detection active, sensitivity=0.7
[2024-01-15 10:30:05] [INFO] [resource_monitor] Memory usage: 150MB/200MB (75%)
[2024-01-15 10:30:10] [WARN] [message_validator] Schema validation failed for topic 'voice_command'
```

**Health Checks**:
- Wake word detection status
- Resource usage trends
- Message validation statistics
- Security audit events

## Conclusion

The Phase 2 enhanced features design maintains the core Nevil v3.0 principles while adding essential production capabilities. The modular, optional approach ensures that these enhancements improve reliability and usability without compromising the fundamental "simple architecture = working robot" philosophy.

Key design decisions prioritize backward compatibility, performance efficiency, and operational simplicity. The staged implementation approach allows for careful validation and rollback capabilities, ensuring that enhanced features genuinely improve the system rather than introducing complexity without benefit.

# TEST: All Phase 2 features can be disabled while maintaining full v1.0 compatibility
# TEST: Wake word detection integrates seamlessly without affecting v1.0 audio pipeline
# TEST: Resource monitoring prevents failures while maintaining <1% overhead
# TEST: Security features protect sensitive data without impacting normal operation
# TEST: Message validation improves reliability with minimal performance impact