#!/bin/bash

# Nevil Navigation Monitor Launcher
# This script starts the real-time navigation monitoring dashboard

echo "🤖 Starting Nevil Navigation Monitor..."
echo "========================================="

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 not sourced. Please run:"
    echo "   source /opt/ros/humble/setup.bash"
    echo "   source install/setup.bash"
    exit 1
fi

# Default options
LOG_FILE=""
REFRESH_RATE=100
HELP=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -l|--log-file)
            LOG_FILE="$2"
            shift 2
            ;;
        -r|--refresh-rate)
            REFRESH_RATE="$2"
            shift 2
            ;;
        -h|--help)
            HELP=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            HELP=true
            shift
            ;;
    esac
done

# Show help
if [ "$HELP" = true ]; then
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -l, --log-file FILE     Log messages to file for analysis"
    echo "  -r, --refresh-rate MS   Display refresh rate in milliseconds (default: 100)"
    echo "  -h, --help             Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Basic monitoring"
    echo "  $0 -l /tmp/nav_monitor.log           # With logging"
    echo "  $0 -r 50 -l /tmp/nav_monitor.log     # Fast refresh with logging"
    echo ""
    echo "Controls during monitoring:"
    echo "  Ctrl+C                 Clean shutdown"
    echo "  r                      Reset message counters"
    echo "  p                      Pause/resume monitoring"
    echo ""
    echo "Critical Features:"
    echo "  🚨 Fatal error detection and alerting"
    echo "  🏥 Node health monitoring"
    echo "  📊 System resource monitoring"
    echo "  💀 Process failure detection"
    exit 0
fi

# Build command
CMD="python3 $(dirname "$0")/navigation_monitor.py"

if [ -n "$LOG_FILE" ]; then
    CMD="$CMD --log-file $LOG_FILE"
    echo "📝 Logging to: $LOG_FILE"
fi

if [ "$REFRESH_RATE" != "100" ]; then
    CMD="$CMD --refresh-rate $REFRESH_RATE"
    echo "⚡ Refresh rate: ${REFRESH_RATE}ms"
fi

echo "🚀 Starting monitor..."
echo "📡 Monitoring topics: /cmd_vel, /goal_pose, /system_mode, /nevil/action_command, /planned_path"
echo "🚨 Critical error detection: ENABLED"
echo "🏥 Node health monitoring: ENABLED"
echo ""

# Check if rich library is available
python3 -c "import rich" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✨ Rich terminal formatting: ENABLED"
else
    echo "⚠️  Rich library not found - using basic terminal output"
    echo "   Install with: pip3 install rich"
fi

echo ""
echo "Press Ctrl+C to stop monitoring"
echo "========================================="

# Run the monitor
exec $CMD