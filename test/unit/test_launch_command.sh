#!/bin/bash

echo "Testing Nevil launch command..."
echo "Command: ./nevil launch nevil_bringup physical_robot.launch.py"
echo ""

# Test if the command can parse the launch file
echo "1. Testing launch file parsing..."
./nevil launch nevil_bringup physical_robot.launch.py --print > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "   ✅ Launch file parsing: SUCCESS"
else
    echo "   ❌ Launch file parsing: FAILED"
    exit 1
fi

# Test if the command can show arguments
echo "2. Testing launch arguments..."
./nevil launch nevil_bringup physical_robot.launch.py --show-args > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "   ✅ Launch arguments: SUCCESS"
else
    echo "   ❌ Launch arguments: FAILED"
    exit 1
fi

echo ""
echo "🎉 Launch command verification complete!"
echo "The command './nevil launch nevil_bringup physical_robot.launch.py' is ready to use."
echo ""
echo "Note: When running on actual hardware, ensure:"
echo "- Raspberry Pi with GPIO access"
echo "- PiCar-X hardware connected"
echo "- Required hardware libraries installed"