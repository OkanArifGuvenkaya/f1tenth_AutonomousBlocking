#!/bin/bash
# Setup F1Tenth workspace

echo "======================================"
echo "F1Tenth Workspace Setup"
echo "======================================"

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ Workspace sourced"
else
    echo "⚠️  Workspace not built yet. Run:"
    echo "   colcon build --packages-select sensors opponent_detection"
fi

# Set environment variable for PyTorch
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1

echo "✅ Environment configured"
echo ""
echo "Available packages:"
ros2 pkg list | grep -E "sensors|opponent"
