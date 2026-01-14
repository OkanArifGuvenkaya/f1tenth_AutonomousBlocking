#!/bin/bash
# Build and test opponent detection system

echo "======================================"
echo "F1Tenth Build & Test Script"
echo "======================================"

# Navigate to workspace
cd "$(dirname "$0")/.."

# Clean previous builds
echo "Cleaning previous builds..."
rm -rf build/ install/ log/

# Build packages
echo "Building packages..."
colcon build --packages-select sensors opponent_detection

# Source workspace
echo "Sourcing workspace..."
source install/setup.bash

# Check if built successfully
if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    echo ""
    echo "Run the system with:"
    echo "  ros2 launch full_system.launch.py"
    echo ""
    echo "Or individually:"
    echo "  Terminal 1: ros2 launch sensors camera.launch.py"
    echo "  Terminal 2: ros2 launch opponent_detection opponent_detector.launch.py"
else
    echo "❌ Build failed!"
    exit 1
fi
