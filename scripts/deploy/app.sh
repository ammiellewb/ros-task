#!/bin/bash
# app.sh - Main execution script for LIMO simulation and control

# Source ROS
eval "$(pixi shell -e humble)"

# Verify environment
echo "Checking environment..."
which ros2 && which gz

# Build workspace if not built
if [ ! -f "install/setup.bash" ]; then
    echo "Building workspace..."
    colcon build --symlink-install --packages-select limo_control
fi

# Source the workspace
source install/setup.bash

# Launch components with proper display handling
if [ -z "$DISPLAY" ]; then
    echo "No display detected - running headless"
    gz sim -s -v 4 &
    sleep 2
    ros2 launch limo_control limo_complete.launch.py
else
    echo "Launching with GUI..."
    gz sim -s -v 4 &
    sleep 2
    gz sim -g --connect 127.0.0.1 &
    ros2 launch limo_control limo_complete.launch.py
fi

wait