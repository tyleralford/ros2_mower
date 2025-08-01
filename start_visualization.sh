#!/bin/bash

# RViz2 Launcher Script - Workaround for VSCode Snap Package Conflicts
# This script resolves the snap/core20 library conflicts that prevent RViz2 from running

echo "Starting ROS 2 Mower Visualization..."
echo "Note: Ignoring 'Failed to load module canberra-gtk-module' warnings - these are harmless"

# Remove snap-related environment variables that cause conflicts
unset GTK_PATH
unset GIO_MODULE_DIR

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Source local package if built
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "Using locally built ros2_mower package"
    ros2 launch ros2_mower view_robot.launch.py
else
    echo "Package not built yet. Building first..."
    colcon build
    source install/setup.bash
    ros2 launch ros2_mower view_robot.launch.py
fi
