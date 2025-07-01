#!/bin/bash

# Mars Yard v2 Simulation Startup Script
# This script automatically builds and launches the Mars Yard simulation

set -e

echo "🚀 Starting Mars Yard v2 Simulation..."
echo "=================================="

# Check if we're in the right directory
if [ ! -f "package.xml" ]; then
    echo "❌ Error: Please run this script from the mars_yard_sim package directory"
    echo "   Expected location: /husarion_ws/src/mars_yard_sim/"
    exit 1
fi

# Source ROS environment
echo "📦 Sourcing ROS environment..."
source /opt/ros/jazzy/setup.bash

# Build the package
echo "🔨 Building mars_yard_sim package..."
cd /husarion_ws
colcon build --packages-select mars_yard_sim

# Source the workspace
echo "📡 Sourcing workspace..."
source install/setup.bash

# Launch the simulation
echo "🌌 Launching Mars Yard simulation..."
echo "   - Gazebo will show the Mars yard environment"
echo "   - Panther robot will spawn at (2.0, 2.0, 0.2)"
echo "   - RViz will display sensor data"
echo "   - All controllers will be active"
echo ""
echo "🎮 Control the robot with:"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""

ros2 launch mars_yard_sim mars_yard_simulation.launch.py