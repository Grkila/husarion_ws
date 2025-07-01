# ERC 2025 Mars Yard Remote Simulation

This repository contains a complete simulation environment for the European Rover Challenge 2025, featuring the Mars Yard 2024 terrain with Husarion UGV robots.

## 🚀 Quick Start

### Prerequisites

- Docker installed on your system
- Git
- At least 8GB RAM and 4GB free disk space
- GPU support for Gazebo (optional but recommended)

### Option 1: Clone and Build from Source

```bash
# Clone the repository
git clone https://github.com/Grkila/husarion_ws.git
cd husarion_ws

# Switch to the Mars Yard simulation branch
git checkout mars_yard_v1

# Build and run with Docker
docker-compose up --build
```

### Download the Workspace

```bash
# Create workspace directory
mkdir -p /husarion_ws
cd /husarion_ws

# Clone the repository
git clone https://github.com/Grkila/husarion_ws.git .

# Switch to the Mars Yard simulation branch
git checkout mars_yard_v1

# Source ROS 2
source /opt/ros/humble/setup.bash
```

### Install Package Dependencies

```bash
# Install dependencies using rosdep
rosdep install --from-paths src --ignore-src -r -y

# Install additional Python dependencies if needed
pip3 install -r requirements.txt  # if exists
```

### Build the Workspace

```bash
# Build all packages
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash

# Add to bashrc for future sessions
echo "source /husarion_ws/install/setup.bash" >> ~/.bashrc
```

## 🎮 Running the Simulation

### Launch the Complete Simulation

```bash
# Source the workspace
source /husarion_ws/install/setup.bash

# Launch the Mars Yard simulation with robot
ros2 launch erc2025_remote_sim startsimulation.launch.py
```

### Alternative: Launch Components Separately

```bash
# Terminal 1: Launch the Mars Yard world
ros2 launch erc2025_remote_sim sim_world.launch.py

# Terminal 2: Spawn the robot (in a new terminal)
ros2 launch husarion_ugv_gazebo spawn_robot.launch.py \
  robot_model:=panther \
  robot_name:=panther01

# Terminal 3: Launch teleop control
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/panther01/cmd_vel
```

## 🎯 Available Launch Configurations

### World Configurations
- `marsyard2024.world` - Complete Mars Yard 2024 terrain
- Custom robot spawn locations via `start_locations.yaml`

### Robot Models
- `panther` - Husarion Panther UGV
- `lynx` - Husarion Lynx UGV

### Control Options
- Keyboard teleop
- Joystick control (if configured)
- Autonomous navigation (if implemented)

## 📁 Key Files and Directories

```
husarion_ws/
├── src/
│   ├── erc2025_remote_sim/          # Main simulation package
│   │   ├── launch/                  # Launch files
│   │   ├── worlds/                  # Gazebo world files
│   │   ├── models/                  # 3D models and meshes
│   │   ├── config/                  # Configuration files
│   │   └── README.md               # Package-specific docs
│   ├── husarion_ugv_*/             # Husarion UGV packages
│   ├── behaviortree_ros2/          # Behavior Tree support
│   └── ros_components_description/ # ROS component descriptions
├── build/                           # Build artifacts
├── install/                         # Installed packages
├── log/                            # Build and runtime logs
└── README.md                       # This file
```

## 🔧 Configuration

### Robot Spawn Configuration

Edit `src/erc2025_remote_sim/config/start_locations.yaml`:

```yaml
spawn_locations:
  start_1:
    position: [0.0, 0.0, 0.1]
    orientation: [0.0, 0.0, 0.0, 1.0]
  start_2:
    position: [5.0, 5.0, 0.1]
    orientation: [0.0, 0.0, 0.707, 0.707]
```

### Component Configuration

Edit `src/erc2025_remote_sim/config/components.yaml` to configure robot sensors and actuators.

### Teleop Configuration

Edit `src/erc2025_remote_sim/config/teleop.config` for custom control mappings.

## 🐛 Troubleshooting

### Common Issues

1. **Gazebo doesn't start**
   ```bash
   # Check if X11 forwarding works
   xeyes  # Should open a window
   
   # If not, enable X11 forwarding
   xhost +local:docker
   ```

2. **Missing packages**
   ```bash
   # Re-run dependency installation
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build failures**
   ```bash
   # Clean build
   rm -rf build/ install/
   colcon build --symlink-install
   ```

4. **Robot doesn't spawn**
   ```bash
   # Check Gazebo is running
   gz topic -l
   
   # Manually spawn robot
   ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity \
     '{name: "panther01", xml: "$(cat install/husarion_ugv_description/share/husarion_ugv_description/urdf/panther.urdf.xacro)"}'
   ```

### Performance Optimization

```bash
# For better performance, set Gazebo physics parameters
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/husarion_ws/src/erc2025_remote_sim/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/husarion_ws/src/erc2025_remote_sim

# Reduce Gazebo real-time factor if running slow
gz physics -u 0.5  # Run at 50% real-time
```

## 📊 System Requirements

### Minimum Requirements
- 4 CPU cores
- 8GB RAM
- 4GB free disk space
- OpenGL 3.3 support

### Recommended Requirements
- 8+ CPU cores
- 16GB+ RAM
- 10GB+ free disk space
- Dedicated GPU with OpenGL 4.5+
- SSD storage
