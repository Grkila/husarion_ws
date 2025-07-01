# Mars Yard Simulation v2 🚀

A comprehensive Mars surface simulation environment for Husarion UGV robots featuring realistic terrain, ArUco markers, and scientific landmarks.

## Features

- **Realistic Mars Terrain**: High-detail Mars surface with proper textures and elevation
- **ArUco Markers**: 15 strategically placed ArUco poles (IDs 17-31) for robot localization
- **Scientific Landmarks**: 6 realistic landmarks for navigation and mission objectives:
  - Astronaut helmet, Hammer tool, Wrench, Satellite dish, Sample bottle, Drill equipment
- **Full Sensor Integration**: Complete sensor suite including ZED cameras, IMU, and battery monitoring
- **RViz Visualization**: Real-time visualization of robot state and sensor data

## Quick Start

### Prerequisites
- Docker installed on your system
- X11 forwarding enabled (for GUI)

### Setup
1. **Clone the repository**:
   ```bash
   git clone https://github.com/Grkila/husarion_ws.git
   cd husarion_ws
   git checkout mars_yard_v2
   ```

2. **Start the Docker container**:
   ```bash
   docker compose up
   ```

3. **Launch the Mars Yard simulation**:
   ```bash
   # Inside the container
   source /husarion_ws/install/setup.bash
   ros2 launch mars_yard_sim mars_yard_simulation.launch.py
   ```

## Asset Management

Due to GitHub's file size limitations, large texture files are managed separately:

### Included by Default
- ✅ 3D terrain mesh (`model.obj` - 32MB)
- ✅ Heightmap data (`mars_yard_depth.tif` - 8MB)  
- ✅ ArUco marker textures (all small files)
- ✅ Placeholder Mars texture (lightweight version)

### Optional High-Resolution Assets
- 🔄 Full Mars surface texture (129MB) - contact repository maintainer
- The simulation works perfectly with the included lightweight textures
- For research requiring high-fidelity visuals, use custom textures

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `robot_model` | `panther` | Robot model (panther, lynx) |
| `x` | `2.0` | Initial X position |
| `y` | `2.0` | Initial Y position |
| `z` | `0.2` | Initial Z position |
| `log_level` | `INFO` | ROS logging level |
| `use_rviz` | `True` | Launch RViz visualization |

## Robot Control

### Teleoperation
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Programmatic Control
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"
```

## ArUco Marker Locations

The simulation includes 15 ArUco markers positioned around the terrain (IDs 17-31) for robot localization and navigation testing.

## Landmark Locations

Scientific equipment and landmarks for mission objectives:
- Helmet (10, 5.45, 0.02) - Astronaut helmet
- Hammer (4.98, 10.92, 0.1) - Tool for sample collection
- Wrench (3.5, 20.04, 0.1) - Maintenance tool
- Satellite Dish (-2.55, 9.0, 0.5) - Communication equipment
- Bottle (0.5, 20.04, 0.43) - Sample container
- Drill (9.9, 16.43, 1.57) - Core sampling drill

## Troubleshooting

### Common Issues
1. **Gazebo doesn't start**: Ensure X11 forwarding is enabled
2. **Robot doesn't spawn**: Wait for Gazebo to fully load (5-10 seconds)
3. **No camera feed**: Verify ZED camera bridges are running
4. **Low-quality textures**: This is normal for the GitHub-compatible version

## Development

### Building from Source
```bash
cd /husarion_ws
colcon build --packages-select mars_yard_sim
source install/setup.bash
```

## Version Information

**Mars Yard v2** features:
- Improved asset management for GitHub compatibility
- Comprehensive Mars surface simulation
- Full integration with Husarion UGV ecosystem
- Professional documentation and setup scripts

## License

Apache 2.0 License

## Acknowledgments

- Built on Husarion UGV ROS 2 framework
- Uses Gazebo simulation environment
- ArUco markers for robot localization research