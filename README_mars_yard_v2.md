# Mars Yard Simulation v2 🚀

A comprehensive Mars surface simulation environment for Husarion UGV robots featuring realistic terrain, ArUco markers, and scientific landmarks.

## Features

- **Realistic Mars Terrain**: High-detail Mars surface with proper textures and elevation
- **ArUco Markers**: 15 strategically placed ArUco poles (IDs 17-31) for robot localization
- **Scientific Landmarks**: 6 realistic landmarks for navigation and mission objectives:
  - Astronaut helmet
  - Hammer tool
  - Wrench
  - Satellite dish
  - Sample bottle
  - Drill equipment
- **Full Sensor Integration**: Complete sensor suite including ZED cameras, IMU, and battery monitoring
- **RViz Visualization**: Real-time visualization of robot state and sensor data

## Quick Start

### Prerequisites

- Docker installed on your system
- X11 forwarding enabled (for GUI)

### Run the Simulation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/your-username/husarion_ws.git
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

The simulation will start with:
- Gazebo showing the Mars yard environment
- Panther robot spawned at position (2.0, 2.0, 0.2)
- RViz for real-time visualization
- All sensors and controllers active

## Launch Arguments

Customize your simulation with these arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `robot_model` | `panther` | Robot model (panther, lynx) |
| `x` | `2.0` | Initial X position |
| `y` | `2.0` | Initial Y position |
| `z` | `0.2` | Initial Z position |
| `log_level` | `INFO` | ROS logging level |
| `use_rviz` | `True` | Launch RViz visualization |

Example with custom position:
```bash
ros2 launch mars_yard_sim mars_yard_simulation.launch.py x:=5.0 y:=3.0 robot_model:=lynx
```

## Robot Control

### Teleoperation
Control the robot using keyboard:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Programmatic Control
Publish velocity commands:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"
```

## Available Topics

### Robot Control
- `/cmd_vel` - Velocity commands
- `/odometry/wheels` - Wheel odometry
- `/imu/data` - IMU sensor data

### Camera Feeds
- `/front_cam/zed_node/rgb/image_rect_color` - Front camera RGB
- `/back_cam/zed_node/rgb/image_rect_color` - Back camera RGB
- `/left_cam/zed_node/rgb/image_rect_color` - Left camera RGB
- `/right_cam/zed_node/rgb/image_rect_color` - Right camera RGB

### Depth Data
- `/front_cam/zed_node/depth/points` - Front camera point cloud
- `/back_cam/zed_node/depth/points` - Back camera point cloud
- `/left_cam/zed_node/depth/points` - Left camera point cloud
- `/right_cam/zed_node/depth/points` - Right camera point cloud

## ArUco Marker Locations

The simulation includes 15 ArUco markers positioned around the terrain:

| Marker ID | Position (x, y, z) | Rotation |
|-----------|-------------------|----------|
| 17 | (-5.77, -0.55, 0.0) | 4.500 |
| 18 | (-6.13, 3.60, 0.09) | 0.750 |
| 19 | (-4.07, 12.0, 0.35) | 3.927 |
| 20 | (-4.91, 18.41, 0.20) | 1.166 |
| 21 | (-6.2, 26.0, -0.22) | 5.115 |
| 22 | (-0.5, 26.0, 0.0) | 2.007 |
| 23 | (8.30, 26.74, 0.09) | 4.889 |
| 24 | (11.07, 32.05, 0.09) | 0.123 |
| 25 | (12.13, 22.85, 0.50) | 6.012 |
| 26 | (11.71, 17.21, 0.16) | 3.141 |
| 27 | (11.56, 12.7, 0.23) | 2.501 |
| 28 | (13.72, 7.90, 0.26) | 5.768 |
| 29 | (12.8, 1.94, 0.20) | 0.994 |
| 30 | (8.25, -2.52, -0.10) | 4.317 |
| 31 | (0.75, -2.82, -0.07) | 1.852 |

## Landmark Locations

Scientific equipment and landmarks for mission objectives:

| Landmark | Position (x, y, z) | Description |
|----------|-------------------|-------------|
| Helmet | (10, 5.45, 0.02) | Astronaut helmet |
| Hammer | (4.98, 10.92, 0.1) | Tool for sample collection |
| Wrench | (3.5, 20.04, 0.1) | Maintenance tool |
| Satellite Dish | (-2.55, 9.0, 0.5) | Communication equipment |
| Bottle | (0.5, 20.04, 0.43) | Sample container |
| Drill | (9.9, 16.43, 1.57) | Core sampling drill |

## Troubleshooting

### Common Issues

1. **Gazebo doesn't start**:
   - Ensure X11 forwarding is enabled
   - Check Docker has access to display

2. **Robot doesn't spawn**:
   - Wait for Gazebo to fully load (5-10 seconds)
   - Check that all dependencies are built

3. **No camera feed**:
   - Verify ZED camera bridges are running
   - Check RViz camera display topics

### Performance Optimization

- For headless operation: `use_rviz:=False`
- Reduce Gazebo verbosity: `gz_log_level:=1`
- Limit sensor updates if needed

## Development

### Building from Source

```bash
cd /husarion_ws
colcon build --packages-select mars_yard_sim
source install/setup.bash
```

### Package Structure

```
mars_yard_sim/
├── launch/                    # Launch files
│   └── mars_yard_simulation.launch.py
├── worlds/                    # World definitions
│   └── mars_yard.sdf
├── models/                    # 3D models and assets
│   ├── aruco_pole_textures/   # ArUco marker poles
│   ├── landmarks/             # Scientific equipment
│   ├── mars_yard/            # Terrain model
│   └── panther_robot/        # Robot model
└── package.xml               # Package configuration
```

## Contributing

This simulation is designed for Mars rover research and development. Feel free to:
- Add new landmarks or modify existing ones
- Adjust ArUco marker positions for your research needs
- Enhance terrain features
- Improve sensor configurations

## License

Apache 2.0 License - See LICENSE file for details.

## Acknowledgments

- Built on Husarion UGV ROS 2 framework
- Uses Gazebo simulation environment
- ArUco markers for robot localization research
- Realistic Mars terrain modeling