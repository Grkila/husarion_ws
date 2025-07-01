import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package directories
    pkg_mars_yard_sim = get_package_share_directory('mars_yard_sim')
    
    # Paths
    mars_yard_world_path = os.path.join(pkg_mars_yard_sim, 'worlds', 'mars_yard.sdf')
    
    # Launch arguments
    robot_model = LaunchConfiguration('robot_model')
    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    log_level = LaunchConfiguration('log_level')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'robot_model', 
            default_value='panther', 
            description='Robot model to spawn (panther, lynx)'
        ),
        DeclareLaunchArgument(
            'robot_name', 
            default_value='', 
            description='Robot name in simulation'
        ),
        DeclareLaunchArgument(
            'x', 
            default_value='2.0', 
            description='Initial X position'
        ),
        DeclareLaunchArgument(
            'y', 
            default_value='2.0', 
            description='Initial Y position'
        ),
        DeclareLaunchArgument(
            'z', 
            default_value='0.2', 
            description='Initial Z position'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='INFO',
            description='Logging level'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='True',
            description='Launch RViz'
        ),
        
        # Include Husarion simulation with Mars yard world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('husarion_ugv_gazebo'),
                    'launch',
                    'simulation.launch.py'
                ])
            ]),
            launch_arguments={
                'gz_world': mars_yard_world_path,
                'robot_model': robot_model,
                'x': x,
                'y': y,
                'z': z,
                'log_level': log_level,
                'use_rviz': use_rviz,
            }.items()
        )
    ])