import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get package directories
    pkg_mars_yard_sim = get_package_share_directory('mars_yard_sim')
    pkg_husarion_ugv_gazebo = get_package_share_directory('husarion_ugv_gazebo')
    
    # Paths
    world_path = os.path.join(pkg_mars_yard_sim, 'worlds', 'mars_yard.sdf')
    model_path = os.path.join(pkg_mars_yard_sim, 'models')
    
    # Launch arguments
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    z_pos = LaunchConfiguration('z_pos')
    robot_name = LaunchConfiguration('robot_name')
    robot_model = LaunchConfiguration('robot_model')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'x_pos', 
            default_value='0.0', 
            description='Initial X position of the robot'
        ),
        DeclareLaunchArgument(
            'y_pos', 
            default_value='0.0', 
            description='Initial Y position of the robot'
        ),
        DeclareLaunchArgument(
            'z_pos', 
            default_value='0.5', 
            description='Initial Z position of the robot'
        ),
        DeclareLaunchArgument(
            'robot_name', 
            default_value='panther', 
            description='Name of the robot in simulation'
        ),
        DeclareLaunchArgument(
            'robot_model', 
            default_value='panther', 
            description='Robot model (panther, lynx, etc.)'
        ),
        
        # Start Gazebo with Mars Yard world
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '4', '-r', world_path],
            output='screen',
            additional_env={
                'GZ_SIM_RESOURCE_PATH': model_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
            }
        ),
        
        # Include Husarion UGV robot spawning
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            pkg_husarion_ugv_gazebo,
                            'launch',
                            'spawn_robot.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'robot_name': robot_name,
                        'robot_model': robot_model,
                        'x': x_pos,
                        'y': y_pos,
                        'z': z_pos,
                        'world_name': 'mars_yard'
                    }.items()
                )
            ]
        )
    ])
