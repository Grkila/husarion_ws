import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the Mars Yard world path
    pkg_husarion_gz_worlds = get_package_share_directory('husarion_gz_worlds')
    world_path = os.path.join(pkg_husarion_gz_worlds, 'worlds', 'mars_yard.sdf')
    
    # Set up model paths
    models_path = '/root/mars_yard_ws/models'
    husarion_models_path = os.path.join(pkg_husarion_gz_worlds, 'models')
    
    # Robot description from URDF
    urdf_file = '/root/mars_yard_ws/models/panther_robot/panther.urdf'
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Launch arguments
    x_pos = LaunchConfiguration('x_pos', default='0.0')
    y_pos = LaunchConfiguration('y_pos', default='0.0')
    z_pos = LaunchConfiguration('z_pos', default='0.5')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('x_pos', default_value='0.0', description='X position'),
        DeclareLaunchArgument('y_pos', default_value='0.0', description='Y position'), 
        DeclareLaunchArgument('z_pos', default_value='0.5', description='Z position'),

        # Set environment variables for Gazebo
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', 
                             f'{models_path}:{husarion_models_path}'),

        # Start Gazebo with Mars Yard world
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '4', '-r', world_path, '--gui-config', '/dev/null'],
            output='screen',
            additional_env={'DISPLAY': ':0', 'XDG_RUNTIME_DIR': '/tmp/runtime-root'}
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Wait for Gazebo to start, then spawn robot
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-topic', '/robot_description',
                        '-name', 'panther',
                        '-x', x_pos,
                        '-y', y_pos,
                        '-z', z_pos
                    ],
                    output='screen'
                )
            ]
        ),

        # Start ROS-Gazebo bridge for robot control  
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    arguments=[
                        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                        '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                        '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                        '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                        '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
                    ],
                    output='screen'
                )
            ]
        )
    ])