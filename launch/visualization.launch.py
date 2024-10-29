from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('elderly_activity_visualization')
    config_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'params_file',
            default_value=config_file,
            description='Full path to params file'
        ),
        
        # Data Fetcher Node
        Node(
            package='elderly_activity_visualization',
            executable='data_fetcher',
            name='data_fetcher',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        ),
        
        # Activity Processor Node
        Node(
            package='elderly_activity_visualization',
            executable='activity_processor',
            name='activity_processor',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        ),
        
        # Visualization Node
        Node(
            package='elderly_activity_visualization',
            executable='visualization',
            name='visualization',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        ),
        
        # ROS Bridge Server
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        )
    ])
