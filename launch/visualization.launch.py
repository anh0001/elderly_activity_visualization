from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('elderly_activity_visualization'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='elderly_activity_visualization',
            executable='data_fetcher',
            name='data_fetcher',
            parameters=[config_file],
            output='screen'
        ),
        Node(
            package='elderly_activity_visualization',
            executable='activity_processor',
            name='activity_processor',
            parameters=[config_file],
            output='screen'
        ),
        Node(
            package='elderly_activity_visualization',
            executable='visualization',
            name='visualization',
            parameters=[config_file],
            output='screen'
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen'
        ),
    ])
