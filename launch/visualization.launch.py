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
            executable='logo_publisher.py',
            name='logo_publisher',
            output='screen'
        ),

        # Data Fetcher
        Node(
            package='elderly_activity_visualization',
            executable='data_fetcher',
            name='data_fetcher',
            parameters=[config_file],
            output='screen'
        ),

        # Activity Duration
        Node(
            package='elderly_activity_visualization',
            executable='activity_processor',
            name='activity_processor',
            parameters=[config_file],
            output='screen'
        ),
        Node(
            package='elderly_activity_visualization',
            executable='activity_visualizer',
            name='activity_visualizer',
            parameters=[config_file],
            output='screen',
            # prefix='bash -c "sleep 2.0 && $0 $@"'
        ),

        # # ICF Staging
        # Node(
        #     package='elderly_activity_visualization',
        #     executable='icf_staging_processor',
        #     name='icf_staging_processor',
        #     parameters=[config_file],
        #     output='screen'
        # ),
        # Node(
        #     package='elderly_activity_visualization',
        #     executable='icf_staging_visualizer',
        #     name='icf_staging_visualizer',
        #     parameters=[config_file],
        #     output='screen'
        # ),

        # Add rosapi node
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi_node',
            output='screen'
        ),

        # Configure rosbridge with proper parameters
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
                'address': '0.0.0.0',
                'retry_startup_delay': 5.0,
                'fragment_timeout': 600,
                'delay_between_messages': 0,
                'max_message_size': 10000000,
                'unregister_timeout': 10.0
            }],
            output='screen'
        ),
    ])