from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'server_url',
            default_value='http://192.168.1.109/ncgg_icf_stage.php',
            description='Server URL for fetching activity data'
        ),
        
        DeclareLaunchArgument(
            'fetch_count',
            default_value='100',
            description='Number of records to fetch per request'
        ),
        
        Node(
            package='elderly_activity_visualization',
            executable='data_fetcher',
            name='data_fetcher',
            parameters=[{
                'server_url': LaunchConfiguration('server_url'),
                'fetch_count': LaunchConfiguration('fetch_count'),
                'fetch_interval': 1.0
            }]
        ),
        
        Node(
            package='elderly_activity_visualization',
            executable='activity_processor',
            name='activity_processor'
        ),
        
        Node(
            package='elderly_activity_visualization',
            executable='visualization',
            name='visualization'
        ),
        
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket'
        )
    ])