#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from collections import defaultdict
from datetime import datetime
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ActivityProcessor(Node):
    def __init__(self):
        super().__init__('activity_processor')
        
        # Initialize action durations tracking
        self.action_durations = defaultdict(float)
        
        # Create subscription to raw data
        self.subscription = self.create_subscription(
            String,
            'raw_activity_data',
            self.raw_data_callback,
            1000
        )
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher for processed data
        self.publisher = self.create_publisher(
            String, 
            'processed_activities',
            qos_profile
        )
        
        self.get_logger().info('Activity processor initialized')
        
    def raw_data_callback(self, msg):
        """Handle incoming raw activity data."""
        try:
            # Parse the incoming JSON data
            data = json.loads(msg.data)
            
            # Process the activities
            self.process_activities(data['data'])
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error decoding JSON: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error processing data: {str(e)}')
            
    def process_activities(self, data):
        """Process activity data and calculate durations."""
        try:
            for record in data:
                self._process_record(record)
                
            # Publish processed data
            self._publish_durations()
            
        except Exception as e:
            self.get_logger().error(f'Error in process_activities: {str(e)}')

    def _process_record(self, record):
        """Process individual activity record."""
        try:
            # Extract action and duration
            action = record['action']
            duration = float(record['duration'])
            
            # Add duration to the appropriate action
            self.action_durations[action] += duration
            
            self.get_logger().debug(
                f'Action {action} updated: {self.action_durations[action]}'
            )
                    
        except Exception as e:
            self.get_logger().error(f'Error processing record: {str(e)}')
            
    def _publish_durations(self):
        """Publish processed durations as chart data."""
        try:
            # Prepare message
            msg = String()
            msg.data = json.dumps({
                'type': 'action_durations',
                'timestamp': datetime.now().isoformat(),
                'data': dict(self.action_durations)
            })
            
            # Publish message
            self.publisher.publish(msg)
            
            self.get_logger().debug(
                f'Published action durations: {dict(self.action_durations)}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error publishing durations: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ActivityProcessor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()