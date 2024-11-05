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
        
        # Initialize action durations tracking per day - will accumulate over time
        self.daily_action_durations = defaultdict(lambda: defaultdict(float))
        self.all_activities = set()  # Track all possible activities
        
        # Create subscription to raw data
        self.subscription = self.create_subscription(
            String,
            'raw_activity_data',
            self.raw_data_callback,
            100
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
        """Process activity data and accumulate durations per day."""
        try:
            # Accumulate durations for each activity per day
            for record in data:
                date = datetime.strptime(record['create_dt'], '%Y-%m-%d %H:%M:%S').date()
                action = record['action']
                duration = float(record['duration'])
                
                self.all_activities.add(action)
                self.daily_action_durations[date][action] += duration
            
            # Ensure all days have all activities
            for date in self.daily_action_durations.keys():
                for action in self.all_activities:
                    if action not in self.daily_action_durations[date]:
                        self.daily_action_durations[date][action] = 0.0
            
            # Publish updated data for each day
            for date, actions in self.daily_action_durations.items():
                self._publish_day_data(date, actions)
                
            self.get_logger().info(
                f'Processed data for {len(data)} records, '
                f'Total days: {len(self.daily_action_durations)}, '
                f'Total activities: {len(self.all_activities)}'
            )
                
        except Exception as e:
            self.get_logger().error(f'Error in process_activities: {str(e)}')

    def _publish_day_data(self, date, actions):
        """Publish processed durations for a specific day."""
        try:
            # Create timestamp for the end of the day
            timestamp = datetime.combine(date, datetime.max.time()).isoformat()
            
            # Prepare message with all activities
            msg = String()
            msg.data = json.dumps({
                'type': 'action_durations',
                'timestamp': timestamp,
                'data': dict(actions)
            })
            
            # Publish message
            self.publisher.publish(msg)
            
            self.get_logger().debug(
                f'Published action durations for {date}: {dict(actions)}'
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