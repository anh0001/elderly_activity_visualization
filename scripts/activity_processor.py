#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from collections import defaultdict
from datetime import datetime
import random

class ActivityProcessor(Node):
    def __init__(self):
        super().__init__('activity_processor')
        
        # Define activity categories and their corresponding fields
        self.categories = {
            'transportation': 'transportation_stage',
            'grooming': 'grooming_stage',
            'mouth_care': 'mouth_care_stage',
            'basic': 'basic_stage',
            'dressing': 'dressing_stage'
        }
        
        # Initialize tracking variables
        self.category_durations = defaultdict(float)
        self.previous_values = {}
        self.last_timestamp = None
        
        # Create subscription to raw data
        self.subscription = self.create_subscription(
            String,
            'raw_activity_data',
            self.raw_data_callback,
            10
        )
        
        # Publisher for processed data
        self.publisher = self.create_publisher(
            String, 
            'processed_activities',
            10
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

    #
    # TODO: For testing only with random condition to check the visualization
    def _process_record(self, record):
        """Process individual activity record with random selection."""
        try:
            timestamp = datetime.strptime(record['create_dt'], '%Y-%m-%d %H:%M:%S')
            
            for category, field in self.categories.items():
                current_value = record[field]
                
                # Initialize if first record
                if category not in self.previous_values:
                    self.previous_values[category] = current_value
                    continue
                
                # Randomly decide whether to add duration (30% chance)
                if random.random() < 0.3:  # Adjust this probability as needed
                    # Add duration to the appropriate category
                    duration = float(record['duration'])
                    # Add some random variation to the duration
                    adjusted_duration = duration * random.uniform(0.5, 2.0)
                    self.category_durations[category] += adjusted_duration
                    
                    self.get_logger().debug(
                        f'Category {category} updated: {self.category_durations[category]}'
                    )
                    
                self.previous_values[category] = current_value
                    
        except Exception as e:
            self.get_logger().error(f'Error processing record: {str(e)}')

    # def _process_record(self, record):
    #     """Process individual activity record."""
    #     try:
    #         timestamp = datetime.strptime(record['create_dt'], '%Y-%m-%d %H:%M:%S')
            
    #         for category, field in self.categories.items():
    #             current_value = record[field]
                
    #             # Initialize if first record
    #             if category not in self.previous_values:
    #                 self.previous_values[category] = current_value
    #                 continue
                
    #             # Check if value changed
    #             if current_value != self.previous_values[category]:
    #                 # Add duration to the appropriate category
    #                 self.category_durations[category] += float(record['duration'])
    #                 self.previous_values[category] = current_value
                    
    #                 self.get_logger().debug(
    #                     f'Category {category} updated: {self.category_durations[category]}'
    #                 )
                    
    #     except Exception as e:
    #         self.get_logger().error(f'Error processing record: {str(e)}')
            
    def _publish_durations(self):
        """Publish processed durations as radar chart data."""
        try:
            # Prepare message
            msg = String()
            msg.data = json.dumps({
                'type': 'radar_chart',
                'timestamp': datetime.now().isoformat(),
                'data': dict(self.category_durations)
            })
            
            # Publish message
            self.publisher.publish(msg)
            
            self.get_logger().debug(
                f'Published durations: {dict(self.category_durations)}'
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