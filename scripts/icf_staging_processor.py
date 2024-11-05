#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from collections import defaultdict

class ICFStagingProcessor(Node):
    def __init__(self):
        super().__init__('icf_staging_processor')
        
        # Define ICF stages to track
        self.stages = {
            'transportation': 'transportation_stage',
            'grooming': 'grooming_stage',
            'mouth_care': 'mouth_care_stage',
            'basic': 'basic_stage',
            'dressing': 'dressing_stage'
        }
        
        # Store daily stage values
        self.daily_stage_values = defaultdict(dict)
        
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

        # Publisher for processed staging data
        self.publisher = self.create_publisher(
            String, 
            'processed_icf_stages',
            qos_profile
        )
        
        self.get_logger().info('ICF staging processor initialized')
        
    def raw_data_callback(self, msg):
        """Handle incoming raw activity data."""
        try:
            data = json.loads(msg.data)
            self.process_staging(data['data'])
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error decoding JSON: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error processing data: {str(e)}')
            
    def process_staging(self, data):
        """Process ICF staging data."""
        try:
            for record in data:
                date = record['create_dt'].split()[0]
                self._process_record(record, date)
                
            # Publish processed data for each date
            for date, stages in self.daily_stage_values.items():
                self._publish_staging_data(date, stages)
            
        except Exception as e:
            self.get_logger().error(f'Error in process_staging: {str(e)}')

    def _process_record(self, record, date):
        """Process individual ICF staging record."""
        try:
            current_stages = {}
            for stage_name, field_name in self.stages.items():
                current_stages[field_name] = record[field_name]
            
            # Update the daily values
            self.daily_stage_values[date] = current_stages
            
            self.get_logger().debug(
                f'Updated stages for date {date}: {current_stages}'
            )
                    
        except Exception as e:
            self.get_logger().error(f'Error processing record: {str(e)}')
            
    def _publish_staging_data(self, date, stages):
        """Publish processed staging data for a specific date."""
        try:
            # Create timestamp at end of the day
            timestamp = f"{date} 23:59:59"
            
            # Prepare message
            msg = String()
            msg.data = json.dumps({
                'type': 'icf_staging',
                'timestamp': timestamp,
                'data': {
                    'create_dt': timestamp,
                    **stages
                }
            })
            
            # Publish message
            self.publisher.publish(msg)
            
            self.get_logger().debug(
                f'Published ICF staging data for {date}: {stages}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error publishing staging data: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ICFStagingProcessor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()