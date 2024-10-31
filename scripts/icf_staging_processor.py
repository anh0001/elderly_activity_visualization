#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

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
        
        # Initialize stage values
        self.stage_values = {}
        
        # Create subscription to raw data
        self.subscription = self.create_subscription(
            String,
            'raw_activity_data',
            self.raw_data_callback,
            1000
        )
        
        # Create QoS profile for sensor-like data
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
            # Parse the incoming JSON data
            data = json.loads(msg.data)
            
            # Process the staging data
            self.process_staging(data['data'])
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error decoding JSON: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error processing data: {str(e)}')
            
    def process_staging(self, data):
        """Process ICF staging data."""
        try:
            if data:  # Process only if there's data
                # Get the most recent record (first in the list)
                latest_record = data[0]
                self._process_record(latest_record)
                
                # Publish processed data
                self._publish_staging_data()
            
        except Exception as e:
            self.get_logger().error(f'Error in process_staging: {str(e)}')

    def _process_record(self, record):
        """Process individual ICF staging record."""
        try:
            # For each defined stage
            for stage_name, field_name in self.stages.items():
                # Get the stage value
                self.stage_values[stage_name] = record[field_name]
                
                self.get_logger().debug(
                    f'Stage {stage_name} value: {self.stage_values[stage_name]}'
                )
                    
        except Exception as e:
            self.get_logger().error(f'Error processing record: {str(e)}')
            
    def _publish_staging_data(self):
        """Publish processed staging data."""
        try:
            # Prepare message
            msg = String()
            msg.data = json.dumps({
                'type': 'icf_staging',
                'timestamp': datetime.now().isoformat(),
                'data': self.stage_values
            })
            
            # Publish message
            self.publisher.publish(msg)
            
            self.get_logger().debug(
                f'Published ICF staging data: {self.stage_values}'
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