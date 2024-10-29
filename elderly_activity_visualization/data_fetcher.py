#!/usr/bin/env python3

import requests
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

class DataFetcher(Node):
    def __init__(self):
        super().__init__('data_fetcher')
        
        self.declare_parameter(
            'server_url', 
            'http://192.168.1.109/ncgg_icf_stage.php',
            ParameterDescriptor(description='Server URL for fetching activity data')
        )
        self.declare_parameter(
            'fetch_count', 
            100,
            ParameterDescriptor(description='Number of records to fetch per request')
        )
        self.declare_parameter(
            'fetch_interval', 
            1.0,
            ParameterDescriptor(description='Data fetch interval in seconds')
        )
        
        self.processed_ids = set()
        self.create_timer(
            self.get_parameter('fetch_interval').value,
            self.fetch_data
        )
        
    def fetch_data(self):
        """Fetch activity data from server."""
        try:
            url = self.get_parameter('server_url').value
            count = self.get_parameter('fetch_count').value
            
            response = requests.get(f"{url}?count={count}")
            response.raise_for_status()
            
            data = response.json()
            new_data = [
                record for record in data['data']
                if record['create_dt'] not in self.processed_ids
            ]
            
            if new_data:
                self.process_new_data(new_data)
                self.processed_ids.update(
                    record['create_dt'] for record in new_data
                )
                
        except Exception as e:
            self.get_logger().error(f'Error fetching data: {str(e)}')
            
    def process_new_data(self, new_data):
        """Process newly fetched data."""
        # Implementation in activity_processor.py
        pass

def main(args=None):
    rclpy.init(args=args)
    node = DataFetcher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()