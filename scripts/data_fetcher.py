#!/usr/bin/env python3

import requests
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
import json
from datetime import datetime
import os

class DataFetcher(Node):
    def __init__(self):
        super().__init__('data_fetcher')
        
        # Parameters
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
        self.declare_parameter(
            'use_test_data', 
            False,
            ParameterDescriptor(description='Whether to use test data instead of server')
        )
        self.declare_parameter(
            'test_data_path',
            os.path.join(os.path.dirname(__file__), '..', 'test', 'test_data', 'sample_activity_data.json'),
            ParameterDescriptor(description='Path to test data file')
        )
        
        # Initialize publisher for raw activity data
        self.raw_data_publisher = self.create_publisher(
            String,
            'raw_activity_data',
            10
        )
        
        self.processed_ids = set()
        self.test_data_cache = None  # Cache for test data
        
        self.create_timer(
            self.get_parameter('fetch_interval').value,
            self.fetch_data
        )
        
    def load_test_data(self):
        """Load test data from file if not already cached."""
        if self.test_data_cache is None:
            try:
                test_data_path = self.get_parameter('test_data_path').value
                with open(test_data_path, 'r') as f:
                    self.test_data_cache = json.load(f)
                self.get_logger().info(f'Loaded test data from {test_data_path}')
            except Exception as e:
                self.get_logger().error(f'Error loading test data: {str(e)}')
                return None
        return self.test_data_cache
        
    def fetch_data(self):
        """Fetch activity data from server or test data."""
        if self.get_parameter('use_test_data').value:
            self.fetch_test_data()
        else:
            self.fetch_server_data()
            
    def fetch_test_data(self):
        """Fetch data from test data file."""
        try:
            test_data = self.load_test_data()
            if test_data is None:
                return
                
            count = self.get_parameter('fetch_count').value
            new_data = [
                record for record in test_data['data']
                if record['create_dt'] not in self.processed_ids
            ][:count]  # Limit to fetch_count
            
            if new_data:
                self.get_logger().info(f'Found {len(new_data)} new records from test data')
                self.process_new_data(new_data)
                self.processed_ids.update(
                    record['create_dt'] for record in new_data
                )
                
        except Exception as e:
            self.get_logger().error(f'Error processing test data: {str(e)}')
            
    def fetch_server_data(self):
        """Fetch data from server."""
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
                self.get_logger().info(f'Found {len(new_data)} new records from server')
                self.process_new_data(new_data)
                self.processed_ids.update(
                    record['create_dt'] for record in new_data
                )
                
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Network error: {str(e)}')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON parsing error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {str(e)}')
            
    def process_new_data(self, new_data):
        """Process newly fetched data and publish to activity processor."""
        try:
            # Sort data by timestamp to ensure proper processing order
            sorted_data = sorted(
                new_data,
                key=lambda x: datetime.strptime(x['create_dt'], '%Y-%m-%d %H:%M:%S')
            )
            
            # Create message with sorted data
            msg = String()
            msg.data = json.dumps({
                'timestamp': datetime.now().isoformat(),
                'data': sorted_data
            })
            
            # Publish data for processing
            self.raw_data_publisher.publish(msg)
            
            self.get_logger().debug(f'Published {len(sorted_data)} records for processing')
            
        except Exception as e:
            self.get_logger().error(f'Error processing data: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = DataFetcher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()