#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from collections import defaultdict

class ActivityProcessor(Node):
    def __init__(self):
        super().__init__('activity_processor')
        
        self.categories = {
            'transportation': 'transportation_stage',
            'grooming': 'grooming_stage',
            'mouth_care': 'mouth_care_stage',
            'basic': 'basic_stage',
            'dressing': 'dressing_stage'
        }
        
        self.category_durations = defaultdict(float)
        self.previous_values = {}
        
        # Publisher for processed data
        self.publisher = self.create_publisher(
            String, 
            'processed_activities',
            10
        )
        
    def process_activities(self, data):
        """Process activity data and calculate durations."""
        for record in sorted(data, key=lambda x: x['create_dt']):
            self._process_record(record)
            
        # Publish processed data
        self._publish_durations()
        
    def _process_record(self, record):
        """Process individual activity record."""
        for category, field in self.categories.items():
            current_value = record[field]
            
            if category not in self.previous_values:
                self.previous_values[category] = current_value
                continue
                
            if current_value != self.previous_values[category]:
                # Category value changed, add duration
                self.category_durations[category] += float(record['duration'])
                self.previous_values[category] = current_value
                
    def _publish_durations(self):
        """Publish processed durations as radar chart data."""
        msg = String()
        msg.data = json.dumps({
            'type': 'radar_chart',
            'data': dict(self.category_durations)
        })
        self.publisher.publish(msg)