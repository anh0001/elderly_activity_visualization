#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
from sensor_msgs.msg import CompressedImage
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
import io
from datetime import datetime
from collections import defaultdict

class ActivityVisualizer(Node):
    def __init__(self):
        super().__init__('activity_visualizer')
        
        # Configure number of days to show in history
        self.HISTORY_DAYS = 6
        
        # Store the last data point for each day
        self.daily_data = {}
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            String,
            '/processed_activities',
            self.visualization_callback,
            qos_profile
        )

        self.image_publisher = self.create_publisher(
            CompressedImage,
            '/activity_visualization/compressed',
            qos_profile
        )
        
        self._publish_dummy_image()
        self.get_logger().info('Activity visualizer initialized')

    def _fig_to_jpeg_bytes(self, fig):
        """Convert matplotlib figure to JPEG bytes."""
        buf = io.BytesIO()
        fig.savefig(buf, format='jpg', 
                   bbox_inches='tight', 
                   facecolor='white',
                   dpi=100,
                   pad_inches=0.5)
        buf.seek(0)
        return buf.getvalue()
        
    def _publish_dummy_image(self):
        try:
            fig = Figure(figsize=(8, 8), facecolor='white')
            ax = fig.add_subplot(111)
            ax.text(0.5, 0.5, 'Initializing Activity Visualization...', 
                   horizontalalignment='center',
                   verticalalignment='center',
                   transform=ax.transAxes,
                   fontsize=14)
            ax.axis('off')
            
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.format = "jpeg"
            msg.data = self._fig_to_jpeg_bytes(fig)
            
            self.image_publisher.publish(msg)
            plt.close(fig)
            self.get_logger().info('Published dummy image')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing dummy image: {str(e)}')
            
    def _update_daily_data(self, data, timestamp):
        """Update the stored data for the day."""
        date = timestamp.split('T')[0]
        self.daily_data[date] = data
        
        # Keep only the most recent HISTORY_DAYS
        dates = sorted(self.daily_data.keys())
        if len(dates) > self.HISTORY_DAYS:
            for old_date in dates[:-self.HISTORY_DAYS]:
                del self.daily_data[old_date]
        
    def visualization_callback(self, msg):
        try:
            self.get_logger().info('Received data on processed_activities')
            data = json.loads(msg.data)
            
            if data['type'] != 'action_durations' or not data['data']:
                return
                
            self.get_logger().info(f'Received activity data: {data["data"]}')
            
            # Update daily data
            self._update_daily_data(data['data'], data['timestamp'])
            
            # Create radar chart
            activities = list(data['data'].keys())
            num_vars = len(activities)
            
            if num_vars == 0:
                return
            
            # Compute angles
            angles = [n / float(num_vars) * 2 * np.pi for n in range(num_vars)]
            angles += angles[:1]
            
            # Create figure
            fig = Figure(figsize=(12, 8), facecolor='white', dpi=100)
            ax = fig.add_subplot(111, projection='polar')
            
            # Colors for different days
            colors = ['blue', 'green', 'red']
            
            # Plot data for each day
            sorted_dates = sorted(self.daily_data.keys())
            for i, date in enumerate(sorted_dates):
                values = [self.daily_data[date][activity] for activity in activities]
                values += values[:1]  # Complete the circle
                
                label = f'Date: {date}'
                ax.plot(angles, values, 'o-', linewidth=2, 
                       color=colors[i], markersize=8,
                       label=label, alpha=0.7)
                ax.fill(angles, values, color=colors[i], alpha=0.1)
            
            # Configure chart
            ax.set_theta_offset(np.pi / 2)
            ax.set_theta_direction(-1)
            ax.set_xticks(angles[:-1])
            ax.set_xticklabels(activities, size=10)
            
            # Set title with date range
            if sorted_dates:
                title = f"Activities Duration\n{sorted_dates[0]} to {sorted_dates[-1]}"
            else:
                title = "Activities Duration"
            ax.set_title(title, pad=20, size=14, weight='bold')
            
            # Add legend
            ax.legend(loc='center left', bbox_to_anchor=(1.2, 0.5))
            
            ax.grid(True, color='gray', alpha=0.3)
            
            # Create message
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.format = "jpeg"
            msg.data = self._fig_to_jpeg_bytes(fig)
            
            # Publish and cleanup
            self.image_publisher.publish(msg)
            plt.close(fig)
            self.get_logger().info('Successfully published radar chart')
            
        except Exception as e:
            self.get_logger().error(f'Error in visualization: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ActivityVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()