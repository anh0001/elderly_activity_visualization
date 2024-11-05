#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
from datetime import datetime
import io
from collections import OrderedDict

class ActivityVisualizer(Node):
    def __init__(self):
        super().__init__('activity_visualizer')
        
        # Parameter for number of days to retain - changed to 6
        self.declare_parameter('days_to_retain', 6)
        
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
        
        # Store last data point for each day
        self.daily_data = OrderedDict()  # {date: activity_data}
        self.days_to_retain = self.get_parameter('days_to_retain').value
        
        # Extended color palette for 6 days
        self.colors = ['blue', 'green', 'red', 'purple', 'orange', 'cyan']
        
        self._publish_dummy_image()
        self.get_logger().info('Activity visualizer initialized with 6-day retention')

    def _extract_date(self, timestamp):
        """Extract date from timestamp string."""
        return datetime.fromisoformat(timestamp).date()

    def _update_daily_data(self, timestamp, data):
        """Update the daily data store with latest data point."""
        try:
            date = self._extract_date(timestamp)
            
            # Store data with date as key
            self.daily_data[date] = data
            
            # Sort dictionary by date
            self.daily_data = OrderedDict(sorted(self.daily_data.items()))
            
            # Keep only the last N days
            while len(self.daily_data) > self.days_to_retain:
                self.daily_data.popitem(last=False)
                
            self.get_logger().info(f'Updated data for date {date}, total dates: {len(self.daily_data)}')
            
        except Exception as e:
            self.get_logger().error(f'Error updating daily data: {str(e)}')

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
            
        except Exception as e:
            self.get_logger().error(f'Error publishing dummy image: {str(e)}')
        
    def visualization_callback(self, msg):
        try:
            self.get_logger().info('Received data on processed_activities')
            data = json.loads(msg.data)
            
            if data['type'] != 'action_durations' or not data['data']:
                return
            
            # Update daily data store
            self._update_daily_data(data['timestamp'], data['data'])
            
            if not self.daily_data:
                return
            
            self.get_logger().info(f'Plotting data for dates: {list(self.daily_data.keys())}')
                
            # Create radar chart with more space for legend
            fig = Figure(figsize=(14, 10), facecolor='white', dpi=100)
            ax = fig.add_subplot(111, projection='polar')
            
            # Get common activities across all days
            all_activities = set()
            for day_data in self.daily_data.values():
                all_activities.update(day_data.keys())
            activities = sorted(list(all_activities))
            
            # Compute angles
            num_vars = len(activities)
            if num_vars == 0:
                return
                
            angles = [n / float(num_vars) * 2 * np.pi for n in range(num_vars)]
            angles += angles[:1]
            
            # Plot each day's data
            for (date, day_data), color in zip(self.daily_data.items(), self.colors):
                values = [day_data.get(activity, 0) for activity in activities]
                values += values[:1]
                
                # Plot data with increased line width for better visibility
                ax.plot(angles, values, 'o-', linewidth=2.5, 
                       label=date.strftime('%Y-%m-%d'),
                       color=color, markersize=8)
                ax.fill(angles, values, alpha=0.15, color=color)  # Reduced alpha for less overlap
            
            # Configure chart
            ax.set_theta_offset(np.pi / 2)
            ax.set_theta_direction(-1)
            ax.set_xticks(angles[:-1])
            ax.set_xticklabels(activities, size=10)
            
            # Add title and legend with adjusted position
            ax.set_title("Activities Duration", 
                        pad=20, size=14, weight='bold')
            ax.grid(True, color='gray', alpha=0.3)
            ax.legend(loc='upper right', bbox_to_anchor=(1.4, 1.1),
                     title="Dates", title_fontsize=12)
            
            # Add value annotations for the most recent day
            latest_data = list(self.daily_data.values())[-1]
            latest_values = [latest_data.get(activity, 0) for activity in activities]
            
            for angle, value, activity in zip(angles[:-1], latest_values, activities):
                if value > 0:  # Only annotate non-zero values
                    ax.text(angle, value*1.1, f'{value:.1f}s', 
                           horizontalalignment='center',
                           verticalalignment='center',
                           size=9)
            
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