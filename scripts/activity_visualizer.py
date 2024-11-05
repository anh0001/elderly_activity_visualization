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
        
        # Track current date being processed
        self.current_date = None
        
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
            
        except Exception as e:
            self.get_logger().error(f'Error publishing dummy image: {str(e)}')
            
    def _update_daily_data(self, record):
        """Update the stored data for the day."""
        # Extract date from create_dt
        date = record['create_dt'].split()[0]
        
        # If it's a new date or first record, update current_date
        if self.current_date != date:
            self.current_date = date
            
        # Update activity data for this date
        if date in self.daily_data:
            self.daily_data[date]['duration'] = float(record['duration'])
            self.daily_data[date]['action'] = record['action']
        else:
            self.daily_data[date] = {
                'duration': float(record['duration']),
                'action': record['action']
            }
        
        # Keep only the most recent HISTORY_DAYS
        dates = sorted(self.daily_data.keys())
        if len(dates) > self.HISTORY_DAYS:
            for old_date in dates[:-self.HISTORY_DAYS]:
                del self.daily_data[old_date]
                
    def visualization_callback(self, msg):
        try:
            data = json.loads(msg.data)
            
            if not data.get('data'):
                return
                
            # Update activity data for the record
            self._update_daily_data(data['data'])
            
            # Get unique actions and create activity data structure
            all_actions = sorted(set(
                record['action'] 
                for daily_records in self.daily_data.values()
                for record in [daily_records]  # Convert single record to list
            ))
            
            # Compute angles for the radar chart
            num_vars = len(all_actions)
            if num_vars == 0:
                return
                
            angles = [n / float(num_vars) * 2 * np.pi for n in range(num_vars)]
            angles += angles[:1]  # Complete the circle
            
            # Create figure
            fig = Figure(figsize=(12, 8), facecolor='white', dpi=100)
            ax = fig.add_subplot(111, projection='polar')
            
            # Colors and styles for different days
            colors = ['blue', 'green', 'red', 'purple', 'orange', 'cyan']
            line_styles = ['-', '--', '-.', ':', '-', '--']
            
            # Plot data for each day
            sorted_dates = sorted(self.daily_data.keys())
            for i, date in enumerate(sorted_dates):
                # Create values array with durations for each action
                values = []
                for action in all_actions:
                    if self.daily_data[date]['action'] == action:
                        values.append(self.daily_data[date]['duration'])
                    else:
                        values.append(0)  # No duration for this action on this date
                        
                values += values[:1]  # Complete the circle
                
                label = datetime.strptime(date, '%Y-%m-%d').strftime('%Y-%m-%d')
                ax.plot(angles, values, 'o-', linewidth=2, 
                       color=colors[i % len(colors)], markersize=8,
                       linestyle=line_styles[i % len(line_styles)],
                       label=label, alpha=0.7)
                ax.fill(angles, values, color=colors[i % len(colors)], alpha=0.1)
                
                # Add value labels for the current day's data
                if date == self.current_date:
                    for angle, value, action in zip(angles[:-1], values[:-1], all_actions):
                        if value > 0:
                            ax.text(angle, value, f'{value:.1f}s', 
                                   horizontalalignment='center',
                                   verticalalignment='bottom',
                                   color=colors[i % len(colors)])
            
            # Configure chart
            ax.set_theta_offset(np.pi / 2)
            ax.set_theta_direction(-1)
            ax.set_xticks(angles[:-1])
            ax.set_xticklabels(all_actions, size=10)
            
            # Set title with date range
            if sorted_dates:
                title = f"Activity Durations\n{sorted_dates[0]} to {sorted_dates[-1]}"
            else:
                title = "Activity Durations"
            ax.set_title(title, pad=20, size=14, weight='bold')
            
            # Add legend
            ax.legend(loc='center left', bbox_to_anchor=(1.2, 0.5))
            
            # Add grid
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
            
        except Exception as e:
            self.get_logger().error(f'Error in visualization: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ActivityVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()