#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
import io
from datetime import datetime
from collections import OrderedDict

class ICFStagingVisualizer(Node):
    def __init__(self):
        super().__init__('icf_staging_visualizer')
        
        # Configure number of days to show in history
        self.declare_parameter('days_to_retain', 6)
        
        # Store the staging data for each day
        self.daily_staging = OrderedDict()
        self.days_to_retain = self.get_parameter('days_to_retain').value
        
        # Human-readable stage names for display
        self.stage_names = {
            'transportation_stage': 'Transportation',
            'grooming_stage': 'Grooming',
            'mouth_care_stage': 'Mouth Care',
            'basic_stage': 'Basic',
            'dressing_stage': 'Dressing'
        }
        
        # Colors for different days
        self.colors = ['blue', 'green', 'red', 'purple', 'orange', 'cyan']
        self.line_styles = ['-', '--', '-.', ':', '-', '--']
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            String,
            '/processed_icf_stages',
            self.visualization_callback,
            qos_profile
        )
        
        self.image_publisher = self.create_publisher(
            CompressedImage,
            '/icf_staging_visualization/compressed',
            qos_profile
        )
        
        self._publish_dummy_image()
        self.get_logger().info('ICF staging visualizer initialized')

    def _extract_date(self, timestamp):
        """Extract date from timestamp string."""
        return timestamp.split()[0]

    def _update_daily_staging(self, timestamp, data):
        """Update the stored staging data for the day."""
        try:
            date = self._extract_date(timestamp)
            
            # Store staging data for this date
            self.daily_staging[date] = {
                key: value for key, value in data.items() 
                if key in self.stage_names.keys()
            }
            
            # Sort dictionary by date
            self.daily_staging = OrderedDict(sorted(self.daily_staging.items()))
            
            # Keep only the most recent days_to_retain days
            while len(self.daily_staging) > self.days_to_retain:
                self.daily_staging.popitem(first=True)
                
            self.get_logger().info(f'Updated staging for {date}, total dates: {len(self.daily_staging)}')
            self.get_logger().info(f'Current dates in store: {list(self.daily_staging.keys())}')
            
        except Exception as e:
            self.get_logger().error(f'Error updating daily staging: {str(e)}')

    def _fig_to_jpeg_bytes(self, fig):
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
            ax.text(0.5, 0.5, 'Initializing ICF Staging Visualization...', 
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
            data = json.loads(msg.data)
            
            if data['type'] != 'icf_staging' or not data['data']:
                return
                
            # Update staging data
            self._update_daily_staging(data['timestamp'], data['data'])
            
            if not self.daily_staging:
                return
                
            # Create figure
            fig = Figure(figsize=(14, 10), facecolor='white', dpi=100)
            ax = fig.add_subplot(111, projection='polar')
            
            # Get stages and compute angles
            stages = list(self.stage_names.values())
            stage_keys = list(self.stage_names.keys())
            
            num_vars = len(stages)
            angles = [n / float(num_vars) * 2 * np.pi for n in range(num_vars)]
            angles += angles[:1]
            
            # Plot each day's data
            for i, (date, staging) in enumerate(self.daily_staging.items()):
                values = [float(staging[stage]) for stage in stage_keys]
                values += values[:1]
                
                color = self.colors[i % len(self.colors)]
                style = self.line_styles[i % len(self.line_styles)]
                
                self.get_logger().info(f'Plotting data for {date} with color {color}')
                
                # Plot data
                line = ax.plot(angles, values, 'o-', linewidth=2.5,
                             label=date, color=color, markersize=8,
                             linestyle=style, alpha=0.7)
                ax.fill(angles, values, color=color, alpha=0.15)
                
                # Add value labels for most recent date
                if i == len(self.daily_staging) - 1:
                    for angle, value, stage in zip(angles[:-1], values[:-1], stages):
                        if value > 0:
                            ax.text(angle, value*1.1, f'{int(value)}',
                                   horizontalalignment='center',
                                   verticalalignment='center',
                                   color=color)
            
            # Configure chart
            ax.set_theta_offset(np.pi / 2)
            ax.set_theta_direction(-1)
            ax.set_xticks(angles[:-1])
            ax.set_xticklabels(stages, size=10)
            
            # Set title and limits
            ax.set_title("ICF Staging Progress", pad=20, size=14, weight='bold')
            ax.set_ylim(0, 5)
            ax.set_rgrids([1, 2, 3, 4, 5], angle=0)
            
            # Add legend with adjusted position
            legend = ax.legend(loc='upper right', bbox_to_anchor=(1.4, 1.1),
                             title="Dates", title_fontsize=12)
            
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
            self.get_logger().info('Successfully published ICF staging chart')
            
        except Exception as e:
            self.get_logger().error(f'Error in visualization: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ICFStagingVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()