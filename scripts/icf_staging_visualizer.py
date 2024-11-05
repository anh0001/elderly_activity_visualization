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

class ICFStagingVisualizer(Node):
    def __init__(self):
        super().__init__('icf_staging_visualizer')
        
        # Configure number of days to show in history
        self.HISTORY_DAYS = 6
        
        # Store the last data point for each day
        self.daily_staging = {}
        
        # Human-readable stage names for display
        self.stage_names = {
            'transportation': 'Transportation',
            'grooming': 'Grooming',
            'mouth_care': 'Mouth Care',
            'basic': 'Basic',
            'dressing': 'Dressing'
        }
        
        # Create QoS profile for better reliability
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscription to processed staging data
        self.subscription = self.create_subscription(
            String,
            '/processed_icf_stages',
            self.visualization_callback,
            qos_profile
        )
        
        # Create publisher for the visualization
        self.image_publisher = self.create_publisher(
            CompressedImage,
            '/icf_staging_visualization/compressed',
            qos_profile
        )
        
        self._publish_dummy_image()
        self.get_logger().info('ICF staging visualizer initialized')
        
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
        """Publish an initialization image."""
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
            self.get_logger().info('Published initialization image')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing dummy image: {str(e)}')
    
    def _update_daily_staging(self, data, timestamp):
        """Update the stored staging data for the day."""
        date = timestamp.split('T')[0]
        self.daily_staging[date] = data
        
        # Keep only the most recent HISTORY_DAYS
        dates = sorted(self.daily_staging.keys())
        if len(dates) > self.HISTORY_DAYS:
            for old_date in dates[:-self.HISTORY_DAYS]:
                del self.daily_staging[old_date]
        
    def visualization_callback(self, msg):
        """Process incoming staging data and create visualization."""
        try:
            self.get_logger().info('Received data on processed_icf_stages')
            data = json.loads(msg.data)
            
            if data['type'] != 'icf_staging' or not data['data']:
                return
                
            self.get_logger().info(f'Received staging data: {data["data"]}')
            
            # Update daily staging data
            self._update_daily_staging(data['data'], data['timestamp'])
            
            # Get stages
            stages = list(self.stage_names.values())
            stage_keys = list(self.stage_names.keys())
            
            # Compute angles for the radar chart
            num_vars = len(stages)
            angles = [n / float(num_vars) * 2 * np.pi for n in range(num_vars)]
            angles += angles[:1]  # Complete the circle
            
            # Create figure
            fig = Figure(figsize=(12, 8), facecolor='white', dpi=100)
            ax = fig.add_subplot(111, projection='polar')
            
            # Colors and styles for different days
            colors = ['blue', 'green', 'red']
            line_styles = ['-', '--', '-.']
            
            # Plot data for each day
            sorted_dates = sorted(self.daily_staging.keys())
            for i, date in enumerate(sorted_dates):
                values = [float(self.daily_staging[date][stage]) for stage in stage_keys]
                values += values[:1]  # Complete the values circle
                
                label = f'Date: {date}'
                ax.plot(angles, values, 'o-', linewidth=2, 
                       color=colors[i], markersize=8,
                       linestyle=line_styles[i],
                       label=label, alpha=0.7)
                ax.fill(angles, values, color=colors[i], alpha=0.1)
                
                # Add value labels for the current day's data
                if i == len(sorted_dates) - 1:  # Only for most recent day
                    for angle, value, stage in zip(angles[:-1], values[:-1], stages):
                        ax.text(angle, value, f'{int(value)}', 
                               horizontalalignment='center',
                               verticalalignment='bottom',
                               color=colors[i])
            
            # Configure radar chart
            ax.set_theta_offset(np.pi / 2)
            ax.set_theta_direction(-1)
            ax.set_xticks(angles[:-1])
            ax.set_xticklabels(stages, size=12)
            
            # Set title with date range
            if sorted_dates:
                title = f"ICF Staging Progress\n{sorted_dates[0]} to {sorted_dates[-1]}"
            else:
                title = "ICF Staging Progress"
            ax.set_title(title, pad=20, size=14, weight='bold')
            
            # Set radial limits based on ICF scale (typically 1-5)
            ax.set_ylim(0, 5)
            ax.set_rgrids([1, 2, 3, 4, 5], angle=0)
            
            # Add legend
            ax.legend(loc='center left', bbox_to_anchor=(1.2, 0.5))
            
            # Add grid
            ax.grid(True, color='gray', alpha=0.3)
            
            # Create and publish message
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.format = "jpeg"
            msg.data = self._fig_to_jpeg_bytes(fig)
            
            self.image_publisher.publish(msg)
            plt.close(fig)
            self.get_logger().info('Successfully published radar chart')
            
        except Exception as e:
            self.get_logger().error(f'Error in visualization: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ICFStagingVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()