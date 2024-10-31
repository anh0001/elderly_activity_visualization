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

class ActivityVisualizer(Node):
    def __init__(self):
        super().__init__('activity_visualizer')
        
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

        # # Wait a short time to ensure topic advertisement
        # self.create_timer(5.0, self._create_subscription_delayed, callback_group=None)
        
    # def _create_subscription_delayed(self):
    #     """Create subscription after ensuring publisher is ready."""
    #     qos_profile = QoSProfile(
    #         reliability=QoSReliabilityPolicy.RELIABLE,
    #         history=QoSHistoryPolicy.KEEP_LAST,
    #         depth=10
    #     )
        
    #     self.subscription = self.create_subscription(
    #         String,
    #         '/processed_activities',
    #         self.visualization_callback,
    #         qos_profile
    #     )
        
    #     # Destroy the timer as it's no longer needed
    #     for timer in self.get_timer_handles():
    #         if timer.callback == self._create_subscription_delayed:
    #             self.destroy_timer(timer)
    #             break
                
    #     self.get_logger().info('Activity visualizer fully initialized')

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
            # Create a simple figure
            fig = Figure(figsize=(8, 8), facecolor='white')
            ax = fig.add_subplot(111)
            ax.text(0.5, 0.5, 'Initializing Activity Visualization...', 
                   horizontalalignment='center',
                   verticalalignment='center',
                   transform=ax.transAxes,
                   fontsize=14)
            ax.axis('off')
            
            # Convert to JPEG and publish
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
        
    def visualization_callback(self, msg):
        try:
            self.get_logger().info('Received data on processed_activities')
            data = json.loads(msg.data)
            
            if data['type'] != 'action_durations' or not data['data']:
                return
                
            self.get_logger().info(f'Received activity data: {data["data"]}')
            
            # Create radar chart
            activities = list(data['data'].keys())
            values = list(data['data'].values())
            num_vars = len(activities)
            
            if num_vars == 0:
                return
            
            # Compute angles
            angles = [n / float(num_vars) * 2 * np.pi for n in range(num_vars)]
            angles += angles[:1]
            values += values[:1]
            
            # Create figure
            fig = Figure(figsize=(10, 10), facecolor='white', dpi=100)
            ax = fig.add_subplot(111, projection='polar')
            
            # Plot data
            ax.plot(angles, values, 'o-', linewidth=2, color='blue', markersize=8)
            ax.fill(angles, values, alpha=0.25, color='blue')
            
            # Configure chart
            ax.set_theta_offset(np.pi / 2)
            ax.set_theta_direction(-1)
            ax.set_xticks(angles[:-1])
            ax.set_xticklabels(activities, size=10)  # Adjusted font size for potentially longer labels
            ax.set_title("Activities Duration Analysis", pad=20, size=14, weight='bold')
            ax.grid(True, color='gray', alpha=0.3)
            
            # Add value annotations
            for angle, value, activity in zip(angles[:-1], values[:-1], activities):
                ax.text(angle, value*1.1, f'{value:.1f}s', 
                       horizontalalignment='center',
                       verticalalignment='center',
                       size=9)  # Adjusted font size for duration labels
            
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