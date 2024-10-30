#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
import io
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ActivityVisualizer(Node):
    def __init__(self):
        super().__init__('activity_visualizer')
        
        self.subscription = self.create_subscription(
            String,
            'processed_activities',
            self.visualization_callback,
            10
        )
        
        self.image_publisher = self.create_publisher(
            Image,
            'activity_visualization',
            10
        )
        
        self.bridge = CvBridge()
        
    def visualization_callback(self, msg):
        """Create and publish radar chart visualization."""
        try:
            data = json.loads(msg.data)
            
            if data['type'] != 'radar_chart':
                return
                
            # Create radar chart
            categories = list(data['data'].keys())
            values = list(data['data'].values())
            
            # Number of variables
            num_vars = len(categories)
            
            # Compute angle for each axis
            angles = [n / float(num_vars) * 2 * np.pi for n in range(num_vars)]
            angles += angles[:1]
            
            # Initialize the figure
            fig = Figure(figsize=(10, 10))
            ax = fig.add_subplot(111, projection='polar')
            
            # Plot data
            values += values[:1]
            ax.plot(angles, values)
            ax.fill(angles, values, alpha=0.25)
            
            # Set the labels
            ax.set_xticks(angles[:-1])
            ax.set_xticklabels(categories)
            
            # Convert plot to image
            canvas = FigureCanvasAgg(fig)
            canvas.draw()
            
            # Get the RGBA buffer from the figure
            w, h = fig.get_size_inches() * fig.get_dpi()
            buf = canvas.buffer_rgba()
            
            # Convert to numpy array
            img_array = np.frombuffer(buf, dtype=np.uint8)
            img_array = img_array.reshape(int(h), int(w), 4)
            
            # Convert RGBA to BGR for cv_bridge
            img_bgr = cv2.cvtColor(img_array, cv2.COLOR_RGBA2BGR)
            
            # Convert to ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(img_bgr, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Publish image
            self.image_publisher.publish(image_msg)
            
            # Close the figure to free memory
            plt.close(fig)
            
        except Exception as e:
            self.get_logger().error(f'Error in visualization callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ActivityVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()