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
        
        # Convert to numpy array
        buf = io.BytesIO()
        canvas.print_png(buf)
        buf.seek(0)
        
        # Convert to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(
            np.frombuffer(buf.getvalue(), dtype=np.uint8),
            encoding='bgr8'
        )
        
        # Publish image
        self.image_publisher.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ActivityVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()