#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
from sensor_msgs.msg import CompressedImage
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ActivityVisualizer(Node):
    def __init__(self):
        super().__init__('activity_visualizer')
        
        # Define QoS profile for better reliability
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            String,
            'processed_activities',
            self.visualization_callback,
            qos_profile
        )
        
        self.image_publisher = self.create_publisher(
            CompressedImage,
            'activity_visualization/compressed',
            qos_profile
        )
        
        # Create a dummy image to ensure topic is active
        self._publish_dummy_image()
        self.get_logger().info('Activity visualizer initialized')
        
    def _publish_dummy_image(self):
        """Publish a dummy image to initialize the topic."""
        try:
            # Create a small black image
            dummy_image = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(dummy_image, 'Initializing...', (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # Create compressed image message
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', dummy_image)[1]).tobytes()
            
            self.image_publisher.publish(msg)
            self.get_logger().info('Published dummy compressed image')
        except Exception as e:
            self.get_logger().error(f'Error publishing dummy image: {str(e)}')
        
    def visualization_callback(self, msg):
        """Create and publish a simple visualization for testing."""
        try:
            self.get_logger().info('Received data on processed_activities')
            data = json.loads(msg.data)
            
            if not data['data']:
                self.get_logger().warn('Received empty data')
                return
                
            self.get_logger().info(f'Received data: {data["data"]}')
            
            # Create a simple colored image
            height, width = 480, 640  # Standard VGA size
            image = np.ones((height, width, 3), dtype=np.uint8) * 255
            
            # Draw some text and shapes
            font = cv2.FONT_HERSHEY_SIMPLEX
            # Title
            cv2.putText(image, 'Activity Visualization', 
                       (int(width/2 - 150), 50), font, 1, (0, 0, 255), 2)
            
            # Background rectangle
            cv2.rectangle(image, (50, 70), (width-50, height-50), 
                         (200, 200, 200), cv2.FILLED)
            cv2.rectangle(image, (50, 70), (width-50, height-50), 
                         (0, 0, 0), 2)
            
            # Add values from the data
            y_pos = 100
            for category, value in data['data'].items():
                text = f'{category}: {value:.2f}'
                cv2.putText(image, text, (100, y_pos), font, 0.7, (0, 0, 0), 2)
                # Draw a bar
                bar_length = int(min(value, 400))  # Limit bar length
                cv2.rectangle(image, (100, y_pos+10), (100+bar_length, y_pos+30),
                            (0, 255, 0), cv2.FILLED)
                y_pos += 60
            
            # Create compressed image message
            try:
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"
                msg.format = "jpeg"
                msg.data = np.array(cv2.imencode('.jpg', image)[1]).tobytes()
                
                # Publish image
                self.image_publisher.publish(msg)
                self.get_logger().info('Successfully published compressed image')
                
            except Exception as e:
                self.get_logger().error(f'Error in image conversion/publishing: {str(e)}')
            
        except Exception as e:
            self.get_logger().error(f'Error in visualization: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ActivityVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()




# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import json
# import matplotlib.pyplot as plt
# import numpy as np
# from matplotlib.figure import Figure
# from matplotlib.backends.backend_agg import FigureCanvasAgg
# import io
# from sensor_msgs.msg import Image, CompressedImage
# from cv_bridge import CvBridge
# import cv2

# class ActivityVisualizer(Node):
#     def __init__(self):
#         super().__init__('activity_visualizer')
        
#         self.subscription = self.create_subscription(
#             String,
#             'processed_activities',
#             self.visualization_callback,
#             10
#         )
        
#         # Create two publishers - one for raw images and one for compressed
#         self.image_publisher = self.create_publisher(
#             Image,
#             'activity_visualization',
#             10
#         )
#         self.compressed_publisher = self.create_publisher(
#             CompressedImage,
#             'activity_visualization/compressed',
#             10
#         )
        
#         self.bridge = CvBridge()
#         self.get_logger().info('Activity visualizer initialized')
        
#     def visualization_callback(self, msg):
#         """Create and publish radar chart visualization."""
#         try:
#             data = json.loads(msg.data)
            
#             if data['type'] != 'radar_chart':
#                 return
                
#             if not data['data']:  # Skip if data is empty
#                 return
                
#             # Create radar chart
#             categories = list(data['data'].keys())
#             values = list(data['data'].values())
            
#             # Number of variables
#             num_vars = len(categories)
            
#             if num_vars == 0:  # Skip if no categories
#                 return
            
#             # Compute angle for each axis
#             angles = [n / float(num_vars) * 2 * np.pi for n in range(num_vars)]
#             angles += angles[:1]
            
#             # Initialize the figure with white background
#             fig = Figure(figsize=(8, 8), facecolor='white')
#             ax = fig.add_subplot(111, projection='polar')
            
#             # Complete the loop of values
#             values += values[:1]
            
#             # Plot data
#             ax.plot(angles, values, 'o-', linewidth=2, color='blue')
#             ax.fill(angles, values, alpha=0.25, color='blue')
            
#             # Fix axis to start from top
#             ax.set_theta_offset(np.pi / 2)
#             ax.set_theta_direction(-1)
            
#             # Set labels
#             ax.set_xticks(angles[:-1])
#             ax.set_xticklabels(categories)
            
#             # Add title
#             ax.set_title("Activity Categories Duration", pad=20)
            
#             # Add gridlines
#             ax.grid(True)
            
#             # Draw everything to the canvas
#             canvas = FigureCanvasAgg(fig)
#             canvas.draw()
            
#             # Convert canvas to image
#             buf = io.BytesIO()
#             fig.savefig(buf, format='png', bbox_inches='tight', facecolor='white', dpi=100)
#             buf.seek(0)
            
#             # Convert buffer to numpy array
#             img_arr = np.frombuffer(buf.getvalue(), dtype=np.uint8)
#             buf.close()
            
#             # Decode buffer to OpenCV image
#             cv_image = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
            
#             # Get current timestamp
#             current_time = self.get_clock().now().to_msg()
            
#             # Create and publish raw image message
#             img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
#             img_msg.header.stamp = current_time
#             img_msg.header.frame_id = "map"  # Set a valid frame_id
#             self.image_publisher.publish(img_msg)
            
#             # Create and publish compressed image message
#             compressed_msg = CompressedImage()
#             compressed_msg.header.stamp = current_time
#             compressed_msg.header.frame_id = "map"  # Set a valid frame_id
#             compressed_msg.format = "png"
#             compressed_msg.data = np.array(cv2.imencode('.png', cv_image)[1]).tobytes()
#             self.compressed_publisher.publish(compressed_msg)
            
#             # Clean up
#             plt.close(fig)
            
#             self.get_logger().debug('Published visualization')
            
#         except Exception as e:
#             self.get_logger().error(f'Error in visualization: {str(e)}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = ActivityVisualizer()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()