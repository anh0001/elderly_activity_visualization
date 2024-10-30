#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory

class LogoPublisher(Node):
    def __init__(self):
        super().__init__('logo_publisher')
        
        # Create publisher for compressed image
        self.publisher = self.create_publisher(
            CompressedImage, 
            'ros2hc_logo/compressed',
            10
        )
        
        # Set timer for periodic publishing (1 Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Get package path and load image
        package_name = 'elderly_activity_visualization'
        package_path = get_package_share_directory(package_name)
        image_path = os.path.join(package_path, 'media', 'ros2hc_logo.jpeg')
        
        # Load and store the image
        self.image = cv2.imread(image_path)
        if self.image is None:
            self.get_logger().error(f'Failed to load image from {image_path}')
            return
            
        # Pre-encode the image as JPEG
        self.compressed_msg = CompressedImage()
        self.compressed_msg.format = "jpeg"
        # JPEG encoding parameters: quality = 95 for high quality
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, 95]
        _, img_encoded = cv2.imencode('.jpg', self.image, encode_params)
        self.compressed_msg.data = np.array(img_encoded).tobytes()
        
        self.get_logger().info('Logo publisher initialized successfully')

    def timer_callback(self):
        if hasattr(self, 'compressed_msg'):
            # Update timestamp
            self.compressed_msg.header.stamp = self.get_clock().now().to_msg()
            self.compressed_msg.header.frame_id = "map"
            
            # Publish the compressed image
            self.publisher.publish(self.compressed_msg)
            self.get_logger().debug('Published compressed logo image')

def main(args=None):
    rclpy.init(args=args)
    logo_publisher = LogoPublisher()
    
    try:
        rclpy.spin(logo_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        logo_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()