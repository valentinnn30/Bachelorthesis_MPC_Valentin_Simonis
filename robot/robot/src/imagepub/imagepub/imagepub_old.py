import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv2 import imread, cvtColor, COLOR_BGR2RGB
import cv2
import numpy as np

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Load your image
        self.image_path = '/home/reefranger/Desktop/cor/firstry.jpg'
        self.image = imread(self.image_path)

        # Ensure the image is loaded
        if self.image is None:
            self.get_logger().error('Failed to load image: {}'.format(self.image_path))
            return

        # Convert image to RGB format (OpenCV loads images in BGR)
        self.image = cvtColor(self.image, COLOR_BGR2RGB)
        self.height, self.width, _ = self.image.shape

    def timer_callback(self):
        # Create an Image message
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'rgb8'  # You can change this to 'bgr8' if needed
        msg.is_bigendian = 0
        msg.step = self.width * 3  # For RGB, 3 bytes per pixel
        msg.data = np.array(self.image).tobytes()

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing image...')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
