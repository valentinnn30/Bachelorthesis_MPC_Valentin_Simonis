import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(
            Image,
            'camera/rgb/image_raw',  
            10)
        self.timer = self.create_timer(10, self.publish_image)  # Adjust the timer frequency as needed
        self.bridge = CvBridge()

    def publish_image(self):
        # Load or capture your image here (replace 'input_image.jpg' with your source)
        cv_image = cv2.imread('/home/reefranger/Desktop/cor/firstry.jpg')  # Example loading a file, replace with actual capture logic

        # Resize the image to 640x640
        #resized_image = cv2.resize(cv_image, (3888,3888))

        # Convert the resized image to a ROS Image message
        resized_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

        # Publish the resized image
        self.publisher_.publish(resized_msg)
        self.get_logger().info('Published resized image')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
