#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from collections import deque
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class FastDeflickerNode(Node):
    def __init__(self):
        super().__init__('fast_deflicker_node')
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.bridge = CvBridge()
        self.history = deque(maxlen=3)  # no optical flow, just raw past grayscale frames

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if not self.history:
            self.history.append(gray)
            self.publish_deflickered(frame, msg)
            return

        # Simple average of past frames for background model
        avg = np.mean(self.history, axis=0).astype(np.uint8)

        # Compute flicker
        delta = cv2.absdiff(gray, avg)
        delta_blur = cv2.GaussianBlur(delta, (3, 3), 0)

        # Soft mask, very mild
        mask_float = np.clip(delta_blur.astype(np.float32) / 100.0, 0.0, 1.0)
        mask_float *= 0.3  # mild influence

        # Blend current with average to suppress only flickering bits
        filtered = (mask_float * avg + (1 - mask_float) * gray).astype(np.uint8)

        # Update history
        self.history.append(gray)

        # Convert to color
        final = cv2.cvtColor(filtered, cv2.COLOR_GRAY2BGR)
        self.publish_deflickered(final, msg)

    def publish_deflickered(self, img, orig_msg):
        out_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out_msg.header = orig_msg.header
        self.publisher.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FastDeflickerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
