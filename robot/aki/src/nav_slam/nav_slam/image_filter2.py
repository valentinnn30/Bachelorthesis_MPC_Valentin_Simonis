#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from collections import deque

class RecGSFilter:
    def __init__(self, buffer_size=5, fft_keep=10):
        self.prev_frames = deque(maxlen=buffer_size)
        self.fft_keep = fft_keep  # number of low frequencies to keep

    def update(self, frame_gray):
        self.prev_frames.append(frame_gray.astype(np.float32))

        if len(self.prev_frames) < self.prev_frames.maxlen:
            return frame_gray  # not enough history yet

        # Estimate static scene (background approximation)
        I_hat = np.mean(self.prev_frames, axis=0)
        R = frame_gray - I_hat

        # FFT
        R_fft = np.fft.fft2(R)
        R_fft_shift = np.fft.fftshift(R_fft)

        # Low-pass filter: keep only central k×k frequencies
        h, w = R.shape
        mask = np.zeros_like(R, dtype=np.uint8)
        k = self.fft_keep
        mask[h//2-k:h//2+k, w//2-k:w//2+k] = 1
        R_fft_filtered = R_fft_shift * mask

        # Inverse FFT to estimate caustic
        C = np.fft.ifft2(np.fft.ifftshift(R_fft_filtered)).real

        # Subtract caustic and clip result
        I_deflickered = frame_gray - C
        I_deflickered = np.clip(I_deflickered, 0, 255).astype(np.uint8)
        return I_deflickered

class RecGSFilterNode(Node):
    def __init__(self):
        super().__init__('recgs_filter_node')
        self.bridge = CvBridge()
        self.filter = RecGSFilter(buffer_size=5, fft_keep=9)

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)

        self.publisher = self.create_publisher(Image, '/camera/rgb/image_raw', 10)

        self.get_logger().info('RecGS Caustic Filter Node started.')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            deflickered_gray = self.filter.update(gray)
            deflickered_bgr = cv2.cvtColor(deflickered_gray, cv2.COLOR_GRAY2BGR)

            out_msg = self.bridge.cv2_to_imgmsg(deflickered_bgr, encoding='bgr8')
            out_msg.header = msg.header
            self.publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing frame: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RecGSFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
