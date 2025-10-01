#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque

# === Core Filtering Class ===
class UltraSoftDeflicker:
    def __init__(self, buffer_size=3, alpha=0.9, blur_kernel=9):
        self.buffer = deque(maxlen=buffer_size)
        self.alpha = np.clip(alpha, 0.0, 1.0)
        self.blur_kernel = blur_kernel

    def warp_to_current(self, current, past):
        flow = cv2.calcOpticalFlowFarneback(past, current, None,
                                            pyr_scale=0.5, levels=3,
                                            winsize=15, iterations=3,
                                            poly_n=5, poly_sigma=1.2,
                                            flags=0)
        h, w = current.shape
        grid_x, grid_y = np.meshgrid(np.arange(w), np.arange(h))
        map_x = (grid_x + flow[..., 0]).astype(np.float32)
        map_y = (grid_y + flow[..., 1]).astype(np.float32)
        return cv2.remap(past, map_x, map_y, interpolation=cv2.INTER_LINEAR)

    def apply(self, gray):
        cur = gray.astype(np.float32)
        warped_stack = [cur]

        for past in self.buffer:
            warped = self.warp_to_current(gray, past)
            warped_stack.append(warped)

        median = np.median(np.stack(warped_stack), axis=0)
        median = cv2.GaussianBlur(median, (self.blur_kernel, self.blur_kernel), 0)

        result = self.alpha * cur + (1.0 - self.alpha) * median
        self.buffer.append(gray)

        return np.clip(result, 0, 255).astype(np.uint8)

# === ROS 2 Node ===
class UltraSoftDeflickerNode(Node):
    def __init__(self):
        super().__init__('ultrasoft_deflicker_node')
        self.bridge = CvBridge()
        self.filter = UltraSoftDeflicker(buffer_size=3, alpha=0.9, blur_kernel=9)

        self.sub = self.create_subscription(Image,
                                            '/image_raw',
                                            self.cb_img,
                                            10)
        self.pub = self.create_publisher(Image,
                                         '/camera/rgb/image_raw',
                                         10)

        self.get_logger().info('UltraSoft Deflicker Node started (very gentle correction).')

    def cb_img(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

            dgray = self.filter.apply(gray)
            dbgr = cv2.cvtColor(dgray, cv2.COLOR_GRAY2BGR)

            out_msg = self.bridge.cv2_to_imgmsg(dbgr, encoding='bgr8')
            out_msg.header = msg.header
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing frame: {e}')

# === Main Entry ===
def main(args=None):
    rclpy.init(args=args)
    node = UltraSoftDeflickerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
