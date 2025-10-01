#!/usr/bin/env python3

"""
Python node for the MonocularMode CPP node using a live USB camera feed.

Author: Your Name
Date: Your Date
"""

# Imports
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2


class MonoDriver(Node):
    def __init__(self, node_name="mono_py_node"):
        super().__init__(node_name)

        # Initialize parameters (optional, can use them in future configurations)
        self.declare_parameter("settings_name", "EuRoC")
        self.settings_name = str(self.get_parameter('settings_name').value)

        # Debug information
        print(f"-------------- Received parameters --------------------------")
        print(f"self.settings_name: {self.settings_name}\n")

        # Initialize CvBridge
        self.br = CvBridge()

        # ROS2 subscription for live camera feed
        self.create_subscription(
            Image,
            '/image_raw',  # Replace with your camera topic
            self.image_callback,
            10  # Queue size
        )

        # Publisher to send experiment settings (if needed by the CPP node)
        self.publish_exp_config_ = self.create_publisher(String, "/mono_py_driver/experiment_settings", 1)

        # Debug information
        print(f"MonoDriver initialized, waiting for live camera feed...\n")

    def image_callback(self, msg):
        """
        Callback function for live camera feed.
        """
        try:
            # Convert ROS Image to OpenCV format
            frame = self.br.imgmsg_to_cv2(msg, "bgr8")

            # Generate a timestamp for SLAM
            timestamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

            # Debug: Display the frame (optional)
            cv2.imshow("Live Camera Feed", frame)

            # TODO: Pass the frame and timestamp to ORB-SLAM (modify to fit your implementation)
            # Example:
            # orb_slam.TrackMonocular(frame, timestamp)

            if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit visualization
                rclpy.shutdown()

        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")


def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2 node
    node = MonoDriver("mono_py_node")  # Initialize the MonoDriver node

    try:
        print("Spinning ROS2 node. Waiting for live camera feed...")
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        print("Shutting down node...")
    finally:
        cv2.destroyAllWindows()  # Close all OpenCV windows
        node.destroy_node()  # Destroy the node
        rclpy.shutdown()  # Shutdown ROS2


if __name__ == "__main__":
    main()
