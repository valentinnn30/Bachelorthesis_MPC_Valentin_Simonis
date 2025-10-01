import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int32, Float32, Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped  # Import TransformStamped instead of TFMessage

import math
import csv
from collections import deque
from threading import Lock

class CSVLoggerNode(Node):
    def __init__(self):
        super().__init__('csv_logger_node')

        # Initialize latest values
        self.latest_transform = None
        self.latest_manual = None
        self.latest_depth = None
        self.latest_thruster = None
        self.latest_imu_yaw = None

        self.lock = Lock()

        # Subscribers
        self.create_subscription(TransformStamped, '/robot_position_to_origin', self.tf_callback, 10)  # Changed from TFMessage to TransformStamped
        self.create_subscription(Int32, '/bcu/manual', self.manual_callback, 10)
        self.create_subscription(Float32, '/depth', self.depth_callback, 10)
        self.create_subscription(Float32MultiArray, '/log/STMThruster', self.thruster_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Open CSV file once and keep it open
        self.csv_file = open('robot_log.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'x', 'y', 'z', 'yaw',
            'manual_command', 'depth',
            'thruster_0', 'thruster_1', 'thruster_2', 'thruster_3',
            'imu_yaw'
        ])

        # Start periodic timer
        self.create_timer(0.1, self.write_csv_row)

    def tf_callback(self, msg):
        with self.lock:
            # Assuming that we receive a single TransformStamped message now
            if msg.child_frame_id == 'robot_frame' and msg.header.frame_id == 'origin_tag36h11:1':
                t = msg.transform.translation
                r = msg.transform.rotation
                yaw = self.quaternion_to_yaw(r.x, r.y, r.z, r.w)
                self.latest_transform = (t.x, t.y, t.z, yaw)

    def manual_callback(self, msg):
        with self.lock:
            self.latest_manual = msg.data

    def depth_callback(self, msg):
        with self.lock:
            self.latest_depth = msg.data

    def thruster_callback(self, msg):
        with self.lock:
            self.latest_thruster = msg.data

    def imu_callback(self, msg):
        with self.lock:
            q = msg.orientation
            self.latest_imu_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def write_csv_row(self):
        with self.lock:
            if self.latest_transform is None or self.latest_manual is None or \
               self.latest_depth is None or self.latest_thruster is None or \
               self.latest_imu_yaw is None:
                self.get_logger().warn('Waiting for all data...')
                return

            timestamp = self.get_clock().now().to_msg().sec + \
                        self.get_clock().now().to_msg().nanosec * 1e-9

            x, y, z, yaw = self.latest_transform
            manual = self.latest_manual
            depth = self.latest_depth
            thrusters = self.latest_thruster[:4]  # Trim or pad as needed
            imu_yaw = self.latest_imu_yaw

            row = [timestamp, x, y, z, yaw, manual, depth] + list(thrusters) + [imu_yaw]
            self.csv_writer.writerow(row)
            self.csv_file.flush()
            self.get_logger().info(f'Row written: {row}')

    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw"""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CSVLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()