import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation
import numpy as np
from collections import defaultdict
import re


class TagPairAveragerPublisher(Node):
    def __init__(self):
        super().__init__('tag_pair_averager_publisher')

        self.subscription = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)

        self.translation_sums = defaultdict(lambda: np.zeros(3))
        self.rotation_sums = defaultdict(lambda: np.zeros(4))
        self.sample_counts = defaultdict(int)
        self.last_log_time = defaultdict(float)
        self.tag_pair_publishers = {}

        self.log_interval = 1.0  # seconds

    def tf_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9

        tag_transforms = {}
        for t in msg.transforms:
            if t.header.frame_id == "camera" and t.child_frame_id.startswith("tag36h11:"):
                tag_transforms[t.child_frame_id] = t.transform

        tag_ids = list(tag_transforms.keys())

        for i in range(len(tag_ids)):
            for j in range(i + 1, len(tag_ids)):
                tag_a = tag_ids[i]
                tag_b = tag_ids[j]
                key = (tag_a, tag_b)

                T_a = self.transform_to_matrix(tag_transforms[tag_a])
                T_b = self.transform_to_matrix(tag_transforms[tag_b])
                T_ab = np.linalg.inv(T_a) @ T_b

                translation = T_ab[:3, 3]
                rotation = Rotation.from_matrix(T_ab[:3, :3]).as_quat()  # x, y, z, w

                self.translation_sums[key] += translation
                self.rotation_sums[key] += rotation
                self.sample_counts[key] += 1

                # Log to terminal
                if now - self.last_log_time[key] > self.log_interval:
                    self.log_average_transform(key)
                    self.last_log_time[key] = now

                # Publish to topic
                self.publish_average_transform(key)

    def transform_to_matrix(self, transform):
        t = transform.translation
        q = transform.rotation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def log_average_transform(self, key):
        count = self.sample_counts[key]
        if count == 0:
            return

        avg_translation = self.translation_sums[key] / count
        avg_rotation = self.rotation_sums[key] / np.linalg.norm(self.rotation_sums[key])

        self.get_logger().info(
            f"\n📌 AVERAGED TRANSFORM {key[0]} ➝ {key[1]} (samples: {count}):\n"
            f"  Translation (m): x={avg_translation[0]:.3f}, y={avg_translation[1]:.3f}, z={avg_translation[2]:.3f}\n"
            f"  Rotation (quat): x={avg_rotation[0]:.4f}, y={avg_rotation[1]:.4f}, "
            f"z={avg_rotation[2]:.4f}, w={avg_rotation[3]:.4f}"
        )

    def publish_average_transform(self, key):
        count = self.sample_counts[key]
        if count == 0:
            return

        avg_translation = self.translation_sums[key] / count
        avg_rotation = self.rotation_sums[key] / np.linalg.norm(self.rotation_sums[key])

        tag_a_clean = re.sub(r"[^a-zA-Z0-9]", "_", key[0])
        tag_b_clean = re.sub(r"[^a-zA-Z0-9]", "_", key[1])
        topic_name = f"/averaged_transforms/{tag_a_clean}_to_{tag_b_clean}"

        if topic_name not in self.tag_pair_publishers:
            self.tag_pair_publishers[topic_name] = self.create_publisher(TransformStamped, topic_name, 10)

        transform_msg = TransformStamped()
        transform_msg.header = Header()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = key[0]
        transform_msg.child_frame_id = key[1]
        transform_msg.transform.translation.x = avg_translation[0]
        transform_msg.transform.translation.y = avg_translation[1]
        transform_msg.transform.translation.z = avg_translation[2]
        transform_msg.transform.rotation.x = avg_rotation[0]
        transform_msg.transform.rotation.y = avg_rotation[1]
        transform_msg.transform.rotation.z = avg_rotation[2]
        transform_msg.transform.rotation.w = avg_rotation[3]

        self.tag_pair_publishers[topic_name].publish(transform_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TagPairAveragerPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
