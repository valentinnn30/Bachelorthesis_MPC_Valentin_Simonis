import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation
import numpy as np
from collections import defaultdict
import re

class RobotPositionPublisher(Node):
    def __init__(self):
        super().__init__('robot_position_publisher')

        # Subscribe to the /tf topic to get the transformations
        self.subscription = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)

        # Store the tag-to-origin transforms (already known)
        self.transforms_to_origin = {
            1: np.eye(4),  # Identity matrix for origin tag
            2: np.array([[ 0.99784852,  0.00441764, -0.06541267, -0.34190208],
                        [-0.00295148,  0.99974263,  0.02249376, -0.01876456],
                        [ 0.06549521, -0.02225231,  0.99760474, -0.01064881],
                        [ 0., 0., 0., 1.]]),
            3: np.array([[ 8.91971885e-01,  4.50468774e-01,  3.82627817e-02, -5.71577442e-01],
                        [-4.52088981e-01,  8.89012621e-01,  7.26093182e-02,  2.42562756e+00],
                        [-1.30786527e-03, -8.20636523e-02,  9.96626232e-01, -6.89636054e-01],
                        [ 0., 0., 0., 1.]]),
            4: np.array([[ 0.47660384, -0.86663953,  0.14759644, -0.27124038],
                        [ 0.87384851,  0.48537757,  0.02823805,  2.26191422],
                        [-0.09611221,  0.11551857,  0.98864448, -0.68892407],
                        [ 0., 0., 0., 1.]]),
            5: np.array([[-0.06297582, -0.99075712,  0.12014314, -0.28067169],
                        [ 0.98000692, -0.08415514, -0.18028964,  1.00676052],
                        [ 0.1887339,   0.10638722,  0.97624857, -0.6839048 ],
                        [ 0., 0., 0., 1.]])
        }


        # Define a publisher for the robot's position
        self.robot_position_publisher = self.create_publisher(TransformStamped, '/robot_position_to_origin', 10)

        self.camera_to_tag_transform = None
        self.origin_tag_id = 1  # Default origin tag ID

        # Smoothing parameters
        self.position_history = []  # History to store previous positions for smoothing
        self.history_size = 5  # Number of previous positions to average
        self.position_threshold = 0.1  # Minimum change required to update the position

    def tf_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9  # Current time in seconds

        tag_transforms = {}
        tag_timestamps = {}

        # Extract transforms for tags in view
        for t in msg.transforms:
            if t.header.frame_id == "camera" and t.child_frame_id.startswith("tag36h11:"):
                tag_transforms[t.child_frame_id] = t.transform
                # Store the timestamp for each tag
                tag_timestamps[t.child_frame_id] = t.header.stamp

        # Iterate over all tags (ID 1 to 4) and use the first one in view
        tag_ids_in_view = [f"tag36h11:{i}" for i in range(2, 6)]  # Tags 1 to 4
        tag_id_in_view = None

        for tag_id in tag_ids_in_view:
            if tag_id in tag_transforms:
                tag_id_in_view = tag_id
                break  # Use the first tag in view

        if tag_id_in_view is None:
            self.get_logger().warn("No tag is currently in view.")
            return

        # Get the transform from the camera to the selected tag
        self.camera_to_tag_transform = tag_transforms[tag_id_in_view]
        tag_timestamp = tag_timestamps[tag_id_in_view]

        # Compute robot position relative to the origin
        self.compute_robot_position(tag_id_in_view, tag_timestamp)

    def compute_robot_position(self, tag_id_in_view, timestamp):
        # Fetch the transform from origin tag (ID 1) to the tag in view
        transform_to_origin = self.transforms_to_origin.get(int(tag_id_in_view.split(':')[1]))

        if transform_to_origin is None:
            self.get_logger().warn(f"No transform available for tag {tag_id_in_view}")
            return

        # Inverse the camera-to-tag transform to get the robot's position relative to the tag
        # robot_position_relative_to_tag = self.inverse_transform(self.camera_to_tag_transform)

        # Inverse the camera-to-tag transform to get the robot's position relative to the tag
        robot_position_relative_to_tag, robot_rotation_relative_to_tag = self.inverse_transform(self.camera_to_tag_transform)

        # Calculate the robot's position and orientation relative to the origin tag
        robot_position_relative_to_origin = self.calculate_robot_position(
            transform_to_origin, robot_position_relative_to_tag)
        robot_rotation_relative_to_origin = self.apply_rotation_to_origin(robot_rotation_relative_to_tag, transform_to_origin)

        # Calculate the robot's position relative to the origin tag
        robot_position_relative_to_origin = self.calculate_robot_position(
            transform_to_origin, robot_position_relative_to_tag)

        # Apply smoothing (moving average)
        smoothed_position = self.apply_smoothing(robot_position_relative_to_origin)

        # Publish the robot's position relative to the origin tag
        self.publish_robot_transform(smoothed_position, robot_rotation_relative_to_origin, timestamp)

    def inverse_transform(self, transform):
        """Inverse the transform from the camera to the tag to get the robot's position"""
        rotation = transform.rotation
        rotation_matrix = self.quaternion_to_rotation_matrix(rotation)
        rotation_matrix_inv = np.transpose(rotation_matrix)

        translation = transform.translation
        translation_vec = np.array([translation.x, translation.y, translation.z])

        robot_translation = np.dot(rotation_matrix_inv, translation_vec)

        # Also return the inverse rotation (as a quaternion) for the robot's orientation
        robot_rotation = Rotation.from_matrix(rotation_matrix_inv).as_quat()
        return robot_translation, robot_rotation

    def calculate_robot_position(self, transform_to_origin, robot_position_relative_to_tag):
        """Calculate the robot's position relative to the origin tag"""
        # Apply the transformation from the tag to the origin
        robot_position_in_origin = robot_position_relative_to_tag + transform_to_origin[:3, 3]
        return robot_position_in_origin
    
    def apply_rotation_to_origin(self, robot_rotation_relative_to_tag, transform_to_origin):
        """Apply the tag-to-origin rotation to the robot's orientation"""
        tag_rotation_matrix = transform_to_origin[:3, :3]  # Rotation part of the transformation matrix
        robot_rotation_matrix = Rotation.from_quat(robot_rotation_relative_to_tag).as_matrix()
        
        # Multiply the rotations to get the robot's orientation relative to the origin
        final_rotation_matrix = np.dot(tag_rotation_matrix, robot_rotation_matrix)

        # Convert the final rotation matrix back to a quaternion
        final_rotation = Rotation.from_matrix(final_rotation_matrix).as_quat()
        return final_rotation

    def quaternion_to_rotation_matrix(self, quaternion):
        """Convert a quaternion to a 3x3 rotation matrix"""
        w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        matrix = np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x**2 + z**2), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x**2 + y**2)]
        ])
        return matrix

    def apply_smoothing(self, new_position):
        """Smooth the robot's position using a moving average"""
        self.position_history.append(new_position)

        # Keep the history size fixed
        if len(self.position_history) > self.history_size:
            self.position_history.pop(0)

        # Calculate the average position
        smoothed_position = np.mean(self.position_history, axis=0)
        return smoothed_position

    def publish_robot_transform(self, robot_position, robot_rotation, timestamp):
        """Publish the robot's position relative to the target tag"""
        transform_stamped = TransformStamped()

        # transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.stamp = timestamp
        transform_stamped.header.frame_id = f"origin_tag36h11:{self.origin_tag_id}"  # Parent frame (origin tag)
        transform_stamped.child_frame_id = "robot_frame"  # Child frame (robot's frame)

        # Set the translation (robot's position relative to the tag)
        transform_stamped.transform.translation.x = robot_position[0]
        transform_stamped.transform.translation.y = robot_position[1]
        transform_stamped.transform.translation.z = robot_position[2]

        # Set the rotation (robot's orientation relative to the origin)
        transform_stamped.transform.rotation.x = robot_rotation[0]
        transform_stamped.transform.rotation.y = robot_rotation[1]
        transform_stamped.transform.rotation.z = robot_rotation[2]
        transform_stamped.transform.rotation.w = robot_rotation[3]

        # Publish the transform
        self.robot_position_publisher.publish(transform_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = RobotPositionPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
