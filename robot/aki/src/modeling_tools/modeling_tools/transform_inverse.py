import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
import numpy as np
from math import pi, cos, sin


class AprilTagPositionNode(Node):
    def __init__(self):
        super().__init__('april_tag_position_node')

        # Define the target AprilTag ID to use as the origin
        self.target_tag_id = 1  # Change to the ID of the tag you want to set as origin

        # Set up the TF2 listener and buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Set up the TransformBroadcaster to publish the robot's position
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer to periodically check and update position
        self.timer = self.create_timer(0.1, self.update_position)

        # To store the robot's position (relative to the chosen tag)
        self.robot_position = None

    def update_position(self):
        try:
            # Listen for transforms between the camera and AprilTags
            for tag_id in range(1, 6):  # Assume tag IDs 1 to 4 are used, adjust as needed
                try:
                    # Get the transform from the camera to the tag
                    transform = self.tf_buffer.lookup_transform(
                        f"camera",  # Camera frame
                        f"tag36h11:{tag_id}",  # AprilTag frame, based on your tf names
                        rclpy.time.Time(),  # Get the latest transform
                        timeout=rclpy.duration.Duration(seconds=1.0)  # Timeout duration
                    )

                    # If the target tag is found, compute robot position
                    if tag_id == self.target_tag_id:
                        # Inverse the transform to get the robot's position relative to the tag
                        robot_transform = self.inverse_transform(transform)
                        self.robot_position = robot_transform

                        # Log the position
                        self.get_logger().info(f"Robot position relative to tag {self.target_tag_id}: "
                                               f"x: {self.robot_position[0]}, y: {self.robot_position[1]}, "
                                               f"z: {self.robot_position[2]}")

                        # Publish the robot's position relative to the target tag
                        self.publish_robot_transform()

                        break  # Exit loop once target tag position is found
                except TransformException as e:
                    self.get_logger().warn(f"Transform failed for tag {tag_id}: {e}")

        except Exception as e:
            self.get_logger().error(f"Error while updating position: {e}")

    def inverse_transform(self, transform):
        """Inverse the transform from the camera to the tag to get the robot's position"""
        # Get the rotation matrix from the transform
        rotation = transform.transform.rotation
        # Convert quaternion to rotation matrix
        rotation_matrix = self.quaternion_to_rotation_matrix(rotation)

        # Inverse of rotation is its transpose
        rotation_matrix_inv = np.transpose(rotation_matrix)

        # Get the translation vector
        translation = transform.transform.translation
        translation_vec = np.array([translation.x, translation.y, translation.z])

        # Compute the inverse of the transform (robot's position relative to the tag)
        robot_translation = np.dot(rotation_matrix_inv, translation_vec)
        
        # Return the inverse transform (position only for now)
        return robot_translation

    def quaternion_to_rotation_matrix(self, quaternion):
        """Convert a quaternion to a 3x3 rotation matrix"""
        w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        matrix = np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x**2 + z**2), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x**2 + y**2)]
        ])
        return matrix

    def publish_robot_transform(self):
        """Publish the robot's position relative to the target tag"""
        # Create a TransformStamped message
        transform_stamped = TransformStamped()

        # Set the header information
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = f"inverse_tag36h11:{self.target_tag_id}"  # Parent frame (target tag)
        transform_stamped.child_frame_id = "robot_frame"  # Child frame (robot's frame)

        # Set the translation (robot's position relative to the tag)
        transform_stamped.transform.translation.x = self.robot_position[0]
        transform_stamped.transform.translation.y = self.robot_position[1]
        transform_stamped.transform.translation.z = self.robot_position[2]

        # Set the rotation (identity rotation for simplicity, adjust if needed)
        transform_stamped.transform.rotation.x = 0.0
        transform_stamped.transform.rotation.y = 0.0
        transform_stamped.transform.rotation.z = 0.0
        transform_stamped.transform.rotation.w = 1.0

        # Publish the transform
        self.tf_broadcaster.sendTransform(transform_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPositionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
