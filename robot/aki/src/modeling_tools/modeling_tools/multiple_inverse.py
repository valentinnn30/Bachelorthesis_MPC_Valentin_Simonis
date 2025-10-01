import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
import numpy as np
from geometry_msgs.msg import Quaternion

class MultiAprilTagPositionNode(Node):
    def __init__(self):
        super().__init__('multi_april_tag_position_node')

        # Define the target AprilTag ID to use as the origin
        self.origin_tag_id = 3  # AprilTag 1 is the origin

        # Set up the TF2 listener and buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Set up the TransformBroadcaster to publish the robot's position
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer to periodically check and update position
        self.timer = self.create_timer(0.1, self.update_position)

        # To store the robot's position (relative to the chosen tag)
        self.robot_position = None

        # Cache for transforms between tags
        self.tag_transforms = {}
        # Track visited tags to prevent infinite recursion
        self.visited_tags = set()  # To track visited tags


    def update_position(self):
        try:
            for tag_id in range(1, 6):  # Assume tag IDs 1 to 5 are used
                try:
                    # Get the transform from the camera to the tag
                    transform = self.tf_buffer.lookup_transform(
                        f"camera",  # Camera frame
                        f"tag36h11:{tag_id}",  # AprilTag frame, based on your tf names
                        rclpy.time.Time(),  # Get the latest transform
                        timeout=rclpy.duration.Duration(seconds=1.0)  # Timeout duration
                    )

                    # If the target tag is found, compute robot position
                    if tag_id == self.origin_tag_id:
                        # Inverse the transform to get the robot's position relative to the origin tag
                        robot_transform = self.inverse_transform(transform)
                        self.robot_position = robot_transform

                        # Log the position
                        self.get_logger().info(f"Robot position relative to origin tag {self.origin_tag_id}: "
                                               f"x: {self.robot_position[0]}, y: {self.robot_position[1]}, "
                                               f"z: {self.robot_position[2]}")

                        # Publish the robot's position relative to the target tag
                        self.publish_robot_transform()

                    else:
                        # Handle other tags, compute robot's position relative to the origin tag (ID 1)
                        transform_to_origin = self.get_transform_between_tags(self.origin_tag_id, tag_id)
                        if transform_to_origin:
                            # Apply the transform from the current tag to the origin tag
                            robot_position_in_origin = self.apply_transform_to_origin(transform, transform_to_origin)
                            self.get_logger().info(f"Robot position relative to tag {tag_id}: "
                                                   f"x: {robot_position_in_origin[0]}, "
                                                   f"y: {robot_position_in_origin[1]}, z: {robot_position_in_origin[2]}")

                            # Publish this position too
                            self.publish_robot_transform(robot_position_in_origin)

                    break  # Exit loop once the tag position is found

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

    def get_transform_between_tags(self, origin_tag_id, target_tag_id):
        """Compute and return the transform between the origin tag and other tags if needed"""
        # Prevent recursion by checking if we've already visited this tag pair
        if (origin_tag_id, target_tag_id) in self.tag_transforms or (target_tag_id, origin_tag_id) in self.tag_transforms:
            return self.tag_transforms.get((origin_tag_id, target_tag_id)) or self.tag_transforms.get((target_tag_id, origin_tag_id))
        
        self.get_logger().info(f"geht bis hier1")

        # Try to find an intermediate tag that already has a transform to the origin tag
        for intermediate_tag_id in range(1, 6):
            if intermediate_tag_id != origin_tag_id and intermediate_tag_id != target_tag_id:
                # Check if the intermediate tag has a known transform to the origin tag
                if (self.origin_tag_id, intermediate_tag_id) in self.tag_transforms:
                    # Avoid infinite recursion by marking the intermediate tag as visited
                    self.get_logger().info(f"geht bis hier2")
                    if intermediate_tag_id in self.visited_tags:
                        self.get_logger().info(f"geht bis hier3")
                        continue
                    
                    # Mark the intermediate tag as visited
                    self.visited_tags.add(intermediate_tag_id)

                    # Get the transform between the intermediate tag and the target tag
                    transform_to_intermediate = self.get_transform_between_tags(self.origin_tag_id, intermediate_tag_id)
                    if transform_to_intermediate:
                        # Combine the transforms to compute the final transform between the origin tag and the target tag
                        final_transform = self.combine_transforms(transform_to_intermediate, target_tag_id)
                        # Cache the result
                        self.tag_transforms[(origin_tag_id, target_tag_id)] = final_transform
                        return final_transform
        return None

    def combine_transforms(self, transform_to_origin, target_tag_id):
    
        """Combine the transform between the origin and intermediate tag with the final target transform"""
        
        # Combine the translation part of the transforms
        combined_translation = np.array([transform_to_origin.transform.translation.x,
                                        transform_to_origin.transform.translation.y,
                                        transform_to_origin.transform.translation.z])
        
        # Retrieve the transform from the target tag
        target_transform = self.get_transform_between_tags(transform_to_origin.header.frame_id, target_tag_id)
        
        # If the target transform is available, combine the translation
        if target_transform:
            target_translation = np.array([target_transform.transform.translation.x,
                                        target_transform.transform.translation.y,
                                        target_transform.transform.translation.z])
            
            # Combine the translations (this can be adjusted based on how you want to combine the translations)
            combined_translation += target_translation

            # For rotation, combine using quaternion multiplication (assuming quaternion-based transforms)
            combined_rotation = self.multiply_quaternions(
                transform_to_origin.transform.rotation,
                target_transform.transform.rotation
            )

            # Create a new transform with the combined translation and rotation
            combined_transform = TransformStamped()
            combined_transform.transform.translation.x = combined_translation[0]
            combined_transform.transform.translation.y = combined_translation[1]
            combined_transform.transform.translation.z = combined_translation[2]
            combined_transform.transform.rotation = combined_rotation
            
            # Return the combined transform
            return combined_transform

        else:
            self.get_logger().warn(f"No transform available from target tag {target_tag_id}")
            return None


    def apply_transform_to_origin(self, robot_transform, transform_to_origin):
        """Apply the transform from a tag to the origin to get the robot's position in the origin frame"""
        # Apply the transformation to the robot's position
        robot_position_in_origin = robot_transform + np.array([transform_to_origin.transform.translation.x,
                                                              transform_to_origin.transform.translation.y,
                                                              transform_to_origin.transform.translation.z])
        return robot_position_in_origin
    
    def multiply_quaternions(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1.w, q1.x, q1.y, q1.z
        w2, x2, y2, z2 = q2.w, q2.x, q2.y, q2.z
        
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

        return Quaternion(w=w, x=x, y=y, z=z)


    def publish_robot_transform(self, robot_position=None):
        """Publish the robot's position relative to the target tag"""
        if robot_position is None:
            robot_position = self.robot_position

        # Create a TransformStamped message
        transform_stamped = TransformStamped()

        # Set the header information
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = f"inverse_tag36h11:{self.origin_tag_id}"  # Parent frame (target tag)
        transform_stamped.child_frame_id = "robot_frame"  # Child frame (robot's frame)

        # Set the translation (robot's position relative to the tag)
        transform_stamped.transform.translation.x = robot_position[0]
        transform_stamped.transform.translation.y = robot_position[1]
        transform_stamped.transform.translation.z = robot_position[2]

        # Set the rotation (identity rotation for simplicity, adjust if needed)
        transform_stamped.transform.rotation.x = 0.0
        transform_stamped.transform.rotation.y = 0.0
        transform_stamped.transform.rotation.z = 0.0
        transform_stamped.transform.rotation.w = 1.0

        # Publish the transform
        self.tf_broadcaster.sendTransform(transform_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = MultiAprilTagPositionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
