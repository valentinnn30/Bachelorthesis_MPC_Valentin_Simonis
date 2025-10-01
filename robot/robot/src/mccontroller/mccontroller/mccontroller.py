import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray
#47-99 9600
class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.subscription = self.create_subscription(
            PoseArray,
            '/model/own/pose',
            self.pose_array_callback,
            10
        )
        self.path_publisher = self.create_publisher(Path, '/robot_path', 10)
        self.path = Path()
        self.get_logger().info('Subscribed to /model/own/pose and publishing path to /robot_path')

    def pose_array_callback(self, msg):
        # Select the pose to be added (e.g., the 7th pose in the array)
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "world"  # Set frame to RViz frame (e.g., "map" or "world")
        pose_stamped.pose = msg.poses[6]

        # Print the x, y, z components of the pose
        pose = msg.poses[6].position
        #self.get_logger().info(f'Pose x: {pose.x}, y: {pose.y}, z: {pose.z}')

        # Add the pose to the path
        self.path.poses.append(pose_stamped)

        # Update the path header and publish it
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.header.frame_id = "world"
        self.path_publisher.publish(self.path)
        #self.get_logger().info(f'Updated path with {len(msg.poses)} poses')

def main():
    rclpy.init()
    node = PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
