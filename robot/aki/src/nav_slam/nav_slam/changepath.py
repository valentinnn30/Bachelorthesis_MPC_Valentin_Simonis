import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

class PathFrameIdModifier(Node):
    def __init__(self):
        super().__init__('path_frame_id_modifier')
        self.subscription = self.create_subscription(
            Path,
            '/camera_path',
            self.path_callback,
            10
        )
        self.publisher = self.create_publisher(
            Path,
            '/camera_path_visualisation',
            10
        )
        self.get_logger().info('Node initialized and ready to modify frame_id.')

    def path_callback(self, msg):
        # Create a new Path object and copy the data from the incoming message
        new_path = Path()
        new_path.header = msg.header
        new_path.header.frame_id = 'map'
        new_path.poses = []

        # Modify the frame_id of each pose and add to the new Path object
        for pose in msg.poses:
            new_pose = pose
            new_pose.header.frame_id = 'map'
            new_path.poses.append(new_pose)

        # Publish the new Path object
        self.publisher.publish(new_path)

def main(args=None):
    rclpy.init(args=args)
    node = PathFrameIdModifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()