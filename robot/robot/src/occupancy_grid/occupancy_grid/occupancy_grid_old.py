import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from sensor_msgs_py.point_cloud2 import read_points
import numpy as np

class MapGenerator(Node):
    def __init__(self):
        super().__init__('map_generator')

        # Subscribers
        """self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/camera_path',
            self.pose_callback,
            10
        )"""
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/map_pointcloud2',
            self.pointcloud_callback,
            10
        )

        # Map publisher
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            '/occupancy_grid',
            10
        )

        # Configuration
        self.map_resolution = 0.5  # 0.5m resolution
        self.grid_size = 2  # 2x2 grid for a 1x1m table
        self.map = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)

        self.current_pose = None
        self.pointcloud = None

    """def pose_callback(self, msg):
        """"""Callback for PoseStamped messages.""""""
        self.get_logger().info(f'Received Pose: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}')
        self.current_pose = msg"""

    def pointcloud_callback(self, msg):
        """Callback for PointCloud2 messages."""
        self.get_logger().info('Received Point Cloud')
        #print(read_points(msg, skip_nans=True))
        try:
            #self.pointcloud = np.array([point[:3] for point in read_points(msg, skip_nans=True)])
            self.pointcloud = read_points(msg, skip_nans=True)
            #print(self.pointcloud)
            self.generate_map()
        except Exception as e:
            self.get_logger().error(f"Point cloud processing error: {e}")

    def generate_map(self):
        """Generates a 2D occupancy grid from the point cloud."""
        if self.pointcloud is None: #or self.current_pose is None:
            self.get_logger().warn("Missing pose or point cloud for map generation.")
            return

        # Clear the map
        self.map.fill(0)

        # Convert point cloud to grid
        for point in self.pointcloud:
            grid_x = int((point[0] + 0.5) / self.map_resolution)
            grid_y = int((point[1] + 0.5) / self.map_resolution)

            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                self.map[grid_x, grid_y] = 100  # Mark as occupied

        # Publish the map
        self.publish_map()

    def publish_map(self):
        """Publishes the 2D map as a ROS 2 OccupancyGrid message."""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.grid_size
        map_msg.info.height = self.grid_size
        map_msg.info.origin.position.x = -0.5  # Map center
        map_msg.info.origin.position.y = -0.5
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = self.map.flatten().tolist()
        self.map_publisher.publish(map_msg)
        self.get_logger().info("Published occupancy grid.")

def main(args=None):
    rclpy.init(args=args)
    node = MapGenerator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
