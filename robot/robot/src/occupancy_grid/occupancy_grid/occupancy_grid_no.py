import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from sensor_msgs_py.point_cloud2 import read_points
import numpy as np
import time


class MapGenerator(Node):
    def __init__(self):
        super().__init__('map_generator')

        # Subscribers
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
        self.map_resolution = 0.01  # Grid resolution in meters
        self.collected_points = []  # List to store all point cloud data
        self.start_time = time.time()  # Record start time
        self.map_generated = False  # Flag to ensure only one map is published

    def pointcloud_callback(self, msg):
        """Callback for PointCloud2 messages."""
        # Skip further processing if the map has already been generated
        if self.map_generated:
            return

        # Collect points for 5 seconds
        if time.time() - self.start_time < 5.0:
            try:
                # Convert PointCloud2 message to a list of points
                for point in read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                    self.collected_points.append([point[0], point[1], point[2]])
                self.get_logger().info(f"Collected points, total so far: {len(self.collected_points)}")
            except Exception as e:
                self.get_logger().error(f"Point cloud processing error: {e}")
        else:
            # Stop collecting and generate the map
            self.get_logger().info("Finished collecting points, generating map...")
            self.generate_map()

    def generate_map(self):
        """Generates a 2D occupancy grid from the collected point clouds."""
        if not self.collected_points:
            self.get_logger().warn("No points collected for map generation.")
            return

        # Convert collected points to a numpy array
        points = np.array(self.collected_points)

        # Flatten points to 2D (use only x and y)
        flattened_points = points[:, :2]  # Only keep (x, y)

        # Calculate bounds for the 2D points
        x_min, y_min = np.min(flattened_points, axis=0)
        x_max, y_max = np.max(flattened_points, axis=0)

        self.get_logger().info(f"Grid bounds: x_min={x_min:.2f}, x_max={x_max:.2f}, "
                            f"y_min={y_min:.2f}, y_max={y_max:.2f}")

        # Dynamically calculate grid dimensions
        grid_width = int(np.ceil((x_max - x_min) / self.map_resolution))
        grid_height = int(np.ceil((y_max - y_min) / self.map_resolution))

        self.get_logger().info(f"Grid dimensions: {grid_width} x {grid_height}")

        # Initialize the grid dynamically
        grid = np.zeros((grid_height, grid_width), dtype=np.int8)  # Note: Grid is [height][width]

        # Map points to grid cells
        for point in flattened_points:
            grid_x = int((point[0] - x_min) / self.map_resolution)
            grid_y = int((point[1] - y_min) / self.map_resolution)

            if 0 <= grid_x < grid_width and 0 <= grid_y < grid_height:
                grid[grid_y, grid_x] = 100  # Mark the cell as occupied

        # Publish the map
        self.publish_map(grid, float(x_min), float(y_min), grid_width, grid_height)
        self.map_generated = True  # Prevent further map publishing
        self.get_logger().info("Map generation complete.")


    def publish_map(self, grid, x_min, y_min, grid_width, grid_height):
        """Publishes the 2D map as a ROS 2 OccupancyGrid message."""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = grid_width
        map_msg.info.height = grid_height
        map_msg.info.origin.position.x = x_min  # Ensure these are floats
        map_msg.info.origin.position.y = y_min  # Ensure these are floats
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = grid.flatten().tolist()
        self.map_publisher.publish(map_msg)
        self.get_logger().info("Published occupancy grid.")


def main(args=None):
    rclpy.init(args=args)
    node = MapGenerator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
