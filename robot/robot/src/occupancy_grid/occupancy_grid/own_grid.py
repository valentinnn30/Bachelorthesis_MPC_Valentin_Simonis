import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from sensor_msgs_py.point_cloud2 import read_points
import numpy as np
import struct
from std_msgs.msg import Header



class MapGenerator(Node):
    def __init__(self):
        super().__init__('map_generator')

        # Publishers
        self.map_publisher = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        # Subscribers
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/synthetic_pointcloud',
            self.pointcloud_callback,
            10
        )

        # Configuration
        self.map_resolution = 0.01  # Grid resolution in meters
        self.map_generated = False  # Flag to ensure only one map is published

        # Generate synthetic point cloud and publish it
        self.publish_synthetic_pointcloud()

    def generate_synthetic_pointcloud(self):
        """Generates a synthetic point cloud with the specified shape and z noise."""
        points = []

        # Add a 1x1m square (boundary points with noise in z)
        for x in np.arange(-0.5, 0.5, 0.01):  # Horizontal lines
            z_noise = np.random.uniform(-1.0, 1.0)  # Random disturbance in z
            points.append([x, -0.5, z_noise])  # Bottom boundary
            points.append([x, 0.5, z_noise])   # Top boundary
        for y in np.arange(-0.5, 0.5, 0.01):  # Vertical lines
            z_noise = np.random.uniform(-1.0, 1.0)  # Random disturbance in z
            points.append([-0.5, y, z_noise])  # Left boundary
            points.append([0.5, y, z_noise])   # Right boundary

        # Add a 0.5x0.2m rectangle in the center (representing the object with z noise)
        for x in np.arange(-0.25, 0.25, 0.01):  # Horizontal lines
            z_noise = np.random.uniform(-1.0, 1.0)  # Random disturbance in z
            points.append([x, -0.1, z_noise])  # Bottom of rectangle
            points.append([x, 0.1, z_noise])   # Top of rectangle
        for y in np.arange(-0.1, 0.1, 0.01):  # Vertical lines
            z_noise = np.random.uniform(-1.0, 1.0)  # Random disturbance in z
            points.append([-0.25, y, z_noise])  # Left of rectangle
            points.append([0.25, y, z_noise])   # Right of rectangle

        self.get_logger().info(f"Generated synthetic point cloud with {len(points)} points.")
        return points

    def publish_synthetic_pointcloud(self):
        """Publishes the synthetic point cloud as a PointCloud2 message."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        # Create a PointCloud2 message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        points = self.generate_synthetic_pointcloud()
        data = []
        for point in points:
            data += struct.pack('fff', point[0], point[1], point[2])

        pointcloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=12,  # 3 float32 values (x, y, z) * 4 bytes
            row_step=12 * len(points),
            data=bytearray(data)
        )

        # Publish the synthetic point cloud periodically
        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/synthetic_pointcloud', 10)
        self.pointcloud_publisher.publish(pointcloud_msg)
        self.get_logger().info("Published synthetic PointCloud2.")

    def pointcloud_callback(self, msg):
        """Callback for processing received PointCloud2 messages and generating the map."""
        self.get_logger().info("Received PointCloud2 message. Generating map...")

        try:
            # Extract points from the PointCloud2 message
            points = []
            for point in read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                points.append([point[0], point[1], point[2]])
            
            self.generate_map(points)

        except Exception as e:
            self.get_logger().error(f"Error processing PointCloud2: {e}")

    def generate_map(self, points):
        """Generates a 2D occupancy grid from the given points."""
        if not points:
            self.get_logger().warn("No points collected for map generation.")
            return

        # Convert collected points to a numpy array
        points = np.array(points)

        # Flatten points to 2D (use only x and y)
        flattened_points = points[:, :2]  # Only keep (x, y)

        # Recalculate bounds for the 2D points
        x_min, y_min = np.min(flattened_points, axis=0)
        x_max, y_max = np.max(flattened_points, axis=0)

        self.get_logger().info(f"Grid bounds: x_min={x_min:.2f}, x_max={x_max:.2f}, "
                               f"y_min={y_min:.2f}, y_max={y_max:.2f}")

        # Calculate grid dimensions
        grid_width = int(np.ceil((x_max - x_min) / self.map_resolution))
        grid_height = int(np.ceil((y_max - y_min) / self.map_resolution))

        self.get_logger().info(f"Grid dimensions: {grid_width} x {grid_height}")

        # Initialize the grid
        grid = np.zeros((grid_height, grid_width), dtype=np.int8)

        # Fill the grid based on points
        for point in flattened_points:
            grid_x = int((point[0] - x_min) / self.map_resolution)
            grid_y = int((point[1] - y_min) / self.map_resolution)

            if 0 <= grid_x < grid_width and 0 <= grid_y < grid_height:
                grid[grid_y, grid_x] = 100  # Note: grid[y][x] to match row-major layout

        # Publish the map
        self.publish_map(grid, float(x_min), float(y_min), grid_width, grid_height)

    def publish_map(self, grid, x_min, y_min, grid_width, grid_height):
        """Publishes the 2D map as a ROS 2 OccupancyGrid message."""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = grid_width
        map_msg.info.height = grid_height
        map_msg.info.origin.position.x = x_min
        map_msg.info.origin.position.y = y_min
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
