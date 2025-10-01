import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from rclpy.action import ActionClient

from slam_interface.action import ActivateSlam
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from sensor_msgs_py.point_cloud2 import read_points
import numpy as np
import time
from std_msgs.msg import Empty  # Message type for the reset topic
from std_msgs.msg import Float64MultiArray



class MapGenerator(Node):
    def __init__(self):
        super().__init__('map_generator')

        self.navigation_complete = False

        # Subscribers
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/map_pointcloud2',
            self.pointcloud_callback,
            10
        )

        # self.reset_subscriber = self.create_subscription(
        #     Empty,
        #     '/reset_cloudpoints',
        #     self.reset_points_callback,
        #     10
        # )

        # Map publisher
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            '/occupancy_grid',
            10
        )

        self.quadrant_centers_publisher = self.create_publisher(  # New publisher
            Float64MultiArray,
            '/quadrant_centers',
            10
        )

        # Add subscriber for navigation completion
        self.navigation_complete_subscriber = self.create_subscription(
            Empty, '/navigation_complete', self.navigation_complete_callback, 10)

        # Action server
        self.action_server = ActionServer(
            self, ActivateSlam, 'activateslam', self.handle_process_grid)

        # Configuration
        self.map_resolution = 0.01  # Grid resolution in meters
        self.points = []  # List to store all point cloud data
        self.start_time = time.time()  # Record start time
        self.map_generated = True  # Flag to ensure only one map is published

    def navigation_complete_callback(self, msg):
        """Callback triggered when navigation is complete."""
        self.get_logger().info("Received navigation complete signal.")
        self.navigation_complete = True

    def handle_process_grid(self, goal_handle):
        """Handles the action to reset and process the grid."""
        self.get_logger().info("Received process grid action.")
        self.reset_points_callback(None)  # Reset points
        self.get_logger().info("Reset complete. Collecting data...")

        # Simulate processing by waiting for point cloud data
        self.map_generated = False
        while not self.map_generated:
            rclpy.spin_once(self)

        self.get_logger().info("Grid processing complete.")

        # Wait for navigation to complete
        while not self.navigation_complete:
            rclpy.spin_once(self)

        self.get_logger().info("Navigation complete.")

        goal_handle.succeed()

        return ActivateSlam.Result(success=True)

    def reset_points_callback(self, msg):
        """Callback to reset the collected points."""
        self.points = []
        self.map_generated = False
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Reset collection timer
        self.get_logger().info("Collected points reset via topic!")

    def pointcloud_callback(self, msg):
        """Callback for PointCloud2 messages."""
        # Skip further processing if the map has already been generated
        if self.map_generated:
            return

        # Collect points for 5 seconds
        if time.time() - self.start_time < 5.0:
            try:
                # Extract points from the PointCloud2 message
                for point in read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                    if -0.5 <= point[0] <= 0.5 and -0.5 <= point[1] <= 0.5:
                        self.points.append([point[0], point[1], point[2]])

            except Exception as e:
                self.get_logger().error(f"Error processing PointCloud2: {e}")
        else:
            # Stop collecting and generate the map
            self.get_logger().info("Finished collecting points, generating map...")
            
            # Detect corners and divide grid
            x_min, x_max, y_min, y_max = self.detect_grid_corners(self.points)
            quadrants = self.divide_grid_into_quadrants(x_min, x_max, y_min, y_max)

            # Calculate quadrant centers
            quadrant_centers = self.calculate_quadrant_centers(x_min, x_max, y_min, y_max)

            # Publish quadrant centers
            self.publish_quadrant_centers(quadrant_centers)

            # Generate and publish the map
            self.generate_quadrant_occupancy_grid(x_min, x_max, y_min, y_max, quadrants)

    def calculate_quadrant_centers(self, x_min, x_max, y_min, y_max):
            """Calculates the center point of each quadrant."""
            x_mid = (x_min + x_max) / 2
            y_mid = (y_min + y_max) / 2

            # Center points of the quadrants
            centers = np.array([
                [(x_min + x_mid) / 2, (y_mid + y_max) / 2],  # Top-left
                [(x_mid + x_max) / 2, (y_mid + y_max) / 2],  # Top-right
                [(x_min + x_mid) / 2, (y_min + y_mid) / 2],  # Bottom-left
                [(x_mid + x_max) / 2, (y_min + y_mid) / 2],  # Bottom-right
            ]).reshape(2, 2, 2)  # Reshape into 2x2 array with (x, y) for each quadrant

            self.get_logger().info(f"Calculated quadrant centers: {centers.tolist()}")
            return centers
    
    def publish_quadrant_centers(self, centers):
        """Publishes the quadrant centers as a Float64MultiArray."""
        msg = Float64MultiArray()
        msg.data = centers.flatten().tolist()  # Convert 2D array to 1D list

        self.quadrant_centers_publisher.publish(msg)
        self.get_logger().info("Published quadrant centers.") 
          

    def detect_grid_corners(self, points):
        """Detect the outermost points in the grid (min/max x and y)."""
        points = np.array(points)
        x_min = np.min(points[:, 0])
        x_max = np.max(points[:, 0])
        y_min = np.min(points[:, 1])
        y_max = np.max(points[:, 1])
        self.get_logger().info(f"Detected grid corners: "
                               f"x_min={x_min}, x_max={x_max}, y_min={y_min}, y_max={y_max}")
        return x_min, x_max, y_min, y_max

    def divide_grid_into_quadrants(self, x_min, x_max, y_min, y_max):
        """Divide the grid into 4 equal quadrants."""
        x_mid = (x_min + x_max) / 2
        y_mid = (y_min + y_max) / 2

        quadrants = {
            "top_left": {"x_range": (x_min, x_mid), "y_range": (y_mid, y_max)},
            "top_right": {"x_range": (x_mid, x_max), "y_range": (y_mid, y_max)},
            "bottom_left": {"x_range": (x_min, x_mid), "y_range": (y_min, y_mid)},
            "bottom_right": {"x_range": (x_mid, x_max), "y_range": (y_min, y_mid)},
        }

        self.get_logger().info("Divided grid into quadrants:")
        for name, ranges in quadrants.items():
            self.get_logger().info(f"{name}: x_range={ranges['x_range']}, y_range={ranges['y_range']}")

        return quadrants

    def generate_quadrant_occupancy_grid(self, x_min, x_max, y_min, y_max, quadrants):
        """Generate an OccupancyGrid with quadrants color-coded."""
        # Calculate grid dimensions
        grid_width = int(np.ceil((x_max - x_min) / self.map_resolution))
        grid_height = int(np.ceil((y_max - y_min) / self.map_resolution))
        
        # Initialize the grid with zeros
        grid = np.zeros((grid_height, grid_width), dtype=np.int8)

        # Precompute quadrant ranges
        x_mid = (x_min + x_max) / 2
        y_mid = (y_min + y_max) / 2

        for y in range(grid_height):
            for x in range(grid_width):
                # Compute coordinates of the cell
                x_coord = x_min + x * self.map_resolution
                y_coord = y_min + y * self.map_resolution

                # Determine which quadrant the cell belongs to
                if x_coord < x_mid and y_coord >= y_mid:  # Top-left
                    grid[y, x] = 50
                elif x_coord >= x_mid and y_coord >= y_mid:  # Top-right
                    grid[y, x] = 100
                elif x_coord < x_mid and y_coord < y_mid:  # Bottom-left
                    grid[y, x] = 150
                elif x_coord >= x_mid and y_coord < y_mid:  # Bottom-right
                    grid[y, x] = 200

        # Publish the map
        self.publish_map(grid, x_min, y_min, grid_width, grid_height)
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
        map_msg.info.origin.position.x = float(x_min)  # Ensure these are floats
        map_msg.info.origin.position.y = float(y_min)  # Ensure these are floats
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
