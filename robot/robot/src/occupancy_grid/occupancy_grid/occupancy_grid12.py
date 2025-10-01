import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData

class MapPointSubscriber(Node):
    def __init__(self):
        super().__init__("map_point_subscriber")
        self.subscription = self.create_subscription(
            PointCloud2, "/map_pointcloud2", self.map_point_callback, 10)
        self.publisher = self.create_publisher(OccupancyGrid, "/map", 10)
        self.map_points = []
        self.occupancy_grid = OccupancyGrid()

    def map_point_callback(self, msg):
        self.get_logger().info('Received Point Cloud')
        for point in  read_points(msg, skip_nans=True):
            self.map_points.append((point.x, point.y, point.z))

        self.project_to_2d(self.map_points, resolution=0.5)
        self.get_logger().info(f"Received {len(self.map_points)} points")


   

    def project_to_2d(self, map_points, resolution=0.5):
        """
        Project 3D points onto the XY plane and map to a 2D grid.

        Args:
            map_points (list of tuples): 3D points [(x, y, z), ...].
            resolution (float): Grid cell size in meters.

        Returns:
            numpy.ndarray: 2D occupancy grid.
        """
        # Extract X and Y coordinates
        xy_points = [(p[0], p[1]) for p in map_points]

        # Determine grid bounds
        min_x = min(p[0] for p in xy_points)
        min_y = min(p[1] for p in xy_points)
        origin_x = (min_x + max_x)/2
        origin_y = (min_y + max_y)/2
        max_x = max(p[0] for p in xy_points)
        max_y = max(p[1] for p in xy_points)

        print(min_x,min_y,max_x,max_y)

        # Calculate grid dimensions
        width = int((max_x - min_x) / resolution)
        height = int((max_y - min_y) / resolution)

        # Initialize grid
        grid = np.zeros((height, width), dtype=np.int8)

        # Grid not occupied
        for x, y in xy_points:
            grid_x = int((x - min_x) / resolution)
            grid_y = int((y - min_y) / resolution)
            grid[grid_y, grid_x] = 100  # Mark as occupied
        
        self.create_occupancy_grid(grid, resolution, origin_x, origin_y)
    
    

    def create_occupancy_grid(self, grid, resolution, origin_x, origin_y):
        """
        Convert a 2D grid to a ROS OccupancyGrid message.

        Args:
            grid (numpy.ndarray): 2D occupancy grid.
            resolution (float): Grid resolution in meters.
            origin_x, origin_y (float): Grid origin in meters.

        Returns:
            OccupancyGrid: ROS-compatible occupancy grid.
        """
        
        self.occupancy_grid.header.frame_id = "map"
        self.occupancy_grid.header.stamp = rclpy.clock.Clock().now().to_msg()

        self.occupancy_grid.info = MapMetaData()
        self.occupancy_grid.info.resolution = resolution
        self.occupancy_grid.info.width = grid.shape[1]
        self.occupancy_grid.info.height = grid.shape[0]
        self.occupancy_grid.info.origin.position.x = origin_x
        self.occupancy_grid.info.origin.position.y = origin_y
        self.occupancy_grid.info.origin.orientation.w = 1.0

        # Flatten grid and convert to ROS format
        self.occupancy_grid.data = grid.flatten().tolist()
        self.publisher.publish(self.occupancy_grid)

def main(args = None):
    rclpy.init(args=args)
    node = MapPointSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()
