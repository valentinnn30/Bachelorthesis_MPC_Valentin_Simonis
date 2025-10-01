import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from rclpy.action import ActionClient

from slam_interface.action import ActivateSlam
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from sensor_msgs_py.point_cloud2 import read_points
import numpy as np
import cv2
import time
from std_msgs.msg import Empty  # Message type for the reset topic
from std_msgs.msg import Float64MultiArray, Float32
from nav_msgs.msg import Path
from geometry_msgs.msg import Vector3
import math  # Needed for rotation

from sklearn.neighbors import NearestNeighbors
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point



class GridGenerator(Node):
    def __init__(self):
        super().__init__('grid_generator')
        self.get_logger().info("Grid Generator Node Started.")

        self.declare_parameter("use_rotated", True)
        self.rotated = self.get_parameter("use_rotated").get_parameter_value().bool_value
        
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
        self.subscription = self.create_subscription(
            Path,
            '/camera_path',
            self.path_array_callback,
            10
        )

        self.quadrant_centers_publisher = self.create_publisher(  # New publisher
            Float64MultiArray,
            '/quadrant_centers',
            10
        )

        self.rectangle_marker_publisher = self.create_publisher(
            Marker, 
            '/rectangle_marker', 
            10
        )

        self.depth_publisher = self.create_publisher(Float32, '/depth_desired', 10)
        self.depth_subscriber = self.create_subscription(Float32, '/depth', self.depth_callback, 10)

        self.goal_depth = 2.0




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
        self.depth_reached_stable = False
        self.depth_reached = False
        self.depth = 0.0
        self.origin = (0.0,0.0)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # Orientation (yaw angle)
        self.origin_saved = False


        # pid variables
        

        # --- PID configuration ---
        self.kp_x = 1.0
        self.ki_x = 0.0
        self.kd_x = 0.2

        self.kp_y = 1.0
        self.ki_y = 0.0
        self.kd_y = 0.2

        self.dt = 0.1  # 10 Hz control loop

        # PID state
        self.error_sum_x = 0.0
        self.error_sum_y = 0.0
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.yaw = 0.0  # If not available, leave at 0

        # PID target
        self.target_x = 0.0
        self.target_y = 0.0

        self.sent_depth = False

        self.z_filter=0.0

        # Publisher for force
        self.power_publisher = self.create_publisher(Vector3, '/power', 10)

        # Timer to run the PID loop
        self.pid_timer = self.create_timer(self.dt, self.timer_callback)

    def path_array_callback(self, msg):
        if len(msg.poses) == 0:
            self.get_logger().warn('Receives an emtpy Path message')
            return
        
        newest_pose = msg.poses[-1] #newest pose is at index 0 of path
        self.x = newest_pose.pose.position.x
        self.y = newest_pose.pose.position.y
        self.z = newest_pose.pose.position.z

        #orientation
        q = newest_pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def depth_callback(self, msg):
        self.depth = msg.data
        

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
        #self.start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Reset collection timer
        self.get_logger().info("Collected points reset via topic!")

    def pointcloud_callback(self, msg):
        """Callback for PointCloud2 messages."""
        # Skip further processing if the map has already been generated
        if self.map_generated:
            return

        if not self.origin_saved:
            self.origin = (self.x, self.y)
            self.target_x = self.origin[0]
            self.target_y = self.origin[1]
            self.z_filter=self.z
            self.origin_saved = True
            self.get_logger().info(f"Origin saved at: {self.origin} & {self.z}")

        if (abs(self.depth - self.goal_depth) <= 0.1):
            if not self.depth_reached:
                self.get_logger().info('depth reached')
                self.depth_reached = True
                self.threshold_start_time = time.time()
            elif time.time() - self.threshold_start_time >= 5.0 and not self.depth_reached_stable:  # Check if 5 seconds have passed
                self.get_logger().info('Target  depth reached within threshold for 5 seconds.')
                self.depth_reached_stable = True
                self.start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Reset collection timer
        else:
            self.depth_reached = False

        # Collect points for 5 seconds
        if (time.time() - self.start_time < 5.0) and self.depth_reached_stable:
            #threshold=0.8 # at swarm keep it bounded
            #threshold=1.0 
            threshold=100.0 # in opfikon set to infinity
            #z_threshold=1.3
            z_threshold=10.0 # set to infinity
            
            try:
                # Extract points from the PointCloud2 message
                for point in read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                    if (-threshold + self.origin[0]) <= point[0] <= (threshold + self.origin[0]) and (-threshold + self.origin[1]) <= point[1] <= (threshold + self.origin[1]) and point[2]<=(self.z_filter+z_threshold): #all points within 1m radius of the current position
                        self.points.append([point[0], point[1], point[2]])

                

            except Exception as e:
                self.get_logger().error(f"Error processing PointCloud2: {e}")
        elif self.depth_reached_stable:
            # Stop collecting and generate the map
            self.get_logger().info("Finished collecting points, generating map...")

            # Convert points to numpy array
            points_array = np.array(self.points)

            # the reason for the following is to guarantee that only the points belonging to the table are used

            # 1. Calculate z_center of all collected points
            """x_mean = np.mean(points_array[:, 0])
            y_mean = np.mean(points_array[:, 1])
            
            distances = np.sqrt((points_array[:, 0] - x_mean)**2 + (points_array[:, 1] - y_mean)**2)
            nearest_index = np.argmin(distances)
            z_center = points_array[nearest_index, 2]  # z value of the nearest point
            y_center = points_array[nearest_index, 1]  # y value of the nearest point
            x_center = points_array[nearest_index, 0]  # x value of the nearest point

            self.get_logger().info(f"Closest point z_center: {z_center} at position {x_center, y_center}")

            # 2. Filter points based on z_center
            filtered_points = []
            z_threshold = 2.0 # Adjust this threshold as needed
            #z_center = 1.5 # Tune harcoded value for actual distance table to robot, adjust threshold to small value (0.2 f.e.)
            for x, y, z in points_array:
                if abs(z- z_center) <= z_threshold:
                    filtered_points.append([x, y, z])"""

            #self.get_logger().info(f"Number of points after filtering: {len(filtered_points)}")
            #self.publish_filtered_points(filtered_points)

            # Densest region filter
            k = 10
            xy_array = points_array[:, :2]
            nn = NearestNeighbors(n_neighbors=k)
            nn.fit(xy_array)

            distances, _ = nn.kneighbors(xy_array)
            avg_distances = np.mean(distances, axis=1)
            i_max = np.argmin(avg_distances)  # small avg distance = dense region
            center = points_array[i_max]

            # z-cooridnate bei dem rosbag mit 1.4 pressure bei 0.74 für kleiner tisch/ 1.07 für mittel und 1.3 für gross -> der alte filter ist alte nicht suitable
            self.get_logger().info(f"Densest center point: ({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f})")
            z_center = center[2]
            z_threshold = 0.01 # 0.05 oder 0.10, 0.05 bissi präziser aber kleiner
            # rotation sehr gut, bisschen verschoben immer. bei kleinem perfekt
            core_points = points_array[np.abs(points_array[:, 2] - z_center) < z_threshold]


            if self.rotated:
                #corners = self.find_rotated_rectangle(filtered_points)
                corners = self.find_rotated_rectangle(core_points)
                sorted_corners = self.sort_corners_clockwise(corners)

                # Outer green square
                outer_marker = self.create_rectangle_marker(sorted_corners, ns="rectangle", marker_id=0, color=(0.0, 1.0, 0.0, 1.0))
                self.rectangle_marker_publisher.publish(outer_marker)

                quadrant_centers = self.divide_rectangle_into_subcenters(sorted_corners)
                self.publish_quadrant_centers(quadrant_centers) # used function

                # Inner blue square (drive targets)
                drive_marker = self.create_rectangle_marker(quadrant_centers, ns="drive_targets", marker_id=1, color=(0.0, 0.0, 1.0, 1.0))
                self.rectangle_marker_publisher.publish(drive_marker)
                self.depth_reached_stable = False  # Reset depth reached flag
                self.depth_reached = False  # Reset depth reached flag


            else:
            
            
                # Detect corners and divide grid
                x_min, x_max, y_min, y_max = self.detect_grid_corners(filtered_points) # used function
                quadrants = self.divide_grid_into_quadrants(x_min, x_max, y_min, y_max)

                # Calculate quadrant centers
                quadrant_centers = self.calculate_quadrant_centers(x_min, x_max, y_min, y_max) # used function

                # Publish quadrant centers
                self.publish_quadrant_centers(quadrant_centers) # used function

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
    
    def find_rotated_rectangle(self, points_array):
        # Convert input
        points_array = np.array(points_array)
        xy_points = points_array[:, :2].astype(np.float32)

        # Get the min-area rectangle
        rect = cv2.minAreaRect(xy_points)
        center, (width, height), angle = rect

        # Compute the size of the square: take the max side
        square_size = min(width, height)

        # Replace width and height with square_size
        square_rect = (center, (square_size, square_size), angle)

        # Get the square corners
        square_box = cv2.boxPoints(square_rect)
        return np.array(square_box)  # shape (4, 2)

    
    def sort_corners_clockwise(self, corners):
        """
        Sort corners into order: top-left, top-right, bottom-right, bottom-left.
        Assumes corners is a (4,2) array of (x,y).
        """
        # Calculate center point
        center = np.mean(corners, axis=0)

        # Calculate angle of each point relative to center
        angles = np.arctan2(corners[:,1] - center[1], corners[:,0] - center[0])

        # Sort by angle (counterclockwise), but flip to get clockwise
        sort_order = np.argsort(-angles)

        sorted_corners = corners[sort_order]

        return sorted_corners

    def divide_rectangle_into_subcenters(self, corners):
        """
        Given 4 corners (top-left, top-right, bottom-right, bottom-left),
        divide into 4 equal subsquares and find their centers.
        
        corners: numpy array of shape (4, 2)
        
        Returns: numpy array of shape (4, 2) -> 4 centers
        Order: top-left, top-right, bottom-right, bottom-left
        """
        # Corners
        tl, tr, br, bl = corners

        # Midpoints of edges
        top_middle = (tl + tr) / 2
        right_middle = (tr + br) / 2
        bottom_middle = (br + bl) / 2
        left_middle = (bl + tl) / 2

        # Center of rectangle
        center = (tl + br) / 2

        # Centers of 4 subsquares
        center_top_left = (tl - center) / 3.5 + center
        center_top_right = (tr - center) / 3.5 + center
        center_bottom_right = (br - center) / 3.5 + center
        center_bottom_left = (bl - center) / 3.5 + center

        # Return centers in order
        centers = np.array([
            center_top_left,
            center_top_right,
            center_bottom_right,
            center_bottom_left
        ])

        self.map_generated = True  # Prevent further map publishing
        self.get_logger().info("Map generation complete.")
        
        return centers
    def publish_filtered_points(self, points):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "filtered_points"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for x, y, z in points:
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.0
            marker.points.append(pt)

        self.rectangle_marker_publisher.publish(marker)



    def publish_quadrant_centers(self, centers):
        """Publishes the quadrant centers as a Float64MultiArray."""
        msg = Float64MultiArray()
        msg.data = centers.flatten().tolist()  # Convert 2D array to 1D list

        self.quadrant_centers_publisher.publish(msg)
        self.get_logger().info("Published quadrant centers.") 


    def create_rectangle_marker(self, corners, frame_id="map", ns="rectangle", marker_id=0, color=(0.0, 1.0, 0.0, 1.0)):
        self.get_logger().info(f"Creating rectangle marker in namespace '{ns}' with ID {marker_id}.")

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01  # line thickness

        # Set color
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        self.get_logger().info(f"Marker color: r={color[0]}, g={color[1]}, b={color[2]}, a={color[3]}")

        # Add corners and close the loop
        for i in range(4):
            pt = Point()
            pt.x = float(corners[i][0])
            pt.y = float(corners[i][1])
            pt.z = 0.0
            marker.points.append(pt)
            self.get_logger().info(f"Added corner {i}: x={pt.x}, y={pt.y}, z={pt.z}")

        # Close loop
        pt = Point()
        pt.x = float(corners[0][0])
        pt.y = float(corners[0][1])
        pt.z = 0.0
        marker.points.append(pt)
        self.get_logger().info(f"Closed loop with first corner: x={pt.x}, y={pt.y}, z={pt.z}")

        self.get_logger().info("Rectangle marker creation complete.")
        return marker


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

    def timer_callback(self):
        if self.map_generated or not self.origin_saved:
            return  # Skip PID if map is done or origin not yet set
        
        #if not self.sent_depth:
        bcu_msg = Float32()
        # self.goal_depth = 1.8
        bcu_msg.data = self.goal_depth
        self.depth_publisher.publish(bcu_msg)
        #self.sent_depth = True


        # Calculate errors
        error_x = self.target_x - self.x
        error_y = self.target_y - self.y

        # Proportional terms
        proportional_x = self.kp_x * error_x
        proportional_y = self.kp_y * error_y

        # Integral terms
        self.error_sum_x += error_x * self.dt
        self.error_sum_y += error_y * self.dt
        integral_x = self.ki_x * self.error_sum_x
        integral_y = self.ki_y * self.error_sum_y

        # Derivative terms
        derivative_x = self.kd_x * (error_x - self.last_error_x) / self.dt
        derivative_y = self.kd_y * (error_y - self.last_error_y) / self.dt
        self.last_error_x = error_x
        self.last_error_y = error_y

        # Compute PID outputs for world frame forces
        world_force_x = proportional_x + integral_x + derivative_x
        world_force_y = proportional_y + integral_y + derivative_y

        # Rotate forces to robot's local frame
        force_x = world_force_x * math.cos(self.yaw) + world_force_y * math.sin(self.yaw)
        force_y = -world_force_x * math.sin(self.yaw) + world_force_y * math.cos(self.yaw)

        # Publish force as power
        power_msg = Vector3()
        power_msg.x = force_x
        power_msg.y = -force_y  # check if sign inversion is intentional
        power_msg.z = 0.0
        self.power_publisher.publish(power_msg)

        self.get_logger().info(
            f"PID Output: x={force_x:.3f}, y={force_y:.3f}, Errors -> x: {error_x:.3f}, y: {error_y:.3f}, Position -> x: {self.x:.3f}, y: {self.y:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = GridGenerator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
