import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
import numpy as np
import time

class PathNavigator(Node):
    def __init__(self):
        super().__init__('path_navigator')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.path = []  # List of grid cells (row, col)
        self.resolution = 0
        self.origin = (0,0)
        self.subscription = self.create_subscription(
            OccupancyGrid,'/map', self.grid_callback,10)
        self.subscription

    def grid_callback(self,msg):
        self.resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        self.origin = (origin_x,origin_y)

        # Convert flattened data to a 2D numpy array
        grid = np.array(msg.data).reshape((height, width))

        # Log some information
        self.get_logger().info(f"Received occupancy grid of size {width}x{height}")
        self.get_logger().info(f"Resolution: {self.resolution}, Origin: ({origin_x}, {origin_y})")

        # Example: Print a section of the grid
        self.get_logger().info(f"Grid section:\n{grid[:10, :10]}")

        self.plan_path(grid)
    
    def plan_path(self, grid): #this function plans the path based on a lawnmower movement
        rows, cols = grid.shape
        
    
        for r in range(rows):
            if r % 2 != 0:
                # Left-to-right traversal for even rows
                for c in range(cols):
                    if grid[r, c] == 0:  # Only consider free cells
                        self.path.append((r, c))
            else:
                # Right-to-left traversal for odd rows
                for c in range(cols - 1, -1, -1):
                    if grid[r, c] == 0:
                        self.path.append((r, c))
        
        

    def grid_to_world(self, row, col):
        """Convert grid cell to world coordinates."""
        x = col * self.resolution + self.origin[0]
        y = row * self.resolution + self.origin[1]
        return x, y

    def send_goal(self, x, y):
        """Send a goal to Nav2."""
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0  # No rotation

        self.action_client.wait_for_server()
        goal = NavigateToPose.Goal()
        goal.pose = goal_msg
        future = self.action_client.send_goal_async(goal)
        return future

    def navigate_path(self):
        """Navigate through all waypoints in the path."""
        for (row, col) in self.path:
            x, y = self.grid_to_world(row, col)
            self.get_logger().info(f"Navigating to ({x:.2f}, {y:.2f})")
            future = self.send_goal(x, y)
            rclpy.spin_until_future_complete(self, future)  # Wait for this goal to complete
            #next grid point reached, know feeding
            time.sleep(1)  # Pause briefly before sending the next goal


def main(args=None):
    rclpy.init(args=args)

    # Example path (list of grid cells)

    # Occupancy grid resolution and origin
    #resolution = 0.5  # 50 cm per grid cell
    #origin = (-0.5, 0.5)  # Top-left corner of the grid in world coordinates

    navigator = PathNavigator()
    rclpy.spin_once(navigator)
    time.sleep(1)
    navigator.navigate_path()
     
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
