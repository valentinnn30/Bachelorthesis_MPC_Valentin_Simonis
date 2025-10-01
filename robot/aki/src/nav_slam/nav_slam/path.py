import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Empty
from geometry_msgs.msg import Vector3



from nav_msgs.msg import OccupancyGrid
import numpy as np
import time

class PathNavigator(Node):
    def __init__(self):
        super().__init__('path_navigator')
        self.get_logger().info("Path Navigator Started")

        self.action_client = ActionClient(self, NavigateToPose, 'slam_nav_pid')
        self.completion_publisher = self.create_publisher(Empty, '/navigation_complete', 10)
        self.power_publisher = self.create_publisher(Vector3, '/power', 10)
        self.path = []  # List of grid cells (row, col)
        self.resolution = 0
        self.origin = (0,0)
        self.message_received = False
        """self.subscription = self.create_subscription(
            OccupancyGrid,'/occupancy_grid', self.grid_callback,10)"""# change to new message type
        self.subscription = self.create_subscription(
            Float64MultiArray,'/quadrant_centers', self.centers_callback,10)
        
    
    def centers_callback(self,msg):#grid is 2x2 with cooridnates xy tuple per cell
        

        data = np.array(msg.data).reshape(-1,2)
        gride=[tuple(coord) for coord in data]
        L = len(gride)
        r = int(np.sqrt(L))
        grid = np.zeros((r,r), dtype=object)  # Ensure r is an integer

        count=0
        for i in range(r):
            for j in range(r):

                grid[i][j]=gride[count]
                count+=1

            
        self.get_logger().info(f"Received data as NumPy array: {grid}")
        self.plan_path(grid)
        self.message_received = True


    def plan_path(self, grid): #this function plans the path based on a lawnmower movement
        rows, cols = grid.shape
        
    
        for r in range(rows):#for varzing amount of rows and columns
            if r % 2 == 0:
                # Left-to-right traversal for even rows
                for c in range(cols):
                   
                    self.path.append((grid[r][c]))#every cell, no occupied, directly the path in x and y cooridnates
            else:
                # Right-to-left traversal for odd rows
                for c in range(cols - 1, -1, -1):
                  
                    self.path.append((grid[r][c]))

        self.get_logger().info(f"Received path: {self.path}")
        
        

    

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
        
    def wait_for_message(self):
        """Block execution until a message is received."""
        self.get_logger().info("Waiting for quadrant centers...")
        while not self.message_received:
            rclpy.spin_once(self)  # Process callbacks
            time.sleep(0.1)  # Reduce CPU usage during wait


    def navigate_path(self):
        """Navigate through all waypoints in the path."""
        for (x, y) in self.path:
            #x, y = self.grid_to_world(row, col) # already in real world coordinates
            self.get_logger().info(f"Navigating to ({x:.2f}, {y:.2f})")
            future = self.send_goal(x, y)
            rclpy.spin_until_future_complete(self, future)  # Wait for this goal to complete
            
            goal_handle = future.result()

            if not goal_handle.accepted:
                self.get_logger().error(f"Goal to ({x:.2f}, {y:.2f}) was not accepted!")
                continue

            # Wait for the result of the goal
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()

            if result.status != 3:  # 3 indicates "SUCCEEDED" in ROS 2
                self.get_logger().error(f"Failed to reach goal at ({x:.2f}, {y:.2f}). Status: {result.status}")
            else:
                self.get_logger().info(f"Successfully reached ({x:.2f}, {y:.2f})")
                self.get_logger().info("next grid point reached, now feeding")
		    #next grid point reached, now feeding
            time.sleep(0.1)  # Pause briefly before sending the next goal
        power_msg = Vector3()
        power_msg.x = 0.0
        power_msg.y = 0.0
        power_msg.z = 0.0
        self.power_publisher.publish(power_msg)#setting power to 0
        self.get_logger().info("All waypoints reached. Publishing completion signal.")

        self.completion_publisher.publish(Empty())


def main(args=None):
    rclpy.init(args=args)

    # Example path (list of grid cells)

    # Occupancy grid resolution and origin
    #resolution = 0.5  # 50 cm per grid cell
    #origin = (-0.5, 0.5)  # Top-left corner of the grid in world coordinates

    navigator = PathNavigator()
    navigator.wait_for_message()
    navigator.navigate_path()
    
    #rclpy.spin_once(navigator)
    
     
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
