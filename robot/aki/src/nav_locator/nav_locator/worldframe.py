import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
import math

class rovlocnav(Node):

    def __init__(self):
        super().__init__('rovloc_nav')

        # Declare parameters (keeping the IMU and frame info)
        self.declare_parameter('init_yaw', 0.0)
        self.init_yaw = self.get_parameter('init_yaw').value
        
        self.declare_parameter('use_imu', True)
        self.use_imu = self.get_parameter('use_imu').value
        
        # Current position and yaw
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        # Subscribe to world frame position and IMU data
        self.create_subscription(Vector3, '/worlframe', self.pose_array_callback, 10)
        
        if self.use_imu:
            self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Publisher for the transformed coordinates in local frame
        self.local_frame_publisher = self.create_publisher(Vector3, '/localframe_position', 10)

        self.get_logger().info('RovLocator transformed frame node started')

    def imu_callback(self, msg):
        # Extract the yaw from the IMU quaternion data
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp) - self.init_yaw  # Apply initial yaw offset

    def pose_array_callback(self, msg):
        # Get world frame position from /worldframe
        world_x = msg.x
        world_y = msg.y
        world_z = msg.z

        # Convert world coordinates to local coordinates using yaw
        local_x = world_x * math.cos(self.yaw) + world_y * math.sin(self.yaw)
        local_y = -world_x * math.sin(self.yaw) + world_y * math.cos(self.yaw)
        local_z = world_z  # Assuming z is unaffected by yaw (flat movement)

        # Create and publish the local frame position
        local_frame_msg = Vector3()
        local_frame_msg.x = local_x
        local_frame_msg.y = local_y
        local_frame_msg.z = local_z

        self.local_frame_publisher.publish(local_frame_msg)

    def main(self):
        rclpy.init()
        node = rovlocnav()
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    node = rovlocnav()
    node.main()
