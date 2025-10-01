import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Vector3
import numpy as np

class InertialNavNode(Node):
    def __init__(self):
        super().__init__('inertial_nav_node')
        
        # Subscribers and Publishers
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.path_pub = self.create_publisher(Path, '/ins_path', 10)
        self.velocity_pub = self.create_publisher(Vector3, '/ins_velocity', 10)
        self.position_pub = self.create_publisher(Vector3, '/ins_position', 10)
        self.timer = self.create_timer(0.01, self.update_position)

        # State variables
        self.acceleration = np.zeros(3)  # [ax, ay, az]
        self.velocity = np.zeros(3)  # [vx, vy, vz]
        self.position = np.array([1.555500070268718,0.007373097978648292,-2.8071228475469256]) # [px, py, pz]

        self.last_time = self.get_clock().now()

        # Path message
        self.path = Path()
        self.path.header.frame_id = "world"  # Change if needed for your TF setup

    def imu_callback(self, msg):
        # Extract linear acceleration
        self.acceleration[0] = msg.linear_acceleration.x
        self.acceleration[1] = msg.linear_acceleration.y
        self.acceleration[2] = 0   # Compensate for gravity

    def update_position(self):
        # Time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Integrate acceleration to update velocity
        self.velocity += self.acceleration * dt

        # Integrate velocity to update position
        self.position += self.velocity * dt

        # Add current position to path
        pose = PoseStamped()
        pose.header.stamp = current_time.to_msg()
        pose.header.frame_id = "world"
        pose.pose.position.x = self.position[0]
        pose.pose.position.y = -self.position[1]
        pose.pose.position.z = self.position[2]
        self.path.poses.append(pose)

        # Publish the path
        self.path.header.stamp = current_time.to_msg()
        self.path_pub.publish(self.path)
        """
        # Publish the velocity as Vector3
        velocity_msg = Vector3()
        velocity_msg.x = self.velocity[0]
        velocity_msg.y = self.velocity[1]
        velocity_msg.z = self.velocity[2]
        self.velocity_pub.publish(velocity_msg)
        """

        # Publish the position as Vector3
        position_msg = Vector3()
        position_msg.x = self.position[0]
        position_msg.y = self.position[1]
        position_msg.z = self.position[2]
        self.position_pub.publish(position_msg)

        self.get_logger().info(f"Position: {self.position}, Velocity: {self.velocity}")

def main(args=None):
    rclpy.init(args=args)
    node = InertialNavNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
