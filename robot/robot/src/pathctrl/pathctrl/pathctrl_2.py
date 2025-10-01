import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import numpy as np

class Controller(Node):
    def __init__(self):
        super().__init__('controller_')
        self.subscription = self.create_subscription(Imu, '/imu/data', self.listener_callback, 10)
        
        # Buoyancy publishers for each BCU
        self.bcu1_pub = self.create_publisher(Float64, '/model/bcu1c/buoyancy_engine', 10)
        self.bcu2_pub = self.create_publisher(Float64, '/model/bcu2c/buoyancy_engine', 10)
        self.bcu3_pub = self.create_publisher(Float64, '/model/bcu3c/buoyancy_engine', 10)
        self.bcu4_pub = self.create_publisher(Float64, '/model/bcu4c/buoyancy_engine', 10)
        self.pid_log = self.create_publisher(Float64, '/logpid', 10)

        # Target range for buoyancy (inferred control limits)
        self.min_buoyancy = 0.002
        self.max_buoyancy = 0.009

        # Control gains for PID controller
        self.kp = 0.01  # Proportional gain
        self.ki = 0.001  # Integral gain
        self.kd = 0.005  # Derivative gain
        
        # PID state variables
        self.integral_roll = 0.0
        self.integral_pitch = 0.0
        self.prev_roll_error = 0.0
        self.prev_pitch_error = 0.0

    def listener_callback(self, msg):
        # Read the quaternion orientation from the IMU message
        ox, oy, oz, ow = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        
        # Convert quaternion to roll, pitch, yaw for stabilization
        roll, pitch, yaw = self.quaternion_to_euler(ox, oy, oz, ow)

        # Calculate buoyancy adjustments using PID control
        self.stabilize_buoyancy(roll, pitch)

    def quaternion_to_euler(self, x, y, z, w):
        # Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        
        return roll, pitch, yaw

    def stabilize_buoyancy(self, roll, pitch):
        # PID Control for Roll
        roll_error = -roll  # We want roll to be 0
        self.integral_roll += roll_error  # Accumulate integral
        derivative_roll = roll_error - self.prev_roll_error  # Change in error
        self.prev_roll_error = roll_error  # Store error for next derivative

        # Calculate roll adjustment using PID
        roll_adj = (self.kp * roll_error) + (self.ki * self.integral_roll) + (self.kd * derivative_roll)

        # PID Control for Pitch
        pitch_error = -pitch  # We want pitch to be 0
        self.integral_pitch += pitch_error  # Accumulate integral
        derivative_pitch = pitch_error - self.prev_pitch_error  # Change in error
        self.prev_pitch_error = pitch_error  # Store error for next derivative

        # Calculate pitch adjustment using PID
        pitch_adj = (self.kp * pitch_error) + (self.ki * self.integral_pitch) + (self.kd * derivative_pitch)

        # Calculate target buoyancies, clamping within defined range
        bcu1_buoyancy = np.clip(0.005 + roll_adj + pitch_adj, self.min_buoyancy, self.max_buoyancy)
        bcu2_buoyancy = np.clip(0.005 - roll_adj + pitch_adj, self.min_buoyancy, self.max_buoyancy)
        bcu3_buoyancy = np.clip(0.005 - roll_adj - pitch_adj, self.min_buoyancy, self.max_buoyancy)
        bcu4_buoyancy = np.clip(0.005 + roll_adj - pitch_adj, self.min_buoyancy, self.max_buoyancy)

        # Publish adjusted buoyancy values
        self.bcu1_pub.publish(Float64(data=bcu1_buoyancy))
        self.bcu2_pub.publish(Float64(data=bcu2_buoyancy))
        self.bcu3_pub.publish(Float64(data=bcu3_buoyancy))
        self.bcu4_pub.publish(Float64(data=bcu4_buoyancy))
        self.pid_log.publish(Float64(data=pitch_adj))

        # Log the adjustments for debugging
        self.get_logger().info(
            f"Buoyancies set - BCU1: {bcu1_buoyancy}, BCU2: {bcu2_buoyancy}, BCU3: {bcu3_buoyancy}, BCU4: {bcu4_buoyancy}"
        )

def main(args=None):
    rclpy.init(args=args)
    controller_ = Controller()
    rclpy.spin(controller_)
    controller_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
