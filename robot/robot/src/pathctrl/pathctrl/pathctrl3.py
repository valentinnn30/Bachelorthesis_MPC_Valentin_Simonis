import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Vector3
import math


class MoveToTarget(Node):
    def __init__(self):
        super().__init__('move_to_target')

        # Target latitude
        self.target_lat = 0.000008532543604559196
        
        # Current position
        self.current_lat = 0.0

        # PID parameters
        self.kp = 500000.0  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.01  # Derivative gain

        self.integral = 0.0  # Integral term accumulator
        self.prev_error = 0.0  # Previous error for derivative term

        self.yaw = 0.0  # Orientation (yaw angle)

        # Sampling time (in seconds)
        self.dt = 0.1

        # Create publishers
        self.power_publisher = self.create_publisher(Vector3, '/power', 10)

        # Create subscription to /navsat for GPS data
        self.navsat_subscription = self.create_subscription(
            NavSatFix,
            '/navsat',
            self.navsat_callback,
            10
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        
        self.timer = self.create_timer(self.dt, self.timer_callback)  # Timer to publish power

    def navsat_callback(self, msg):
        self.current_lat = msg.latitude

    def imu_callback(self, msg):
        # Extract yaw from quaternion
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def timer_callback(self):
        # Compute error
        error = self.target_lat - self.current_lat

        # Calculate PID terms
        self.integral += error * self.dt  # Integral term
        derivative = (error - self.prev_error) / self.dt  # Derivative term
        self.prev_error = error  # Update previous error

        # PID output
        world_force_x = self.kp * error + self.ki * self.integral + self.kd * derivative
        world_force_y = 0.0  # Assuming no longitudinal force needed
        world_force_x = 5.0
        force_x = world_force_x * math.cos(self.yaw) + world_force_y * math.sin(self.yaw)
        force_y = -world_force_x * math.sin(self.yaw) + world_force_y * math.cos(self.yaw)
        # Cap the PID output between -1 and 1
        force_x = max(-1.0, min(1.0, force_x))
        force_y =  max(-1.0, min(1.0, force_y))
        force_z = 0.0  # Assuming no vertical control

        # Create and publish power message
        power_msg = Vector3()
        power_msg.x = force_x
        power_msg.y = force_y
        power_msg.z = force_z

        self.power_publisher.publish(power_msg)
        self.get_logger().info(f"Published power: x={force_x} (PID output), y={force_y}, z={force_z}")


def main(args=None):
    rclpy.init(args=args)
    node = MoveToTarget()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
