import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math
from rcl_interfaces.msg import SetParametersResult


class OrientationPIDNode(Node):
    def __init__(self):
        super().__init__('orientation_pid_node')

        # PID parameters
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('desired_orientation_z', 0.0)

        self.kp = self.get_parameter('kp').value  # Proportional gain
        self.ki = self.get_parameter('ki').value  # Integral gain
        self.kd = self.get_parameter('kd').value  # Derivative gain
        self.desired_orientation_z = self.get_parameter('desired_orientation_z').value

        self.param_map = {
            'kp': 'kp',
            'ki': 'ki',
            'kd': 'kd',
            'desired_orientation_z': 'desired_orientation_z'
            }

        self.add_on_set_parameters_callback(self.parameter_update_callback)
        
        # PID state
        self.prev_error = 0.0 # Previous error
        self.integral = 0.0 # sum of errors

        # Time tracking
        self.prev_time = self.get_clock().now()
        

        # Subscribers and publishers
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        self.publisher = self.create_publisher(Vector3, '/torque', 10)
    
    def parameter_update_callback(self, params):
        for param in params:
            if param.name in self.param_map:
                setattr(self, self.param_map[param.name], param.value)
                self.get_logger().info(f"Updated {param.name} to {param.value}")
        return SetParametersResult(successful=True)

    def imu_callback(self, msg: Imu):
        # Extract the current orientation (roll, pitch, yaw) from the IMU quaternion
        q = msg.orientation
        _, _, yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)

        # Calculate the error (difference from desired orientation)
        error = self.desired_orientation_z - yaw

        # Calculate time delta
        # this is necessary, since now no timer is used to run the function but the imu callback,  which is not called at a fixed rate
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9 
        self.prev_time = current_time

        # PID calculations
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        # Compute the torque around the Z-axis
        torque_z = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Publish the torque vector
        torque_vector = Vector3()
        torque_vector.x = 0.0
        torque_vector.y = 0.0
        torque_vector.z = -10*torque_z  #must be a minus, not sure if mistake in matrix or why exactly
        self.publisher.publish(torque_vector)

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = OrientationPIDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
