import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import math

class YawPIDController(Node):
    def __init__(self):
        super().__init__('yaw_pid_controller')
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)

        self.target_yaw = self.get_parameter('target_yaw').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
        self.publisher = self.create_publisher(Float32, '/spinthruster', 10)

        self.add_on_set_parameters_callback(self.param_callback)

        self.get_logger().info('Yaw PID Controller node started')
        self.get_logger().info(f'Initial parameters: target_yaw={self.target_yaw:.3f}, kp={self.kp:.3f}, ki={self.ki:.3f}, kd={self.kd:.3f}')

    def param_callback(self, params):
        for param in params:
            if param.name == 'target_yaw':
                self.get_logger().info(f'Parameter target_yaw updated: {self.target_yaw:.3f} -> {param.value:.3f}')
                self.target_yaw = param.value
            elif param.name == 'kp':
                self.get_logger().info(f'Parameter kp updated: {self.kp:.3f} -> {param.value:.3f}')
                self.kp = param.value
            elif param.name == 'ki':
                self.get_logger().info(f'Parameter ki updated: {self.ki:.3f} -> {param.value:.3f}')
                self.ki = param.value
            elif param.name == 'kd':
                self.get_logger().info(f'Parameter kd updated: {self.kd:.3f} -> {param.value:.3f}')
                self.kd = param.value
        return SetParametersResult(successful=True)

    def imu_callback(self, msg):
        # Extract yaw from quaternion
        q = msg.orientation
        #yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        yaw = 2
        # PID calculations
        error = self.angle_diff(self.target_yaw, yaw)
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        if dt == 0:
            self.get_logger().warn('dt is zero, skipping this IMU message')
            return

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(min(output, 1.0), -1.0)  # Clamp between -1 and 1

        msg_out = Float32()
        msg_out.data = float(output)
        self.publisher.publish(msg_out)

        self.get_logger().debug(
            f'yaw={yaw:.3f}, target={self.target_yaw:.3f}, error={error:.3f}, '
            f'output={output:.3f}, kp={self.kp:.3f}, ki={self.ki:.3f}, kd={self.kd:.3f}'
        )

        self.prev_error = error

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    @staticmethod
    def angle_diff(target, current):
        # Compute shortest difference between two angles
        diff = target - current
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

def main(args=None):
    rclpy.init(args=args)
    node = YawPIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()