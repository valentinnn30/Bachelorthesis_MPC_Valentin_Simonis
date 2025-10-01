import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
import numpy as np

class PIDTABLE(Node):
    def __init__(self):
        super().__init__('pidtable')
        self.subscription = self.create_subscription(Vector3, '/tracked/table', self.listener_callback, 1)
        self.power = self.create_publisher(Vector3, '/power', 10)
        self.logx = self.create_publisher(Float64, '/logx', 10)
        self.logy = self.create_publisher(Float64, '/logy', 10)

        # Control gains for PID controller
        self.kp = 0.001  # Proportional gain
        self.ki = 0.000  # Integral gain
        self.kd = 0.000 # Derivative gain
        
        # PID state variables for x and y control
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_x_error = 0.0
        self.prev_y_error = 0.0
        
        # Target coordinates
        self.target_x = 400
        self.target_y = 300

    def listener_callback(self, msg):
        x, y = msg.x, msg.y
        self.get_logger().info(
            f"Buoyancies set - BCU1: {x} {y}"
        )
        # Calculate buoyancy adjustments using PID control
        xx = Float64()
        yy = Float64()
        xx.data = x
        yy.data = y

        self.logx.publish(xx)
        self.logy.publish(yy)

        self.go_table(x, y)

    def go_table(self, x, y):
        # Calculate errors for x and y
        #error_x=0
        error_x = self.target_x - x
        error_y = self.target_y - y
        
        # Proportional terms
        p_x = self.kp * error_x
        p_y = self.kp * error_y
        
        # Integral terms
        self.integral_x += error_x
        self.integral_y += error_y
        i_x = self.ki * self.integral_x
        i_y = self.ki * self.integral_y
        
        # Derivative terms
        d_x = self.kd * (error_x - self.prev_x_error)
        d_y = self.kd * (error_y - self.prev_y_error)
        
        # PID output
        power_x = p_x + i_x + d_x
        power_y = p_y + i_y + d_y
        
        # Update previous errors
        self.prev_x_error = error_x
        self.prev_y_error = error_y
        
        # Send power commands to stabilize
        power_msg = Vector3()
        power_msg.x = -power_y
        power_msg.y = -power_x
        self.get_logger().info(
            f"Force is - : {power_msg.x} {power_msg.y}"
        )
        power_msg.z = 0.0  # No need to control the z-axis for this case
        self.power.publish(power_msg)



def main(args=None):
    rclpy.init(args=args)
    controller_ = PIDTABLE()
    rclpy.spin(controller_)
    controller_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
