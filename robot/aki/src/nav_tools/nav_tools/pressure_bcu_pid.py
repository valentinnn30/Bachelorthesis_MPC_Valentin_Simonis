import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from rcl_interfaces.msg import SetParametersResult
import numpy as np
from collections import deque


class PIDPressureController(Node):
    def __init__(self):
        super().__init__('pressure_bcu_PID')

        # Parameters
        self.declare_parameter('setpoint', 0.5)  # desired depth in meters
        self.declare_parameter('calibration', 0.0)  
        self.declare_parameter('kp_up', 50.0)  #50 PID for upward motion
        self.declare_parameter('ki_up', 3.0) #3
        self.declare_parameter('kd_up', 35.0) #35

        self.declare_parameter('kp_down', 50.0)  # 50 PID for downward motion
        self.declare_parameter('ki_down', 0.0)
        self.declare_parameter('kd_down', 35.0)  #35
        
        self.declare_parameter('help_bcu', False)  # New parameter to enable/disable support force
        self.declare_parameter('threshold', 0.0)   # New parameter for threshold range
        self.declare_parameter('help_force', 0.2)  # New parameter for help force, not used in this version
        
        self.declare_parameter('integral_limit', 2.0)  #2 max absolute value for integral term
        
        self.declare_parameter('min_output', 50)  # max absolute value for integral term
        self.declare_parameter('max_output', 100)  # max absolute value for integral term

        self.declare_parameter('compensating_force', 0.0)  # max absolute value for integral term
        self.integral_limit = self.get_parameter('integral_limit').value


        # Load parameters
        self.setpoint = self.get_parameter('setpoint').value
        self.calibration = self.get_parameter('calibration').value
        self.kp_up = self.get_parameter('kp_up').value
        self.ki_up = self.get_parameter('ki_up').value
        self.kd_up = self.get_parameter('kd_up').value

        self.kp_down = self.get_parameter('kp_down').value
        self.ki_down = self.get_parameter('ki_down').value
        self.kd_down = self.get_parameter('kd_down').value


        self.min_output = self.get_parameter('min_output').value
        self.max_output = self.get_parameter('max_output').value

        self.help_bcu = self.get_parameter('help_bcu').value
        self.threshold = self.get_parameter('threshold').value
        self.help_force = self.get_parameter('help_force').value
        self.compensating_force = self.get_parameter('compensating_force').value

        self.param_map = {
                'kp_up': 'kp_up',
                'ki_up': 'ki_up',
                'kd_up': 'kd_up',
                'kp_down': 'kp_down',
                'ki_down': 'ki_down',
                'kd_down': 'kd_down',
                'setpoint': 'setpoint',
                'calibration': 'calibration',
                'help_bcu': 'help_bcu',
                'threshold': 'threshold',
                'help_force': 'help_force',
                'integral_limit': 'integral_limit',
                'compensating_force': 'compensating_force',
                'min_output' : 'min_output',
                'max_output' : 'max_output'
            }
        self.add_on_set_parameters_callback(self.parameter_update_callback)

        # Control state
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()

        # PID output range (Int32 for BCU)

        self.neutral_output = 75

        # Publisher
        self.bcu_pub = self.create_publisher(Int32, '/bcu/manual', 10)
        self.bcu_support_pub = self.create_publisher(Float32, '/bcu/support_force', 10)


        # Subscriber
        self.create_subscription(Float32, '/depth', self.depth_callback, 10)
        self.create_subscription(Float32, '/depth_desired', self.desired_depth_callback, 10)

        # Timer to keep control loop frequency stable (e.g., 10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Buffer for latest measurement
        self.latest_depth = 0.0
        
        self.prev_depth = 0.0  # To detect movement direction

    
    def parameter_update_callback(self, params):
        for param in params:
            if param.name in self.param_map:
                setattr(self, self.param_map[param.name], param.value)
                #self.get_logger().info(f"Updated {param.name} to {param.value}")
        return SetParametersResult(successful=True)
  
    def depth_callback(self, msg):
        
        
        self.latest_depth = msg.data - self.calibration
        

    def desired_depth_callback(self, msg):
        self.setpoint = msg.data

    def control_loop(self):
        if self.latest_depth is None:
            return  # No data yet

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt == 0:
            return  # avoid division by zero

        # Calculate error
        error = self.setpoint - self.latest_depth


        if self.latest_depth - self.setpoint > 0:
            Kp, Ki, Kd = self.kp_up, self.ki_up, self.kd_up

        else:
            Kp, Ki, Kd = self.kp_down, self.ki_down, self.kd_down


        # PID calculations
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))

        derivative = (error - self.prev_error) / dt

        output = Kp * error + Ki * self.integral + Kd * derivative

        # Map to motor range (neutral is 75)
        motor_value = int((self.neutral_output+self.compensating_force) - output)

        # Clamp output
        motor_value = max(self.min_output, min(self.max_output, motor_value))

        # Publish to BCU
        msg = Int32()
        msg.data = motor_value
        self.bcu_pub.publish(msg)

        # Logging
        #self.get_logger().info(f"Depth: {self.latest_depth:.2f}, Error: {error:.2f}, Output: {motor_value}")


        # Check if help_bcu is enabled
        if self.help_bcu:
            # Calculate threshold range
            lower_bound = self.min_output + self.threshold
            upper_bound = self.max_output - self.threshold

            # Determine if motor_value is within the threshold range
            support_force_msg = Float32()
            # if motor_value <= lower_bound:
            #     support_force_msg.data = -self.help_force  # Activate support force
            if motor_value >= upper_bound:
                support_force_msg.data = self.help_force
            else:
                support_force_msg.data = 0.0  # Deactivate support force

            # Publish support force status
            self.bcu_support_pub.publish(support_force_msg)
            
        

        # Update state
        self.prev_error = error
        self.prev_depth = self.latest_depth
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = PIDPressureController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        support_force_msg = Float32()
        support_force_msg.data=0.0
        node.bcu_support_pub.publish(support_force_msg)
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
