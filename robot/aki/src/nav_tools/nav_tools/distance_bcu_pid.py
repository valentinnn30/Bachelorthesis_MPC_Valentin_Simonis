import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from rcl_interfaces.msg import SetParametersResult
import numpy as np
from collections import deque


class PIDBCUController(Node):
    def __init__(self):
        super().__init__('pid_bcu_controller')

        # Parameters
        self.declare_parameter('setpoint', 0.5)  # desired distance in meters
        self.declare_parameter('kp', 100.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 20.0)

        # Load parameters
        self.setpoint = self.get_parameter('setpoint').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        self.param_map = {
            'kp': 'kp',
            'ki': 'ki',
            'kd': 'kd',
            'setpoint': 'setpoint'
        }
        self.add_on_set_parameters_callback(self.parameter_update_callback)

        # Control state
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()

        # PID output range (Int32 for BCU)
        self.min_output = 50
        self.max_output = 100
        self.neutral_output = 75

        # Publisher
        self.bcu_pub = self.create_publisher(Int32, '/bcu/manual', 10)


        # Subscriber
        self.create_subscription(Float32, 'ultrasonic_distance', self.distance_callback, 10)
        self.create_subscription(Float32, '/desired_distance', self.desired_distance_callback, 10)

        # Timer to keep control loop frequency stable (e.g., 10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Buffer for latest measurement
        self.latest_distance = None
        self.values = deque(maxlen=10) #calculate mean of last 0.5 seconds

    
    def parameter_update_callback(self, params):
        for param in params:
            if param.name in self.param_map:
                setattr(self, self.param_map[param.name], param.value)
                #self.get_logger().info(f"Updated {param.name} to {param.value}")
        return SetParametersResult(successful=True)
  
    def distance_callback(self, msg):
        if msg.data == 0.0:
            return  # Ignore zero distance
        self.values.append(msg.data)
        # if len(self.values) == 10:  
        # Always compute mean, otherwise at the beginning mean is zero
        self.latest_distance = np.mean(self.values)
        #self.get_logger().info(f"Mean of last 10 values: {self.latest_distance:.2f}")
        #self.latest_distance = msg.data

    def desired_distance_callback(self, msg):
        self.setpoint = msg.data

    def control_loop(self):
        if self.latest_distance is None:
            return  # No data yet

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt == 0:
            return  # avoid division by zero

        # Calculate error
        error = self.setpoint - self.latest_distance

        # PID calculations
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Map to motor range (neutral is 75)
        motor_value = int(self.neutral_output + output)

        # Clamp output
        motor_value = max(self.min_output, min(self.max_output, motor_value))

        # Publish to BCU
        msg = Int32()
        msg.data = motor_value
        self.bcu_pub.publish(msg)

        # Logging
        self.get_logger().info(f"Distance: {self.latest_distance:.2f}, Error: {error:.2f}, Output: {motor_value}")
            
        

        # Update state
        self.prev_error = error
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = PIDBCUController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
