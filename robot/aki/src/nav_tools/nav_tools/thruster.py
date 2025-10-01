import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
import numpy as np
from scipy.optimize import minimize
import serial
from rcl_interfaces.msg import SetParametersResult


class Commander(Node):
    def __init__(self):
        super().__init__('thrusterss')

        # Declare ROS parameters
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("use_bang_bang", True)
        self.declare_parameter("bang_bang_threshold", 0.15)
        self.declare_parameter("bang_bang_value", 0.2)

        # Get parameter values
        self.serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        self.baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.use_bang_bang = self.get_parameter("use_bang_bang").get_parameter_value().bool_value
        self.bang_bang_threshold = self.get_parameter("bang_bang_threshold").get_parameter_value().double_value
        self.bang_bang_value = self.get_parameter("bang_bang_value").get_parameter_value().double_value

        # Parameter map for dynamic updates
        self.param_map = {
            'serial_port': 'serial_port',
            'baud_rate': 'baud_rate',
            'use_bang_bang': 'use_bang_bang',
            'bang_bang_threshold': 'bang_bang_threshold',
            'bang_bang_value': 'bang_bang_value',
        }

        # Add dynamic parameter callback
        self.add_on_set_parameters_callback(self.parameter_update_callback)

        # Subscriptions
        self.subscription = self.create_subscription(Imu, '/imu', self.listener_callback, 10)
        self.force_subscription = self.create_subscription(Vector3, 'power', self.force_callback, 10)
        self.torque_subscription = self.create_subscription(Vector3, 'torque', self.torque_callback, 10)

        # Publishers
        self.t1pub = self.create_publisher(Float64, '/model/tethys/joint/umdrehung_1/cmd_thrust', 1)
        self.t2pub = self.create_publisher(Float64, '/model/tethys/joint/umdrehung_2/cmd_thrust', 1)
        self.t3pub = self.create_publisher(Float64, '/model/tethys/joint/umdrehung_3/cmd_thrust', 1)
        self.t4pub = self.create_publisher(Float64, '/model/tethys/joint/umdrehung_4/cmd_thrust', 1)
        self.pid_log = self.create_publisher(Float64, '/logpid', 10)

        # Force mapping matrix
        self.A = np.array([
            [0.43, 0.43, -0.87, 0.0],
            [-0.75, 0.75, 0.0, 0.0],
            [0.5, 0.5, 0.5, -1.0],
        ])

        # Init variables
        self.prev_T = None
        self.thrust_change_threshold = 0.01
        self.log = False
        self.FT_desired = np.zeros(3)

        # Tick timer for bang-bang PWM
        self.tick = 0
        self.create_timer(0.05, self.tick_callback)  # 20Hz update

        # Serial connection
        try:
            self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to serial port: {self.serial_port} at baud rate: {self.baud_rate}")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port {self.serial_port}: {e}")
            self.serial_connection = None
            return

        

    def tick_callback(self):
        self.tick += 1

    def parameter_update_callback(self, params):
        for param in params:
            if param.name in self.param_map:
                setattr(self, self.param_map[param.name], param.value)
                self.get_logger().info(f"Updated {param.name} to {param.value}")
        return SetParametersResult(successful=True)

    def listener_callback(self, msg):
        ox, oy, oz, ow = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        roll, pitch, yaw = self.quaternion_to_euler(ox, oy, oz, ow)

    def force_callback(self, msg):
        self.FT_desired = [msg.x, msg.y, msg.z]
        self.move()

    def torque_callback(self, msg):
        print("lol")
        # self.FT_desired[:3] = [msg.x, msg.y, msg.z]

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return roll, pitch, yaw

    def objective(self, T):
        return np.linalg.norm(self.A @ T - self.FT_desired) ** 2

    def move(self):
        T_initial = np.zeros(self.A.shape[1])
        bounds = [(0, 1)] * self.A.shape[1]
        result = minimize(self.objective, T_initial, bounds=bounds)
        self.T = result.x

        if self.use_bang_bang:
            for i in range(len(self.T)):
                if self.T[i] >= self.bang_bang_threshold:
                    continue  # Keep as-is
                elif self.T[i] <= 0: # maybe use some low value above 0
                    self.T[i] = 0.0  # Fully off
                else:
                    # PWM behavior
                    duty_cycle = self.T[i] / self.bang_bang_threshold
                    period = 20  # ticks per cycle (~1 second at 20Hz)
                    on_ticks = int(duty_cycle * period)
                    cycle_position = self.tick % period
                    self.T[i] = self.bang_bang_value if cycle_position < on_ticks else 0.0

        if self.prev_T is None or np.any(np.abs(self.T - self.prev_T) > self.thrust_change_threshold):
            self.get_logger().info(f"Updated Thrust Values: {self.T.tolist()}")
            self.log = True

        T_scaled = (self.T * 255).astype(int)
        
        message_bytes = bytes(T_scaled.tolist())

        if self.log:
            self.get_logger().info(f"Message to send (bytes): {message_bytes}")

        try:
            ser = self.serial_connection
            ser.write(message_bytes)
            if self.log:
                self.get_logger().info("Message sent successfully!")
        except Exception as e:
            nonsense = True

        self.prev_T = self.T.copy()
        self.log = False

        self.t1pub.publish(Float64(data=-self.T[0]))
        self.t2pub.publish(Float64(data=-self.T[1]))
        self.t3pub.publish(Float64(data=-self.T[2]))
        self.t4pub.publish(Float64(data=self.T[3]))

    def destroy_node(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            self.get_logger().info("Serial connection closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    commander_ = Commander()
    rclpy.spin(commander_)
    commander_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
