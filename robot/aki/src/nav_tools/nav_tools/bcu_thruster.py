import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from rcl_interfaces.msg import SetParametersResult
from scipy.optimize import minimize

import serial
import struct
import time
import numpy as np

class BcuPidTestNode(Node):
    def __init__(self):
        super().__init__('bcu_pid_test_node')
        self.get_logger().info('BCU PID Test Node has been started.')
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)#baud-rate not clear yet
        #self.declare_parameter("serial_port_thruster", "/dev/ttyACM1")

        #PID
        self.declare_parameter('pid_p', 1.0)
        self.declare_parameter('pid_i', 0.0)
        self.declare_parameter('pid_d', 0.0)
        

        # Get their values (overwrite in output)
        self.pid_p = self.get_parameter('pid_p').get_parameter_value().double_value
        self.pid_i = self.get_parameter('pid_i').get_parameter_value().double_value
        self.pid_d = self.get_parameter('pid_d').get_parameter_value().double_value
        # Get parameters
        #serial_port_thruster = self.get_parameter("serial_port_thruster").get_parameter_value().string_value
        serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.param_map = {
            'serial_port': 'serial_port',
            #'serial_port_thruster': 'serial_port_thruster',
            'baud_rate': 'baud_rate',
            'pid_p': 'pid_p',
            'pid_i': 'pid_i',
            'pid_d': 'pid_d'
        }
        self.add_on_set_parameters_callback(self.parameter_update_callback)
        self.depth_subscription = self.create_subscription(Float32, '/depth_desired', self.handle_bcu_message, 10)
        self.depth_publisher = self.create_publisher(Float32, '/depth', 10)
        self.force_subscription = self.create_subscription(Vector3, 'power', self.force_callback, 10)

        # Force mapping matrix
        self.A = np.array([
            [0.43, 0.43, -0.87, 0.0],
            [-0.75, 0.75, 0.0, 0.0],
            [0.5, 0.5, 0.5, -1.0],
        ])
        self.FT_desired = np.zeros(3)
        self.timer_write = self.create_timer(0.1, self.write_serial) # 100 Hz

        try:
            self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=1)
            #self.serial_connection_thruster = serial.Serial(serial_port_thruster, baud_rate, timeout=1)
            self.get_logger().info(f"Connected to serial port: {serial_port} at baud rate: {baud_rate}")
            #self.get_logger().info(f"Connected to serial port: {serial_port_thruster} at baud rate: {baud_rate}")

        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port {serial_port}: {e}")
            self.serial_connection = None
            return
        
        


        self.byte1 = 0 
        self.byte2 = 0 
        self.byte3 = 0 
        self.byte4 = 0 
        self.byte5 = 0 
        self.byte6 = 0 
        self.byte7 = 0
        self.byte8 = 0
        """self.byte9 = 0
        self.byte10 = 0
        self.byte11 = 0
        self.byte12 = 0"""
        self.depth = 0.0

    def parameter_update_callback(self, params):
        for param in params:
            if param.name in self.param_map:
                setattr(self, self.param_map[param.name], param.value)
                self.get_logger().info(f"Updated {param.name} to {param.value}")
        return SetParametersResult(successful=True)   
    
    def force_callback(self, msg):
        self.FT_desired = [msg.x, msg.y, msg.z]
        self.move()

    def objective(self, T):
        return np.linalg.norm(self.A @ T - self.FT_desired) ** 2

    def move(self):
        T_initial = np.zeros(self.A.shape[1])
        bounds = [(0, 1)] * self.A.shape[1]
        result = minimize(self.objective, T_initial, bounds=bounds)
        self.T = result.x

       
        self.get_logger().info(f"Updated Thrust Values: {self.T.tolist()}")


        T_scaled = (self.T * 255).astype(int)
        thrust_values = T_scaled.tolist()
        self.byte1 = thrust_values[0] #& 0xFF  
        self.byte2 = thrust_values[1] #& 0xFF
        self.byte3 = thrust_values[2] #& 0xFF
        self.byte4 = thrust_values[3] #& 0xFF

        


    def handle_bcu_message(self, msg):
        """
        Callback function for handling messages from /bcu topic.
        Sends the received value over the serial connection.
        """
        
        x = msg.data
        x = x * 100
        x = int(round(x)) # first use rounding, integer always rounds down!
        x_encoded = max(0, min(x, 65535))
        
        #split into two bytes
        self.byte5 = x_encoded #& 0xFF
        #self.byte6 = (x_encoded >> 8) & 0xFF
        

        """#to allow negativ values: 
        x = msg.data
        x = int(x * 100)  # scale and convert to integer

        # Handle signed 16-bit range: -32768 to 32767
        if x < -32768 or x > 32767:
            print(f"Warning: value {x} out of range for int16, clipping.")
            x = max(-32768, min(x, 32767))

        # Convert to unsigned 16-bit two's complement
        x_encoded = x  

        # Split into two bytes (little-endian)
        self.byte1 = x_encoded & 0xFF         # low byte
        self.byte2 = (x_encoded >> 8) & 0xFF  # high byte"""
        
       

    def write_serial(self):
        pid_p_int = int(self.pid_p * 1000)
        pid_i_int = int(self.pid_i * 1000)
        pid_d_int = int(self.pid_d * 1000)
        
        self.byte6 = pid_p_int 
        self.byte7 = pid_i_int 
        self.byte8 = pid_d_int 
        
        """self.byte7 = pid_p_int & 0xFF
        self.byte8 = (pid_p_int >> 8) & 0xFF
        self.byte9 = pid_i_int & 0xFF
        self.byte10 = (pid_i_int >> 8) & 0xFF
        self.byte11 = pid_d_int & 0xFF
        self.byte12 = (pid_d_int >> 8) & 0xFF"""
        
        
        #if self.serial_connection and self.serial_connection.is_open:
        #if self.serial_connection:
        try:
            message_bytes = bytes(12)
            #message_bytes = bytes([self.byte1, self.byte2, self.byte3, self.byte4, self.byte5, self.byte6, self.byte7, self.byte8, self.byte9, self.byte10, self.byte11, self.byte12])
            message_bytes = bytes([self.byte1, self.byte2, self.byte3, self.byte4]) + struct.pack('<HHHH', self.byte5, self.byte6, self.byte7, self.byte8)
            self.serial_connection.write(message_bytes)
            self.serial_connection.flush()
            """message_bytes_thruster = bytes([self.byte1, self.byte2, self.byte3, self.byte4]) 
            self.serial_connection_thruster.write(message_bytes_thruster)
            self.serial_connection.flush()"""

            #log debug     
            self.get_logger().info(f"Message to send (bytes): {message_bytes}")
            self.get_logger().info(f"Message sent successfully!")
            if self.serial_connection.in_waiting:
                
                self.serial_connection.write(message_bytes)
                self.serial_connection.flush()

                # Wait a tiny bit for STM to respond (tune as needed)
                time.sleep(0.02)

                start_time = time.time()
                lines = []
                while len(lines) < 3 and (time.time() - start_time < 0.5):
                    line = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        lines.append(line)

                if len(lines) == 3:
                    self.get_logger().info(f"vals:{lines[0]}")
                    self.get_logger().info(f"dep:{lines[1]}")
                    self.get_logger().info(f"upd:{lines[2]}")
                else:
                    self.get_logger().warn(f"Timeout! Got {len(lines)} lines: {lines}")

    

                # reponse1 = self.serial_connection.readline().decode('utf-8', errors='ignore')
                # self.get_logger().info(f"values:{reponse1.strip()}")
                # response2 = self.serial_connection.readline().decode('utf-8', errors='ignore')
                # #depth_str = response2.strip()
                # self.get_logger().info(f"depth:{response2.strip()}")
                # response4 = self.serial_connection.readline().decode('utf-8', errors='ignore')
                # self.get_logger().info(f"update:{response4.strip()}")
                """import re
                match = re.search(r'-?\d+\.\d+', depth_str)
                if match:
                self.depth = float(match.group(0))
                
                # Publish it
                msg = Float32()
                msg.data = self.depth
                self.depth_publisher.publish(msg)
                self.get_logger().info(f"Published depth: {self.depth}")"""

            else:
                self.get_logger().warn("No serial data value")

        except Exception as e:
            nonsense = True    
        """else:
            self.get_logger().warn("Serial connection is not open.")"""

def main(args=None):
    rclpy.init(args=args)
    node = BcuPidTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down BCU PID Test Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()