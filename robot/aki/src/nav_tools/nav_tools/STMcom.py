import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32, Bool  # Import the Int32 message type
import numpy as np
from scipy.optimize import nnls
from scipy.optimize import minimize
import serial
import struct
import json
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup # if all callback function should run in parallel
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray



class STMcommunication(Node):
    def __init__(self):
        super().__init__('STM_communication_node')

        self.cb_group_thruster = ReentrantCallbackGroup()
        self.cb_group_spinthruster = ReentrantCallbackGroup()
        self.cb_group_food = ReentrantCallbackGroup()
        self.cb_group_depth = ReentrantCallbackGroup()
        self.cb_group_timer = ReentrantCallbackGroup()



        
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        #self.declare_parameter("baud_rate", 115200)#baud-rate not clear yet
        self.declare_parameter("baud_rate", 2000000)

        #PID
        self.declare_parameter('pid_p', 1.0)
        self.declare_parameter('pid_i', 0.0)
        self.declare_parameter('pid_d', 0.0)

        #Threshold for pump values
        self.declare_parameter('pump_threshold', 255)

        self.declare_parameter('manual_mode', 1)
        self.declare_parameter('calibration_value', 0)
        

        # Get their values (overwrite in output)
        self.pid_p = self.get_parameter('pid_p').get_parameter_value().double_value
        self.pid_i = self.get_parameter('pid_i').get_parameter_value().double_value
        self.pid_d = self.get_parameter('pid_d').get_parameter_value().double_value

        self.manual_mode = self.get_parameter('manual_mode').get_parameter_value().integer_value
        self.calibration_value = self.get_parameter('calibration_value').get_parameter_value().integer_value


        self.pump_threshold = self.get_parameter('pump_threshold').get_parameter_value().integer_value
        # Get parameters
        serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.param_map = {
            'serial_port': 'serial_port',
            'baud_rate': 'baud_rate',
            'pid_p': 'pid_p',
            'pid_i': 'pid_i',
            'pid_d': 'pid_d',
            'pump_threshold':'pump_threshold',
            'manual_mode':'manual_mode',
            'calibration_value':'calibration_value'
        }
        self.add_on_set_parameters_callback(self.parameter_update_callback)


        # Initialize serial connection
        try:
            self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f"Connected to serial port: {serial_port} at baud rate: {baud_rate}")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port {serial_port}: {e}")
            self.serial_connection = None
            return
        
        self.log_locator_pid_publisher = self.create_publisher(Float32MultiArray, '/log/STMThruster', 10)
        self.log_bcupbulisher = self.create_publisher(Int32, '/log/bcu', 10)
   
        self.publisher_distance = self.create_publisher(Float32, 'ultrasonic_distance', 10)
        self.publisher_pressure = self.create_publisher(Float32, 'depth', 10)
        self.publisher_feeding_status = self.create_publisher(Int32, '/feeding_result', 10)
        self.depth_subscription = self.create_subscription(Float32, '/depth_desired', self.handle_bcu_message, 10, callback_group=self.cb_group_depth)
        self.force_subscription = self.create_subscription(Vector3, 'power', self.force_callback, 10, callback_group=self.cb_group_thruster)
        self.food_subscription = self.create_subscription(Int32, '/feeding_command', self.handle_feeding_command, 10, callback_group=self.cb_group_food)
        self.spin_subscription = self.create_subscription(Float32, '/spinthruster', self.spin_callback, 10, callback_group=self.cb_group_spinthruster)
        self.bcu_subscription = self.create_subscription(Int32, '/bcu/manual', self.manual_bcu_handler, 10, callback_group=self.cb_group_depth)
        self.imu_publisher = self.create_publisher(Imu, "/imu", 10)
        self.sos_publisher = self.create_publisher(Int32, "/leakage", 10)
        self.support_force_subscription = self.create_subscription(Float32, '/bcu/support_force', self.support_force_callback, 10, callback_group=self.cb_group_thruster)
        self.servoreset_subscription = self.create_subscription(Bool, '/servo_reset', self.handle_servo_reset, 10)
        self.resetimu_subscription = self.create_subscription(Bool, '/reset_imu', self.handle_IMU_reset, 10)


        # Combined force and torque matrix
        """self.A = np.array([
            [0.43, 0.43, -0.87, 0.0],          # Force in x-direction
            [-0.75, 0.75, 0.0, 0.0],           # Force in y-direction
            [0.5, 0.5, 0.5, -1.0],             # Force in z-direction
  
        ])"""
        self.A = np.array([
            [0.4698, 0.4698, -0.9396, 0.0],          # Force in x-direction
            [-0.8138, 0.8138, 0.0, 0.0],           # Force in y-direction
            [0.342, 0.342, 0.342, -1.0],             # Force in z-direction
  
        ])

        # new matrix

        self.FT_desired = np.zeros(3)
        self.spin_force = 0
        self.depth_offset=0.0

        self.support_force_z = 0.0

        self.timer_write = self.create_timer(0.1, self.write_serial, callback_group=self.cb_group_timer) # 10 Hz
        #self.timer_read = self.create_timer(0.05, self.read_serial_data)  # 100 Hz


        self.byte1 = 0
        self.byte2 = 0
        self.byte3 = 0
        self.byte4 = 0
        self.byte5 = 0
        self.byte6 = 0
        self.byte7 = 0
        self.byte8 = 0
        self.byte9 = 0
        self.byte10 = 0
        self.byte11 = 0
        self.byte12 = 0
        self.byte13 = 0
        self.byte14 = 0
        self.byte15 = 0
        self.byte16 = 75
        self.byte17 = 128
        self.byte18 = 0
        self.byte19 = 0




     
    def parameter_update_callback(self, params):
        for param in params:
            if param.name in self.param_map:
                setattr(self, self.param_map[param.name], param.value)
                self.get_logger().info(f"Updated {param.name} to {param.value}")
        return SetParametersResult(successful=True)   

    def support_force_callback(self, msg):
        """
        Callback to handle the support force (z-direction only).
        """
        self.support_force_z = msg.data
        #self.get_logger().info(f"Received support force (z): {self.support_force_z}")

    def manual_bcu_handler(self, msg):
            x = msg.data
            self.byte16 = max(50, min(100, x))  # Ensures it's always in 0–255

    def handle_servo_reset(self, msg):
        # Handle the servo reset command
        if msg.data:
            self.byte18 = 10
        else:
            self.byte18 = 0
    
    def handle_IMU_reset(self, msg):
        # Handle the IMU reset command
        if msg.data:
            self.byte19 = 10
        else:
            self.byte19 = 0
    
    def handle_feeding_command(self, msg):

        # Handle the feeding command
        x = msg.data
        self.byte13 = max(0, min(255, x))  # Ensures it's always in 0–255
        #self.byte13 = x 

    def force_callback(self, msg):
        # Update the force part of the desired force-torque vector
        self.FT_desired = [msg.x, msg.y, msg.z]

        combo=self.FT_desired[2] + self.support_force_z
        self.FT_desired[2]= max(-1.0, min(1.0 , combo)) # Add support force in z-direction
        self.move()

    def spin_callback(self, msg):
        # Update the spin force
        #self.get_logger().info(f"Spin value: {msg.data}")
        spin_mapped = int((msg.data + 1) * 128.5)
        spin_mapped = max(0, min(255, spin_mapped))
        self.byte17 = spin_mapped

    def objective(self,T):
        return np.linalg.norm(self.A @ T - self.FT_desired)**2
    def move(self):
        # Calculate thrust commands using the updated FT_desired
        #A_pseudo_inv = np.linalg.pinv(self.A)
        #self.T = A_pseudo_inv @ self.FT_desired
        #self.T, _ = nnls(self.A, self.FT_desired)
        T_initial = np.zeros(self.A.shape[1])
        bounds = [(0, 1)] * self.A.shape[1]  # (0, None) means T >= 0
        result = minimize(self.objective, T_initial, bounds=bounds)
        self.T=  result.x

        #print("Thrust Commands:", self.T)
        
        #print("Optimized Thrust Values:", self.T)
        self.get_logger().info(f"Optimized Thrust Values: {self.T}")
        # Scale thrust values to 0-255
        T_scaled = (self.T * 255).astype(int)
        # Prepare the bytes to send
        thrust_values = T_scaled.tolist()
        

        log_msg = Float32MultiArray()
        log_msg.data = self.T.tolist()
        self.log_locator_pid_publisher.publish(log_msg)

        # self.byte1 = min(thrust_values[0],self.pump_threshold) #& 0xFF  
        # self.byte2 = min(thrust_values[1],self.pump_threshold) #& 0xFF
        # self.byte3 = min(thrust_values[2],self.pump_threshold) #& 0xFF
        # self.byte4 = min(thrust_values[3],self.pump_threshold) #& 0xFF
        
        self.byte1 = min(thrust_values[0],self.pump_threshold) #& 0xFF  
        self.byte2 = min(thrust_values[1],self.pump_threshold) #& 0xFF
        self.byte3 = min(thrust_values[3],self.pump_threshold) #& 0xFF
        self.byte4 = min(thrust_values[2],self.pump_threshold) #& 0xFF
        #self.byte3=0


    def handle_bcu_message(self, msg):
        """
        Callback function for handling messages from /bcu topic.
        Sends the received value over the serial connection.
        """
        
        x = msg.data
        x = x * 100
        x = int(round(x)) # first use rounding, integer always rounds down!

        x_encoded = max(0, min(x, 65535))
        
        log_msg = Int32()
        log_msg.data = x_encoded
        self.log_bcupbulisher.publish(log_msg)

        #split into two bytes
        self.byte5 = x_encoded #& 0xFF
        #self.byte6 = (x_encoded >> 8) & 0xFF

    def write_serial(self):
        pid_p_int = int(self.pid_p * 1000)
        pid_i_int = int(self.pid_i * 1000)
        pid_d_int = int(self.pid_d * 1000)
        
        """self.byte7 = pid_p_int & 0xFF
        self.byte8 = (pid_p_int >> 8) & 0xFF
        self.byte9 = pid_i_int & 0xFF
        self.byte10 = (pid_i_int >> 8) & 0xFF
        self.byte11 = pid_d_int & 0xFF
        self.byte12 = (pid_d_int >> 8) & 0xFF"""
        pid_p_int = int(self.pid_p * 1000)
        pid_i_int = int(self.pid_i * 1000)
        pid_d_int = int(self.pid_d * 1000)
        
        self.byte6 = pid_p_int 
        self.byte7 = pid_i_int 
        self.byte8 = pid_d_int 
        
        if self.manual_mode:
            self.byte15 = 0
        else:
            self.byte15 = 1
        
        if self.calibration_value:
            self.byte14 = 1
        else:
            self.byte14 = 0
        
        
        if self.serial_connection and self.serial_connection.is_open:
            message_bytes = bytes(17)
            #message_bytes = bytes([self.byte1, self.byte2, self.byte3, self.byte4]) + struct.pack('<HHHH', self.byte5, self.byte6, self.byte7, self.byte8) + bytes([self.byte9])
            #message_bytes = bytes([self.byte1, self.byte2, self.byte3, self.byte4, self.byte5, self.byte6, self.byte7, self.byte8, self.byte9, self.byte10, self.byte11, self.byte12, self.byte13])
            message_bytes = struct.pack('<BBBBHHHHBBBBBBB', self.byte1, self.byte2, self.byte3, self.byte4, self.byte5, self.byte6, self.byte7, self.byte8, self.byte13, self.byte14, self.byte15, self.byte16, self.byte17, self.byte18, self.byte19)

            # Send over serial
            """self.serial_connection.write(message_bytes)
            self.serial_connection.flush()  # Ensure all data is sent
            self.read_serial_data()"""
            try:
                self.serial_connection.write(message_bytes)
                self.serial_connection.flush()
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write failed: {e}")
                return
            self.read_serial_data()

            #log debug     
            #self.get_logger().info(f"Message to send (bytes): {message_bytes}")
            #self.get_logger().info(f"Message sent successfully!")

            
        else:
            self.get_logger().warn("Serial connection is not open.")
        

    def read_serial_data(self):
        if self.serial_connection is None:
            return
        try:
            line = self.serial_connection.readline().decode('utf-8').strip()
            
            """if line.startswith("Sonar Distance:"):
                try:
                    # Extract number (e.g., from "Sonar Distance: 345 mm")
                    mm = int(line.split(":")[1].strip().split()[0])
                    distance_msg = Float32()
                    distance_msg.data = mm / 1000.0  # Convert mm to meters
                    self.publisher_distance.publish(distance_msg)
                    self.get_logger().info(f"Published sonar distance: {distance_msg.data:.3f} m")
                except (IndexError, ValueError):
                    self.get_logger().warn(f"Failed to parse sonar distance from line: {line}")"""
            
            if line.startswith("{"):
                data = json.loads(line)

                distance_msg = Float32()
                #depth_msg.data = data["depth"]
                distance_msg.data = data.get("Sonar_Distance", 0.0) / 1000.0
                self.publisher_distance.publish(distance_msg)

                feeding_msg = Int32()
                #depth_msg.data = data["depth"]
                feeding_result = data.get("feeding", 0)
                if feeding_result == 0: # always 0 while not feeding and 1 while feeding. Becomes 0 immediately after feeding is done
                    feeding_msg.data = 1 # feeding done
                    self.publisher_feeding_status.publish(feeding_msg)
                elif feeding_result == 1:
                    feeding_msg.data = 0 # currently feeding, feeding not done yet
                    self.publisher_feeding_status.publish(feeding_msg)



                # --- Depth Calibration Logic ---
                raw_depth = data.get("depth", 0.0)

                if self.calibration_value == 1:
                    self.depth_offset = raw_depth  # Store the first value as offset
                    self.get_logger().info(f"Depth calibrated. Offset set to {self.depth_offset:.3f} m")
                    self.calibration_value = 0  # Reset the calibration flag

                calibrated_depth = raw_depth - self.depth_offset
                depth_msg = Float32()
                depth_msg.data = float(calibrated_depth)
                self.publisher_pressure.publish(depth_msg)

                # Leackage information
                leakage_msg = Int32()
                if data.get("Leakage", 0) == 1:
                    leakage_msg.data = 1
                    self.sos_publisher.publish(leakage_msg)
                    self.get_logger().warn("Leakage detected!")
                else:
                    leakage_msg.data = 0
                    self.sos_publisher.publish(leakage_msg)

                
                # Extract header information
                header = data["header"]
                seq = header["seq"]
                secs = header["stamp"]["secs"]
                nsecs = header["stamp"]["nsecs"]
                frame_id = header["frame_id"]
                # Extract orientation
                orientation = data["orientation"]
                ox, oy, oz, ow = orientation["x"], orientation["y"], orientation["z"], orientation["w"]
                # Extract linear acceleration
                #linear_acceleration = data["linear_acceleration"]
                #ax, ay, az = linear_acceleration["x"], linear_acceleration["y"], linear_acceleration["z"]
                # Extract angular velocity
                #angular_velocity = data["angular_velocity"]
                #gx, gy, gz = angular_velocity["x"], angular_velocity["y"], angular_velocity["z"]
                # Create and populate the IMU message
                imu_msg = Imu()
                # Populate the header
                imu_msg.header.stamp.sec = secs
                imu_msg.header.stamp.nanosec = nsecs
                imu_msg.header.frame_id = frame_id
                # Populate orientation
                imu_msg.orientation.x = ox
                imu_msg.orientation.y = oy
                imu_msg.orientation.z = oz
                imu_msg.orientation.w = ow
                # Populate linear acceleration
                #imu_msg.linear_acceleration.x = ax
                #imu_msg.linear_acceleration.y = ay
                #imu_msg.linear_acceleration.z = az
                # Populate angular velocity
                #imu_msg.angular_velocity.x = gx
                #imu_msg.angular_velocity.y = gy
                #imu_msg.angular_velocity.z = gz
                # Publish the IMU message
                self.imu_publisher.publish(imu_msg)

                raw_bytes = data.get("raw_bytes", [])
                if len(raw_bytes) == 19:
                    #self.get_logger().info(f"Received raw_bytes: {raw_bytes}")
                    nonsense=0
                else:
                    #self.get_logger().warn(f"raw_bytes has unexpected length: {len(raw_bytes)}")
                    nonsense=1


                # self.get_logger().info(f"Published depth: {depth_msg.data:.2f} m")
                # self.get_logger().info(f"Published sonar distance: {distance_msg.data:.3f} m")


        except json.JSONDecodeError:
            self.get_logger().warn("Received non-JSON or corrupted data from STM")
        except UnicodeDecodeError:
            self.get_logger().warn("Received undecodable byte sequence from STM")





    def destroy_node(self):
        # Close the serial connection when shutting down
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = STMcommunication()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
