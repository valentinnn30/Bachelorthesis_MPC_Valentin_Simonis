import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import json
import numpy as np
from rcl_interfaces.msg import SetParametersResult

class SerialIMUNode(Node):
    def __init__(self):
        super().__init__("serial_imu_node")
        
        # Declare parameters
        self.declare_parameter("serial_port", "/dev/ttyACM0") # Default serial port
        #self.declare_parameter("serial_port", "/dev/ttyACM1") # seperate serial port for the IMU
        self.declare_parameter("baud_rate", 2000000)
        # Get parameters
        serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        
        # Mapping for dynamic update
        self.param_map = {
            'serial_port': 'serial_port',
            'baud_rate': 'baud_rate'
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
        # Publisher for IMU data
        self.imu_publisher = self.create_publisher(Imu, "/imu", 10)
        # Timer to read serial data
        self.timer = self.create_timer(0.01, self.read_serial_data)  # 100 Hz

    def parameter_update_callback(self, params):
        for param in params:
            if param.name in self.param_map:
                setattr(self, self.param_map[param.name], param.value)
                self.get_logger().info(f"Updated {param.name} to {param.value}")
        return SetParametersResult(successful=True)

    def read_serial_data(self):
        if self.serial_connection is None:
            return
        try:
            # Read a line of data from the serial port
            line = self.serial_connection.readline().decode('utf-8').strip()

            # raw_bytes = self.serial_connection.readline()
            # try:
            #     line = raw_bytes.decode('utf-8').strip()
            # except UnicodeDecodeError:
            #     #self.get_logger().warn(f"Received non-UTF-8 data: {raw_bytes}")
            #     return

            #self.get_logger().info(f"Raw serial data: {line}")  # Log raw data
            
            # Parse the JSON data
            imu_data = json.loads(line)
            # Extract header information
            header = imu_data["header"]
            seq = header["seq"]
            secs = header["stamp"]["secs"]
            nsecs = header["stamp"]["nsecs"]
            frame_id = header["frame_id"]
            # Extract orientation
            orientation = imu_data["orientation"]
            ox, oy, oz, ow = orientation["x"], orientation["y"], orientation["z"], orientation["w"]
            # Extract linear acceleration
            linear_acceleration = imu_data["linear_acceleration"]
            ax, ay, az = linear_acceleration["x"], linear_acceleration["y"], linear_acceleration["z"]
            # Extract angular velocity
            angular_velocity = imu_data["angular_velocity"]
            gx, gy, gz = angular_velocity["x"], angular_velocity["y"], angular_velocity["z"]
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
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            # Populate angular velocity
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            # Publish the IMU message
            self.imu_publisher.publish(imu_msg)

            # Convert quaternion to roll, pitch, yaw for stabilization
            roll, pitch, yaw = self.quaternion_to_euler(ox, oy, oz, ow)
            # Convert radians to degrees
            roll_deg = np.degrees(roll)
            pitch_deg = np.degrees(pitch)
            yaw_deg = np.degrees(yaw)
            #self.get_logger().info(f"Roll: {roll_deg}°, Pitch: {pitch_deg}°, Yaw: {yaw_deg}°")
            #self.get_logger().info(f"Yaw: {yaw_deg}°")

        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Failed to parse JSON from serial: {e}")
        except KeyError as e:
            self.get_logger().warn(f"Missing expected key in IMU data: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

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
        #self.get_logger().info(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
        
        return roll, pitch, yaw

    def destroy_node(self):
        # Close the serial connection when shutting down
        if self.serial_connection is not None:
            self.serial_connection.close()
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    node = SerialIMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()









