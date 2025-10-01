import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import json
class SerialIMUNode(Node):
    def __init__(self):
        super().__init__("serial_imu_node")
        # Declare parameters
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 2000000)
        # Get parameters
        serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
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
    def read_serial_data(self):
        if self.serial_connection is None:
            return
        try:
            # Read a line of data from the serial port
            line = self.serial_connection.readline().decode('utf-8').strip()
            self.get_logger().info(f"Raw serial data: {line}")  # Log raw data
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
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Failed to parse JSON from serial: {e}")
        except KeyError as e:
            self.get_logger().warn(f"Missing expected key in IMU data: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
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









