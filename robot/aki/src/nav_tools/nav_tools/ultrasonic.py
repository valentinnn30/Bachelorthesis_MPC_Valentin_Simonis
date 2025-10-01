import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class UltrasonicInputNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_input_node')
        
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 9600)
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
                
        self.publisher_ = self.create_publisher(Float32, 'ultrasonic_distance', 10)
        self.timer = self.create_timer(0.01, self.read_serial_data)  # 100 Hz

    def read_serial_data(self):
        if self.serial_connection is None:
            return
        try:
            line = self.serial_connection.readline().decode('utf-8').strip()
            distance = float(line)
            self.get_logger().info(f"Distance: {distance} cm")
            msg = Float32()
            msg.data = distance
            self.publisher_.publish(msg)
        except ValueError:
            self.get_logger().warn("Received invalid data from serial port")

    def destroy_node(self):
        # Close the serial connection when shutting down
        if self.serial_connection is not None:
            self.serial_connection.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()