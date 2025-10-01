import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # Import the Int32 message type
from std_msgs.msg import Float32 
import serial

class SerialCommandNode(Node):
    def __init__(self):
        super().__init__('pressure_send')
        
        # Declare and retrieve parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')  # Update with your port
        self.declare_parameter('baud_rate', 115200)
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        # Initialize serial connection
        try:
            self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f"Connected to {serial_port} at {baud_rate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            self.serial_connection = None

        # Subscriber to the /bcu topic
        self.subscription = self.create_subscription(
            Float32,  # Message type
            '/bcu',  # Topic name
            self.handle_bcu_message,  # Callback function
            10  # QoS (queue size)
        )
        self.get_logger().info("Subscribed to /bcu topic.")

    def handle_bcu_message(self, msg):
        """
        Callback function for handling messages from /bcu topic.
        Sends the received value over the serial connection.
        """
        if self.serial_connection and self.serial_connection.is_open:
            # Prepare the command
            x = msg.data
            x = x * 100
            x = int(x)
            x_encoded = x + 1500 
            x_encoded = max(0, min(x_encoded, 65535))
            
            #split into two bytes
            byte1 = x_encoded & 0xFF
            byte2 = (x_encoded >> 8) & 0xFF
            #create byte array
            message_bytes = bytes([byte1, byte2])
            # Send over serial
            self.serial_connection.write(message_bytes)
            #log debug
            self.get_logger().info(f"Sent encoded: {x_encoded}-> Bytes: [{byte1:#04x}, {byte2:#04x}]")
            """self.serial_connection.write(command.encode('utf-8'))
            self.get_logger().info(f"Sent: {command.strip()}")"""
        else:
            self.get_logger().warn("Serial connection is not open.")

    def destroy_node(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()