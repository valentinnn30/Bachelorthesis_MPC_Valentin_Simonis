import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoSerialPublisher(Node):
    def __init__(self):
        super().__init__('arduino_serial_publisher')
        # Initialize publisher
        self.publisher_ = self.create_publisher(String, '/microcontroller', 10)
        
        # Serial connection settings
        self.serial_port = '/dev/ttyACM0'  # Update this to match your Arduino's port
        self.baud_rate = 9600
        
        # Establish serial connection
        try:
            self.serial_connection = serial.Serial(self.serial_port, self.baud_rate)
            self.get_logger().info(f'Successfully connected to {self.serial_port} at {self.baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {self.serial_port}: {e}')
            raise
        
        # Create timer to read and publish data
        self.timer = self.create_timer(0.1, self.read_and_publish_data)

    def read_and_publish_data(self):
        if self.serial_connection.in_waiting > 0:
            try:
                # Read line from serial
                serial_data = self.serial_connection.readline().decode('utf-8').strip()
                self.get_logger().info(f'Received from Arduino: {serial_data}')
                
                # Publish to ROS 2 topic
                msg = String()
                msg.data = serial_data
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Error reading serial data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Close serial connection on shutdown
        if node.serial_connection.is_open:
            node.serial_connection.close()
            node.get_logger().info('Serial connection closed.')
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
