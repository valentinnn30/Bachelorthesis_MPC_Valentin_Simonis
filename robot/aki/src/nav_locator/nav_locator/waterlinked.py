import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
import requests
from std_msgs.msg import Float64

class AcousticPositionPublisher(Node):
    def __init__(self):
        super().__init__('acoustic_position_publisher')

        #196.168.1.42-waterlinked
        self.demo = False
        self.declare_parameter('ip_address', '192.168.1.42')
        
        self.declare_parameter('request_period', 0.1)  # seconds

        self.ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        self.request_period = self.get_parameter('request_period').get_parameter_value().double_value

        if self.demo:
            self.ip_address  = 'demo.waterlinked.com'

        # Publishers
        self.valid_pub = self.create_publisher(Bool, '/waterlinked/valid', 10)
        self.position_pub = self.create_publisher(Vector3, '/waterlinked/position', 10)
        self.orientation_pub = self.create_publisher(Float64, '/waterlinked/orientation', 10)

        # Timer
        self.timer = self.create_timer(self.request_period, self.timer_callback)

    def timer_callback(self):
     
        # Fetch global position data
        global_url = f'http://{self.ip_address}/api/v1/position/global'
        global_response = requests.get(global_url, timeout=5)
        global_response.raise_for_status()
        global_data = global_response.json()

        # Publish orientation
        orientation_msg = Float64()
        orientation_msg.data = float(global_data.get('orientation', 0.0))
        self.orientation_pub.publish(orientation_msg)

        
        url = f'http://{self.ip_address}/api/v1/position/acoustic/filtered'
        response = requests.get(url, timeout=5)
        response.raise_for_status()
        data = response.json()

        # Publish position_valid
        valid_msg = Bool()
        valid_msg.data = bool(data.get('position_valid', False))
        self.valid_pub.publish(valid_msg)

        # Publish position (x, y, z) as Vector3
        position_msg = Vector3()
        position_msg.x = float(data.get('x', 0.0))
        position_msg.y = float(data.get('y', 0.0))
        position_msg.z = float(data.get('z', 0.0))
        self.position_pub.publish(position_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AcousticPositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
