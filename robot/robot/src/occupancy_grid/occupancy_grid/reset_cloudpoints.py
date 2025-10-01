import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from pynput import keyboard


class KeyboardReset(Node):
    def __init__(self):
        super().__init__('keyboard_reset')

        # Publisher for the reset topic
        self.reset_publisher = self.create_publisher(Empty, '/reset_cloudpoints', 10)

        # Start listening for key presses
        self.start_key_listener()

    def start_key_listener(self):
        """Start listening for key presses."""
        def on_press(key):
            try:
                if key.char == 'r':  # If 'r' is pressed
                    self.publish_reset()
            except AttributeError:
                pass

        def listen():
            with keyboard.Listener(on_press=on_press) as listener:
                listener.join()

        import threading
        threading.Thread(target=listen, daemon=True).start()

    def publish_reset(self):
        """Publish a reset message."""
        self.reset_publisher.publish(Empty())
        self.get_logger().info("Published reset message!")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardReset()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
