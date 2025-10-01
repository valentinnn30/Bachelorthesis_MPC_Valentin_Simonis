import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from yolov8_msgs.msg import DetectionArray  # Adjust this import to match your message type
import time

class CoordinateLogger(Node):
    def __init__(self):
        super().__init__('coordinate_logger')
        
        # Adjust 'coordinates_topic' to your actual topic name and message type
        self.subscription = self.create_subscription(
            DetectionArray,  # Message type
            'yolo/detections',  # Topic name
            self.listener_callback,
            10)
        
        # Open the file in append mode to write coordinates
        self.file = open('coordinates_log.txt', 'a')

    def listener_callback(self, detections_msg):
        # Extract coordinates (assuming an array of coordinates)
        for detection in detections_msg.detections:
            
            if detection.class_name == "table":
                label = detection.class_name
                coordinates = detection.bbox
        
        # Format and write coordinates to the file
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        log_entry = f"{timestamp}: {coordinates}\n{label}\n"
        self.file.write(log_entry)
        
        # Also print to terminal
        self.get_logger().info(f"Logged coordinates: {coordinates}")

    def destroy_node(self):
        # Ensure the file is closed when the node shuts down
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    save_detect = CoordinateLogger()
    
    try:
        rclpy.spin(save_detect)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown node
        save_detect.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
