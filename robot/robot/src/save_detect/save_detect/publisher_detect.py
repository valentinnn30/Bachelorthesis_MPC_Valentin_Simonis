import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from yolov8_msgs.msg import DetectionArray
from yolov8_msgs.msg import BoundingBox2D

class MarkerPublisher(Node):

    def __init__(self):
        super().__init__('marker_publisher')
        self.marker_pub= self.create_publisher(Marker, 'yolo/visualization_marker', 10)
        
        self.subscription = self.create_subscription(DetectionArray, 'yolo/detections', self.listener_callback,10)
        
    def listener_callback(self, detections_msg):
        # Extract coordinates (assuming an array of coordinates)
        x=0
        y=0
        
        for detection in detections_msg.detections:
        
            box_msg: BoundingBox2D = detection.bbox
            if detection.class_name == "table":
                label = detection.class_name
                x = box_msg.center.position.x
                y = box_msg.center.position.y
        z=0
        self.get_logger().info(f"Received coordinates: x={x}, y={y}, z={z}")
        self.publish_marker(x,y,z)
       

    def publish_marker(self, x, y, z):
    	
        marker = Marker()
        # Set the marker's properties
        marker.header.frame_id = "camera_frame" 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "bbox_center"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        # Define start (camera or origin) and end (bbox center) points
        marker.points.append(Point(x=0, y=0, z=0)) # Start point
        marker.points.append(Point(x=x, y=y, z=z)) # End point at bbox center
        # Set the marker scale and color
        marker.scale.x = 0.1 # Arrow shaft diameter
        marker.scale.y = 0.2 # Arrow head diameter
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        # Publish the marker
        self.marker_pub.publish(marker)
        self.get_logger().info("Published marker at position: ({x}, {y}, {z})")

def main(args=None):
    rclpy.init(args=args)

    marker_publisher = MarkerPublisher()

    rclpy.spin(marker_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    marker_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
