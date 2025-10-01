import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from yolov8_msgs.msg import DetectionArray
from yolov8_msgs.msg import BoundingBox2D

class MarkerPublisher(Node):

    def __init__(self):
        super().__init__('marker_publisher')

        self.marker_pub = self.create_publisher(Marker, 'visualization_marker_array/pfeil', 10)
        self.pos_pub = self.create_publisher(Vector3, 'tracked/table', 10)

        self.subscription = self.create_subscription(DetectionArray, 'yolo/detections', self.listener_callback, 10)


        self.declare_parameter('camera_height', 600) #y
        self.declare_parameter('camera_width', 800) #x
        self.camera_height = self.get_parameter('camera_height').get_parameter_value().integer_value
        self.camera_width = self.get_parameter('camera_width').get_parameter_value().integer_value
        
        # Store previous coordinates
        self.last_x = None
        self.last_y = None
        self.last_z = 0.0  # Default z value

    def listener_callback(self, detections_msg):
  
        self.score = 0.0

        x = self.last_x if self.last_x is not None else 0.0
        y = self.last_y if self.last_y is not None else 0.0

        for detection in detections_msg.detections:
            if detection.class_id ==1:
                box_msg: BoundingBox2D = detection.bbox
                if detection.class_name == "table":
                    x = box_msg.center.position.x
                    y = box_msg.center.position.y
                    self.last_x = x  
                    self.last_y = y
                    
                    diff_x =   abs(self.camera_width/2-x)
                    diff_y =   abs(self.camera_height/2-y)

                    self.get_logger().info(f"New table at: x={x}, y={y}, z={self.last_z}")
            else:
                """self.get_logger().info(f"Coral detected")"""


        self.publish_marker(x, y, self.last_z)

    def publish_marker(self, x, y, z):
        marker = Marker()
        # Set the marker's properties
        marker.header.frame_id = "world" 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "bbox_center"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        # Define start (camera or origin) and end (bbox center) points
        marker.points.append(Point(x=0.0, y=0.0, z=0.0))  # Start point
        marker.points.append(Point(x=x, y=y, z=z))  # End point at bbox center
        # Set the marker scale and color
        marker.scale.x = 0.1  # Arrow shaft diameter
        marker.scale.y = 0.2  # Arrow head diameter
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        # Publish the marker
        self.marker_pub.publish(marker)

        vec = Vector3()
        vec.x = x
        vec.y = y
        vec.z = z
        self.pos_pub.publish(vec)
        self.get_logger().info(f"Published marker at position: ({x}, {y}, {z})")

def main(args=None):
    rclpy.init(args=args)

    marker_publisher = MarkerPublisher()

    rclpy.spin(marker_publisher)

    # Destroy the node explicitly
    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
