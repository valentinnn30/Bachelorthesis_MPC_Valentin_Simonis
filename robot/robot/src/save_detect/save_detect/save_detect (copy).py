import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
#from vision_msgs.msg import Detection2DArray # Assuming detection messages use this
#from yolo import Detection
#/yolov8_ros/yolov8_msgs.msg import Detection

from yolov8_msgs.msg import DetectionArray
format
from cv_bridge import CvBridge
import cv2
import os
class SpecificObjectSaver(Node):
    def __init__(self, target_class="coral"):
        super().__init__('save_detect')
# Subscribe to the image and bounding boxes topics
        #self.image_subscription = self.create_subscription(Image,'camera/rgb/image_raw',self.image_callback,10)
        self.bbox_subscription = self.create_subscription(
            DetectionArray,
            'yolo/detections',
            self.bbox_callback,
            10
        )
        self.bridge = CvBridge()
        #self.current_image = None
        self.target_class = target_class # The class label you want to save (e.g., "person")
        self.image_count = 0
    """def image_callback(self, msg):
        # Store the latest image frame
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")"""
    def bbox_callback(self, detections_msg):
        #if self.current_image is None:
            #return # Ensure we have an image to work with"""
    # Process each detection
        for detection in detections_msg.detections:
# Replace with the field name for your object label (YOLO format varies)
            detected_class = detection.class_id
            if detected_class == self.target_class:
# Extract bounding box (YOLO typically provides center, width, and height)
                bbox = detection.bbox
                x_min = int(bbox.center.x - bbox.size_x / 2)
                y_min = int(bbox.center.y - bbox.size_y / 2)
                x_max = int(bbox.center.x + bbox.size_x / 2)
                y_max = int(bbox.center.y + bbox.size_y / 2)
# Crop or highlight the detected object
                #target_object = self.current_image[y_min:y_max, x_min:x_max]
                target_object = np.array[y_min,y_max, x_min,x_max]
# Define save path
                #save_path = os.path.join("/home/reefranger/Desktop/predictions", f"{self.target_class}_{self.image_count}.png")
                save_path = os.path.join("/home/reefranger/Desktop/predictions", f"{self.target_class}_{self.image_count}.txt")
                cv2.imwrite(save_path, target_object)
                self.get_logger().info(f"Saved {self.target_class} to {save_path}")
                self.image_count += 1
                break # Exit loop if only saving one instance per frame

def main(args=None):
    rclpy.init(args=args)
    target_class = "coral" # Change to your desired class label
    save_detect = SpecificObjectSaver(target_class=target_class)
    rclpy.spin(save_detect)
    save_detect.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
