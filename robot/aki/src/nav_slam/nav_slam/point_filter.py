import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs_py.point_cloud2 import read_points
import numpy as np
import time
import cv2
from sklearn.neighbors import NearestNeighbors
import numpy as np
from sklearn.cluster import DBSCAN


class PointCloudRectangleNode(Node):
    def __init__(self):
        super().__init__('pointcloud_rectangle_node')

        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/map_pointcloud2',
            self.pointcloud_callback,
            10
        )

        self.rectangle_marker_publisher = self.create_publisher(
            Marker,
            '/rectangle_marker',
            10
        )

        self.start_time = time.time()
        self.origin = [0.0, 0.0]  # You may want to replace this with actual origin
        self.z_filter = 0.0  # Adjust as necessary
        self.points = []
        self.map_generated = False

    def pointcloud_callback(self, msg):
        if self.map_generated:
            return  # Avoid processing if the map has already been generated
        if (time.time() - self.start_time < 5.0):
            threshold = 5.0 # 1.0, 5.0 für kein filter basically
            #z_threshold = 1.4 # 1.3/1.4 für nahen Tisch (kurz nach SLAM-start), 1.4/1.5 für mittleren Tisch. 
            # 1.4 als guter tradeoff, manchmal zu groß für den kleinen aber zu klein für den großen Tisch -> somit für den mittleren optimal
            z_threshold = 10.0 # for infiity
            try:
                for point in read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                    if (-threshold + self.origin[0]) <= point[0] <= (threshold + self.origin[0]) and \
                       (-threshold + self.origin[1]) <= point[1] <= (threshold + self.origin[1]) and \
                       point[2] <= (self.z_filter + z_threshold):
                        self.points.append([point[0], point[1], point[2]])
            except Exception as e:
                self.get_logger().error(f"Error processing PointCloud2: {e}")
        else:
            self.get_logger().info("Finished collecting points, generating map...")
            points_array = np.array(self.points)
            
            # old z filtering method -> did not work since the points in the middle where to close to the camera
            """x_mean = np.mean(points_array[:, 0])
            y_mean = np.mean(points_array[:, 1])
            distances = np.sqrt((points_array[:, 0] - x_mean)**2 + (points_array[:, 1] - y_mean)**2)
            nearest_index = np.argmin(distances)
            z_center = points_array[nearest_index, 2]
            x_center = points_array[nearest_index, 0]
            y_center = points_array[nearest_index, 1]
            self.get_logger().info(f"Closest point z_center: {z_center} at position ({x_center}, {y_center})")

            filtered_points = []
            z_threshold = 0.05  # Adjust as necessary
            for x, y, z in points_array:
                if abs(z - z_center) <= z_threshold:
                    filtered_points.append([x, y, z])"""
            
            # Densest region filter
            k = 10
            xy_array = points_array[:, :2]
            nn = NearestNeighbors(n_neighbors=k)
            nn.fit(xy_array)

            distances, _ = nn.kneighbors(xy_array)
            avg_distances = np.mean(distances, axis=1)
            i_max = np.argmin(avg_distances)  # small avg distance = dense region
            center = points_array[i_max]

            # z-cooridnate bei dem rosbag mit 1.4 pressure bei 0.74 für kleiner tisch/ 1.07 für mittel und 1.3 für gross -> der alte filter ist alte nicht suitable
            self.get_logger().info(f"Densest center point: ({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f})")
            z_center = center[2]
            z_threshold = 0.05 # 0.05 oder 0.10, 0.05 bissi präziser aber kleiner
            # rotation sehr gut, bisschen verschoben immer. bei kleinem perfekt
            core_points = points_array[np.abs(points_array[:, 2] - z_center) < z_threshold]

            # Apply DBSCAN clustering to find the largest cluster
            """xy_array = points_array[:, :2]
            db = DBSCAN(eps=0.3, min_samples=10).fit(xy_array)
            labels = db.labels_
            valid = labels != -1
            if np.any(valid):
                largest = np.argmax(np.bincount(labels[valid]))
                core_points = points_array[labels == largest]
            else:
                core_points = np.empty((0, 3))

            if core_points.shape[0] == 0:
                self.get_logger().warn("No valid cluster found.")
                return
            """
            # crashes after using DBSCAN, not sure why
            

            #corners = self.find_rotated_rectangle(filtered_points)
            corners = self.find_rotated_rectangle(core_points)
            sorted_corners = self.sort_corners_clockwise(corners)
            outer_marker = self.create_rectangle_marker(sorted_corners, ns="rectangle", marker_id=0, color=(0.0, 1.0, 0.0, 1.0))
            self.rectangle_marker_publisher.publish(outer_marker)

            self.map_generated = True

    def find_rotated_rectangle(self, points_array):
        points_array = np.array(points_array)
        xy_points = points_array[:, :2].astype(np.float32)
        rect = cv2.minAreaRect(xy_points)
        center, (width, height), angle = rect
        square_size = min(width, height)
        square_rect = (center, (square_size, square_size), angle)
        square_box = cv2.boxPoints(square_rect)
        return np.array(square_box)

    def sort_corners_clockwise(self, corners):
        center = np.mean(corners, axis=0)
        angles = np.arctan2(corners[:,1] - center[1], corners[:,0] - center[0])
        sort_order = np.argsort(-angles)
        return corners[sort_order]

    def create_rectangle_marker(self, corners, frame_id="map", ns="rectangle", marker_id=0, color=(0.0, 1.0, 0.0, 1.0)):
        self.get_logger().info(f"Creating rectangle marker in namespace '{ns}' with ID {marker_id}.")
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        for i in range(4):
            pt = Point()
            pt.x = float(corners[i][0])
            pt.y = float(corners[i][1])
            pt.z = 0.0
            marker.points.append(pt)
            self.get_logger().info(f"Added corner {i}: x={pt.x}, y={pt.y}, z={pt.z}")

        pt = Point()
        pt.x = float(corners[0][0])
        pt.y = float(corners[0][1])
        pt.z = 0.0
        marker.points.append(pt)
        self.get_logger().info(f"Closed loop with first corner: x={pt.x}, y={pt.y}, z={pt.z}")

        self.get_logger().info("Rectangle marker creation complete.")
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudRectangleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
