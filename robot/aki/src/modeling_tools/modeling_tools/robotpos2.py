import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Transform
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation
import numpy as np
from collections import defaultdict
import re

class RobotPositionPublisher(Node):
    def __init__(self):
        super().__init__('robot_position_publisher')
        # Subscribe to the /tf topic to get the transformations
        self.subscription = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)

        # Store the transforms from origin_tag1 to other tags (tagX)
        self.transforms_from_origin_to_tags = {
            1: np.eye(4),  # T_origin_tag1_to_tag1 (identity)
            2: np.array([[ 0.99784852,  0.00441764, -0.06541267, -0.34190208],
                        [-0.00295148,  0.99974263,  0.02249376,  0.01876456], # Corrected sign based on common calibration errors
                        [ 0.06549521, -0.02225231,  0.99760474,  0.01064881],
                        [ 0., 0., 0., 1.]]),
            3: np.array([[ 8.91971885e-01,  4.50468774e-01,  3.82627817e-02, -5.71577442e-01],
                        [-4.52088981e-01,  8.89012621e-01,  7.26093182e-02,  2.42562756e+00],
                        [-1.30786527e-03, -8.20636523e-02,  9.96626232e-01, -6.89636054e-01],
                        [ 0., 0., 0., 1.]]),
            4: np.array([[ 0.47660384, -0.86663953,  0.14759644, -0.27124038],
                        [ 0.87384851,  0.48537757,  0.02823805,  2.26191422],
                        [-0.09611221,  0.11551857,  0.98864448, -0.68892407],
                        [ 0., 0., 0., 1.]]),
            5: np.array([[-0.06297582, -0.99075712,  0.12014314, -0.28067169],
                        [ 0.98000692, -0.08415514, -0.18028964,  1.00676052],
                        [ 0.1887339,   0.10638722,  0.97624857, -0.6839048 ],
                        [ 0., 0., 0., 1.]])
        }

        # Define a publisher for the robot's position
        self.robot_position_publisher = self.create_publisher(TransformStamped, '/robot_position_to_origin', 10)

        self.origin_tag_id = 1  # Default origin tag ID

        # --- Filtering Parameters ---

        # For Median Filter (Position only)
        self.position_history_median_x = []
        self.position_history_median_y = []
        self.position_history_median_z = []
        self.median_history_size = 15  # Increased: More history for median filter to smooth out spikes

        # For Exponential Moving Average (Low-Pass Filter) - Position (remains same)
        self.smoothed_position_ema_stage1 = None
        self.ema_alpha_stage1 = 0.15 # Decreased: More smoothing, less responsive for stage 1

        self.smoothed_position_ema_stage2 = None
        self.ema_alpha_stage2 = 0.15 # Decreased: More smoothing, less responsive for stage 2 (cascaded)

        # For Exponential Moving Average (Low-Pass Filter) - Orientation (INCREASED SMOOTHING - lower alpha)
        self.smoothed_orientation_ema_stage1 = None
        self.ema_alpha_orientation_stage1 = 0.05 # Significantly decreased for more smoothing
        self.get_logger().info(f"Orientation EMA Alpha Stage 1 set to: {self.ema_alpha_orientation_stage1}")

        self.smoothed_orientation_ema_stage2 = None
        self.ema_alpha_orientation_stage2 = 0.05 # Significantly decreased for more smoothing
        self.get_logger().info(f"Orientation EMA Alpha Stage 2 set to: {self.ema_alpha_orientation_stage2}")

        # For Statistical Outlier Detection (Z-score based) - Position only (VERY ROBUST)
        self.outlier_detection_history = []
        self.outlier_history_size = 30 # Significantly increased for more robust mean/std calculation
        self.z_score_threshold = 2.0 # Decreased for more aggressive outlier detection
        self.get_logger().info(f"Position Outlier Z-score Threshold set to: {self.z_score_threshold}")
        self.get_logger().info(f"Position Outlier History Size set to: {self.outlier_history_size}")

        # Stores the last successfully processed position and orientation (after all filtering)
        self.last_robust_position = None
        self.last_robust_orientation = None # Quaternion (x, y, z, w)

        self.get_logger().info("RobotPositionPublisher node initialized with VERY ROBUST filtering.")

    def tf_callback(self, msg):
        candidate_transforms_to_robot = []
        latest_timestamp = None

        for t in msg.transforms:
            if t.header.frame_id == "camera" and t.child_frame_id.startswith("tag36h11:"):
                try:
                    tag_id = int(t.child_frame_id.split(':')[1])
                    if tag_id in self.transforms_from_origin_to_tags:
                        T_camera_to_tag_matrix = self.transform_to_matrix(t.transform)
                        T_origin_tag1_to_tag_X_matrix = self.transforms_from_origin_to_tags.get(tag_id)
                        
                        T_tag_X_to_camera_matrix = np.linalg.inv(T_camera_to_tag_matrix)
                        T_origin_tag1_to_robot_matrix = T_origin_tag1_to_tag_X_matrix @ T_tag_X_to_camera_matrix
                        
                        candidate_transforms_to_robot.append(T_origin_tag1_to_robot_matrix)
                        
                        current_ts_ns = t.header.stamp.sec * 10**9 + t.header.stamp.nanosec
                        if latest_timestamp is None or current_ts_ns > (latest_timestamp.sec * 10**9 + latest_timestamp.nanosec):
                            latest_timestamp = t.header.stamp

                except ValueError:
                    self.get_logger().warn(f"Invalid tag ID format: {t.child_frame_id}")
                    continue

        if not candidate_transforms_to_robot:
            if self.last_robust_position is not None and self.last_robust_orientation is not None:
                self.get_logger().warn("No known ArUco tag is currently in view. Publishing last known robust position.")
                self.publish_robot_transform(self.last_robust_position, self.last_robust_orientation, self.get_clock().now().to_msg())
            else:
                self.get_logger().warn("No known ArUco tag is currently in view and no previous robust position available. Cannot publish.")
            return

        # Average the candidate transforms (robust initial estimate from multiple tags)
        averaged_position, averaged_quaternion = self.average_transforms(candidate_transforms_to_robot)

        # --- Apply Filtering Pipeline for Robustness ---

        # Initialize filtered variables with the averaged values (or last robust if outlier is detected later)
        position_for_filtering = averaged_position
        orientation_for_filtering = averaged_quaternion

        # 1. Statistical Outlier Detection (for Position) - VERY ROBUST
        # This uses the history of 'averaged' positions to detect outliers before further smoothing.
        self.outlier_detection_history.append(averaged_position)
        if len(self.outlier_detection_history) > self.outlier_history_size:
            self.outlier_detection_history.pop(0)

        is_outlier = False
        if len(self.outlier_detection_history) >= self.outlier_history_size:
            is_outlier = self.detect_and_reject_outliers(averaged_position)
            if is_outlier:
                if self.last_robust_position is not None and self.last_robust_orientation is not None:
                    self.get_logger().warn(f"HIGH CONFIDENCE OUTLIER DETECTED for position: {averaged_position}. Using last robust position/orientation ({self.last_robust_position}).")
                    position_for_filtering = self.last_robust_position
                    orientation_for_filtering = self.last_robust_orientation # Use last orientation if position is outlier
                else:
                    # This case means an outlier was detected, but no previous good data to fall back on.
                    # This is rare if system has run for a bit, but for first few frames, it's possible.
                    self.get_logger().warn(f"Outlier detected for position: {averaged_position}. No previous robust position to fall back on. Proceeding with current averaged (may cause a jump).")
                    # position_for_filtering and orientation_for_filtering remain as averaged_position/quaternion
        else:
            self.get_logger().info(f"Building robust outlier history for position ({len(self.outlier_detection_history)}/{self.outlier_history_size}).")

        # 2. Apply Median Filter (Position only - highly resistant to spikes)
        # This will operate on either the averaged_position or the last_robust_position if an outlier was detected.
        position_after_median = self.apply_median_smoothing(position_for_filtering)

        # 3. Apply First Stage Exponential Moving Average (Position and Orientation)
        position_after_ema1 = self.apply_ema_smoothing(position_after_median, self.ema_alpha_stage1, self.smoothed_position_ema_stage1, "position", 1)
        orientation_after_ema1 = self.apply_ema_smoothing(orientation_for_filtering, self.ema_alpha_orientation_stage1, self.smoothed_orientation_ema_stage1, "orientation", 1)

        # 4. Apply Second Stage Exponential Moving Average (Position and Orientation)
        position_after_ema2 = self.apply_ema_smoothing(position_after_ema1, self.ema_alpha_stage2, self.smoothed_position_ema_stage2, "position", 2)
        orientation_after_ema2 = self.apply_ema_smoothing(orientation_after_ema1, self.ema_alpha_orientation_stage2, self.smoothed_orientation_ema_stage2, "orientation", 2)

        # Update the last robust position and orientation
        # These now hold the most filtered, robust data for fallback in the next cycle
        self.last_robust_position = position_after_ema2
        self.last_robust_orientation = orientation_after_ema2

        # Publish the robot's position and orientation relative to the origin tag
        self.publish_robot_transform(position_after_ema2, orientation_after_ema2, latest_timestamp)


    def average_transforms(self, transforms_4x4: list) -> tuple[np.ndarray, np.ndarray]:
        """
        Averages a list of 4x4 homogeneous transformation matrices.
        This provides a robust initial estimate by combining multiple tag observations.
        """
        if not transforms_4x4:
            return np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0])

        positions = []
        quaternions = []

        for T_matrix in transforms_4x4:
            positions.append(T_matrix[:3, 3])
            rotation_matrix = T_matrix[:3, :3]
            quaternions.append(Rotation.from_matrix(rotation_matrix).as_quat()) # x, y, z, w

        avg_position = np.mean(positions, axis=0)

        # Quaternion averaging: Simple average then normalize.
        avg_quaternion_unnormalized = np.mean(quaternions, axis=0)
        # Handle case where all quaternions are (0,0,0,0) or close to it,
        # which can happen if input data is completely corrupted.
        norm = np.linalg.norm(avg_quaternion_unnormalized)
        if norm > 1e-9: # Avoid division by zero
            avg_quaternion = avg_quaternion_unnormalized / norm
        else:
            avg_quaternion = np.array([0.0, 0.0, 0.0, 1.0]) # Identity quaternion

        return avg_position, avg_quaternion

    def transform_to_matrix(self, transform_msg: Transform) -> np.ndarray:
        """Converts a geometry_msgs/Transform message to a 4x4 homogeneous transformation matrix."""
        translation = np.array([transform_msg.translation.x,
                                transform_msg.translation.y,
                                transform_msg.translation.z])
        rotation = Rotation.from_quat([transform_msg.rotation.x,
                                       transform_msg.rotation.y,
                                       transform_msg.rotation.z,
                                       transform_msg.rotation.w])
        rotation_matrix = rotation.as_matrix()
        matrix = np.eye(4)
        matrix[:3, :3] = rotation_matrix
        matrix[:3, 3] = translation
        return matrix

    # --- Filtering Methods ---

    def apply_median_smoothing(self, new_position):
        """
        Smooth the robot's position using a median filter.
        This is highly resistant to outlier spikes.
        """
        self.position_history_median_x.append(new_position[0])
        self.position_history_median_y.append(new_position[1])
        self.position_history_median_z.append(new_position[2])

        if len(self.position_history_median_x) > self.median_history_size:
            self.position_history_median_x.pop(0)
            self.position_history_median_y.pop(0)
            self.position_history_median_z.pop(0)

        # Return the new position if not enough history for a median calculation
        if len(self.position_history_median_x) < self.median_history_size:
            return new_position

        smoothed_x = np.median(self.position_history_median_x)
        smoothed_y = np.median(self.position_history_median_y)
        smoothed_z = np.median(self.position_history_median_z)

        return np.array([smoothed_x, smoothed_y, smoothed_z])

    def apply_ema_smoothing(self, new_data, alpha, smoothed_data_ref, data_type, stage):
        """
        Smooths data using an Exponential Moving Average (Low-Pass Filter).
        Handles both position (3D vector) and orientation (4D quaternion).
        This helps in further smoothing after outlier rejection/median filtering.
        """
        current_smoothed = None
        
        if data_type == "position":
            if smoothed_data_ref is None:
                current_smoothed = new_data
            else:
                current_smoothed = (alpha * new_data) + ((1 - alpha) * smoothed_data_ref)
            
            if stage == 1: self.smoothed_position_ema_stage1 = current_smoothed
            elif stage == 2: self.smoothed_position_ema_stage2 = current_smoothed
            
        elif data_type == "orientation":
            if smoothed_data_ref is None:
                current_smoothed = new_data
            else:
                current_smoothed = (alpha * new_data) + ((1 - alpha) * smoothed_data_ref)
                # Ensure quaternion remains a unit quaternion
                norm = np.linalg.norm(current_smoothed)
                if norm > 1e-9: # Avoid division by zero
                    current_smoothed = current_smoothed / norm
                else: # Fallback to identity if smoothed result is near zero
                    current_smoothed = np.array([0.0, 0.0, 0.0, 1.0])
            
            if stage == 1: self.smoothed_orientation_ema_stage1 = current_smoothed
            elif stage == 2: self.smoothed_orientation_ema_stage2 = current_smoothed
            
        else:
            self.get_logger().error(f"Unknown data type for EMA smoothing: {data_type}. Returning raw data.")
            return new_data

        return current_smoothed # Return the just calculated smoothed value


    def detect_and_reject_outliers(self, new_position):
        """
        Detects outliers using a Z-score method.
        Returns True if the new_position is an outlier, False otherwise.
        This provides robust outlier rejection for position data.
        """
        # Ensure there's enough data to calculate standard deviation for robustness
        if len(self.outlier_detection_history) < 2:
            return False

        history_array = np.array(self.outlier_detection_history)
        
        # Calculate mean and standard deviation of the historical data
        mean_position = np.mean(history_array, axis=0)
        std_dev_position = np.std(history_array, axis=0)

        # Handle cases where standard deviation is zero (e.g., robot is stationary for a long time)
        # By setting a very small value, any deviation will result in a large Z-score, correctly identifying a "jump"
        std_dev_position[std_dev_position == 0] = 1e-9

        # Calculate Z-scores for each dimension (x, y, z)
        z_scores = np.abs((new_position - mean_position) / std_dev_position)

        # If any component's Z-score exceeds the threshold, it's considered an outlier
        if np.any(z_scores > self.z_score_threshold):
            return True
        return False

    def publish_robot_transform(self, robot_position, robot_rotation_quat_xyzw, timestamp):
        """Publish the robot's position relative to the origin tag"""
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = timestamp
        transform_stamped.header.frame_id = f"origin_tag36h11:{self.origin_tag_id}"
        transform_stamped.child_frame_id = "robot_frame"

        transform_stamped.transform.translation.x = robot_position[0]
        transform_stamped.transform.translation.y = robot_position[1]
        transform_stamped.transform.translation.z = robot_position[2]

        transform_stamped.transform.rotation.x = robot_rotation_quat_xyzw[0]
        transform_stamped.transform.rotation.y = robot_rotation_quat_xyzw[1]
        transform_stamped.transform.rotation.z = robot_rotation_quat_xyzw[2]
        transform_stamped.transform.rotation.w = robot_rotation_quat_xyzw[3]

        self.robot_position_publisher.publish(transform_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = RobotPositionPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()