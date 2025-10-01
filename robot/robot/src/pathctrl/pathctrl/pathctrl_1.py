import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math

class TransformListenerNode(Node):

    def __init__(self):
        super().__init__('transform_listener_node')

        # Declare parameters for PID gains with default values
        self.declare_parameter('k_p', 6.0)
        self.declare_parameter('k_i', 0.15)
        self.declare_parameter('k_d', 0.50)

        # Retrieve PID parameters from the parameter server
        self.k_p = self.get_parameter('k_p').get_parameter_value().double_value
        self.k_i = self.get_parameter('k_i').get_parameter_value().double_value
        self.k_d = self.get_parameter('k_d').get_parameter_value().double_value

        # Create a buffer and transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher_ = self.create_publisher(Float64, '/vertical', 10)
        self.logger_ = self.create_publisher(Float64, '/pid', 10)
        self.publisher_path = self.create_publisher(Path, 'path', 10)

        self.prev_error = 0.0  # Previous error for derivative term
        self.integral = 0.0    # Integral accumulator

        # Timer to repeatedly query the transform
        self.timer = self.create_timer(1.0, self.get_transform)

    def get_transform(self):
        try:
            # Lookup the transform between the two links
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'tethys/camera_link/camera',  # Replace with the first link
                'base',  # Replace with the second link
                rclpy.time.Time()  # Get the latest available transform
            )

            # Extract the translation vector (x, y, z)
            translation = transform.transform.translation
            error = translation.x

            # PID controller calculations
            self.integral += error  # Integral term
            derivative = error - self.prev_error  # Derivative term

            control_signal = (self.k_p * error) + (self.k_i * self.integral) + (self.k_d * derivative)
            control_signal = max(-0.26, min(0.26, control_signal))  # Clamp control signal to a range

            # Publish the control signal
            msg = Float64()
            msg.data = error
            self.publisher_.publish(msg)

            # Log the PID control output
            log_data = Float64()
            log_data.data = error
            self.logger_.publish(log_data)
            self.get_logger().info(f"x: {translation.x} {translation.y} {translation.z} ")

            # Update previous error
            self.prev_error = error
            """
            path = Path()
            path.header.frame_id = "base_link"
            path.header.stamp = self.get_clock().now().to_msg()
            x = translation.x
            y = translation.y
            z = translation.z

            # Create a series of points for the path
            num_points = 100  # Number of points in the path
            for i in range(num_points + 1):
                t = i / num_points
                pose = PoseStamped()
                pose.header.frame_id = "base_link"
                pose.header.stamp = self.get_clock().now().to_msg()
                
                # Interpolation using a parametric equation
                pose.pose.position.x = x * t  # Linear interpolation for x
                pose.pose.position.y = y * math.sin(math.pi * t)  # Sinusoidal in y
                pose.pose.position.z = z * (t**2)  # Quadratic in z
                
                # Set orientation (can be identity quaternion if not needed)
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0
                
                # Add pose to the path
                path.poses.append(pose)

            # Publish the path
            self.publisher_path.publish(path)
                """
        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = TransformListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()