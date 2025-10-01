import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from gototable_interface.action import GoTable
import time


#yolo


class GoTableServer(Node):

    def __init__(self):
        super().__init__('go_table_server')

        # Subscription to track the table's position
        self.subscription = self.create_subscription(Vector3, '/tracked/table', self.detection_callback, 1)

        # Publishers for force and PID logs
        self.power = self.create_publisher(Vector3, '/power', 10)
        self.logx = self.create_publisher(Float64, '/logpidtablex', 10)
        self.logy = self.create_publisher(Float64, '/logpidtabley', 10)


        # Action server for GoTable action
        self._action_server = ActionServer(
            self,
            GoTable,
            'gototable',
            self.execute_callback)

        # Declare PID parameters
        self.declare_parameter('kp_x', 0.001)
        self.declare_parameter('ki_x', 0.0)
        self.declare_parameter('kd_x', 0.0)
        self.declare_parameter('kp_y', 0.001)
        self.declare_parameter('ki_y', 0.0)
        self.declare_parameter('kd_y', 0.0)
        self.declare_parameter('threshold_x', 60.0)
        self.declare_parameter('threshold_y', 60.0)

        # Get PID parameters
        self.threshold_x = self.get_parameter('threshold_x').get_parameter_value().double_value
        self.threshold_y = self.get_parameter('threshold_y').get_parameter_value().double_value

        self.kp_x = self.get_parameter('kp_x').get_parameter_value().double_value
        self.ki_x = self.get_parameter('ki_x').get_parameter_value().double_value
        self.kd_x = self.get_parameter('kd_x').get_parameter_value().double_value

        self.kp_y = self.get_parameter('kp_y').get_parameter_value().double_value
        self.ki_y = self.get_parameter('ki_y').get_parameter_value().double_value
        self.kd_y = self.get_parameter('kd_y').get_parameter_value().double_value

        self.get_logger().debug(f'PID Gains for X: kp={self.kp_x}, ki={self.ki_x}, kd={self.kd_x}', skip_first=True, throttle_duration_sec=1.5)
        self.get_logger().debug(f'PID Gains for Y: kp={self.kp_y}, ki={self.ki_y}, kd={self.kd_y}', skip_first=True, throttle_duration_sec=1.5)

        # Initialize PID state
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_x_error = 0.0
        self.prev_y_error = 0.0

        # Variables to store the current position
        self.current_x = 0.0
        self.current_y = 0.0

    def detection_callback(self, detection: Vector3):
        """Update the current position based on tracked table data."""
        self.current_x = detection.x
        self.current_y = detection.y
        self.get_logger().info(f'Updated current position: X={self.current_x}, Y={self.current_y}')

    def execute_callback(self, goal_handle):
        """Perform PID control to stabilize the robot around the target position."""
        self.get_logger().info(f'Starting stabilization to target: X={goal_handle.request.targetx}, Y={goal_handle.request.targety}')

        result = GoTable.Result()
        #target_x = goal_handle.request.targetx
        #target_y = goal_handle.request.targety
        target_x = 400
        target_y = 300

        # Reset PID integral terms and errors
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_x_error = 0.0
        self.prev_y_error = 0.0

        while rclpy.ok():
            # Calculate the error relative to the target
            x_error = target_x - self.current_x
            y_error = target_y - self.current_y

            # Break the loop if the position is within the thresholds
            if abs(x_error) < self.threshold_x and abs(y_error) < self.threshold_y:
                self.get_logger().info('Target reached within threshold.')
                force = Vector3(x=0.0, y=0.0, z=0.0)
                self.power.publish(force)
                time.sleep(10)
                result.success = True
                goal_handle.succeed()
                return result

            # PID calculations for X-axis
            self.integral_x += x_error
            derivative_x = x_error - self.prev_x_error
            output_x = (self.kp_x * x_error) + (self.ki_x * self.integral_x) + (self.kd_x * derivative_x)

            # PID calculations for Y-axis
            self.integral_y += y_error
            derivative_y = y_error - self.prev_y_error
            output_y = (self.kp_y * y_error) + (self.ki_y * self.integral_y) + (self.kd_y * derivative_y)

            # Update previous errors
            self.prev_x_error = x_error
            self.prev_y_error = y_error

            # Publish the calculated force
            force = Vector3(x=-output_y, y=-output_x, z=0.0)
            self.power.publish(force)

            # Log the force and PID output values
            self.logx.publish(Float64(data=x_error))
            self.logy.publish(Float64(data=y_error))
            self.get_logger().info(f'PID Output -> Force: X={force.x}, Y={force.y}, Z={force.z}', skip_first=True, throttle_duration_sec=1.5)
            # Sleep for a short duration to allow the PID loop to function
            rclpy.spin_once(self, timeout_sec=0.1)

        # If loop exits unexpectedly, return failure
        self.get_logger().info('Failed to stabilize to target.')
        result.success = False
        result.currx = self.current_x
        result.curry = self.current_y
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)

    node = GoTableServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
