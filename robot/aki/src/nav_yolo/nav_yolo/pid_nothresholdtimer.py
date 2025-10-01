import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup # if all callback function should run in parallel

from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
import time


from nav2_msgs.action import NavigateToPose  
from std_msgs.msg import Empty


from rcl_interfaces.msg import SetParametersResult

# Inside your class (after declaring parameters)


class NavYoloPID(Node):

    def __init__(self):
        super().__init__('nav_yolo_server')

        self.cb_group_table = ReentrantCallbackGroup()
        self.cb_group_action = ReentrantCallbackGroup()

        # Subscription to track the table's position
        self.subscription = self.create_subscription(Vector3, '/tracked/table', self.detection_callback, 1,
                                                     callback_group=self.cb_group_table)

        # Publishers for force and PID logs

        self.power = self.create_publisher(Vector3, '/power', 10)
        self.logx = self.create_publisher(Float64, '/logpidtablex', 10)
        self.logy = self.create_publisher(Float64, '/logpidtabley', 10)


        # Action server for GoTable action
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'yolo_nav_pid',
            self.execute_callback,
            callback_group=self.cb_group_action,
            )

        # Declare PID parameters
        self.declare_parameter('kp_x', 0.05)
        self.declare_parameter('ki_x', 0.0)
        self.declare_parameter('kd_x', 0.0)
        self.declare_parameter('kp_y', 0.05)
        self.declare_parameter('ki_y', 0.0)
        self.declare_parameter('kd_y', 0.0)
        self.declare_parameter('threshold_x', 50.0)
        self.declare_parameter('threshold_y', 50.0)
        #depended on the camera resolution in usb camera config
        self.declare_parameter('middle_x', 320.0)
        self.declare_parameter('middle_y', 240.0)

        # Get  parameters
        self.threshold_x = self.get_parameter('threshold_x').get_parameter_value().double_value
        self.threshold_y = self.get_parameter('threshold_y').get_parameter_value().double_value

        self.kp_x = self.get_parameter('kp_x').get_parameter_value().double_value
        self.ki_x = self.get_parameter('ki_x').get_parameter_value().double_value
        self.kd_x = self.get_parameter('kd_x').get_parameter_value().double_value

        self.kp_y = self.get_parameter('kp_y').get_parameter_value().double_value
        self.ki_y = self.get_parameter('ki_y').get_parameter_value().double_value
        self.kd_y = self.get_parameter('kd_y').get_parameter_value().double_value

        self.middle_x = self.get_parameter('middle_x').get_parameter_value().double_value
        self.middle_y = self.get_parameter('middle_y').get_parameter_value().double_value
    

        self.get_logger().info(f'PID Gains for X: kp={self.kp_x}, ki={self.ki_x}, kd={self.kd_x}', skip_first=True, throttle_duration_sec=1.5)
        self.get_logger().info(f'PID Gains for Y: kp={self.kp_y}, ki={self.ki_y}, kd={self.kd_y}', skip_first=True, throttle_duration_sec=1.5)



        # Inside your node class, after declaring and getting parameters
        self.param_map = {
            'kp_x': 'kp_x',
            'ki_x': 'ki_x',
            'kd_x': 'kd_x',
            'kp_y': 'kp_y',
            'ki_y': 'ki_y',
            'kd_y': 'kd_y',
            'threshold_x': 'threshold_x',
            'threshold_y': 'threshold_y',
            'middle_x': 'middle_x',
            'middle_y': 'middle_y'
        }

        # Add the parameter callback
        self.add_on_set_parameters_callback(self.parameter_update_callback)


        # Initialize PID state
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_x_error = 0.0
        self.prev_y_error = 0.0

        # Variables to store the current position
        self.current_x = 0.0
        self.current_y = 0.0

        #Position Logger
        self.pos_log=False

        # Store last logged coordinates (only updated when logging occurs)
        self.last_logged_x = None
        self.last_logged_y = None
        self.log_threshold = 10  # Threshold for logging changes



    def parameter_update_callback(self, params):
        for param in params:
            if param.name in self.param_map:
                setattr(self, self.param_map[param.name], param.value)
                self.get_logger().info(f"Updated {param.name} to {param.value}")
        return SetParametersResult(successful=True)

    def detection_callback(self, detection: Vector3):
        """Update the current position based on tracked table data."""
        self.current_x = detection.x
        self.current_y = detection.y
        if self.pos_log:
            if (self.last_logged_x is None or self.last_logged_y is None or 
                abs(self.last_logged_x - self.current_x) > self.log_threshold or abs(self.last_logged_y - self.current_y) > self.log_threshold):
                self.get_logger().info(f'Current position: X={self.current_x}, Y={self.current_y}')
                self.last_logged_x = self.current_x  # Update logged position after logging
                self.last_logged_y = self.current_y # Update logged position after logging

                
    def execute_callback(self, goal_handle):
        """Perform PID control to stabilize the robot around the target position."""
        #self.get_logger().info(f'Starting stabilization to target: X={goal_handle.request.targetx}, Y={goal_handle.request.targety}')

        result = NavigateToPose.Result()
        result.result = Empty()  

        #target_x = goal_handle.request.targetx
        #target_y = goal_handle.request.targety


        # Reset PID integral terms and errors
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_x_error = 0.0
        self.prev_y_error = 0.0

        while rclpy.ok():
            # Calculate the error relative to the target
            x_error = self.middle_x - self.current_x
            y_error = self.middle_y - self.current_y

            # Break the loop if the position is within the thresholds
            if abs(x_error) < self.threshold_x and abs(y_error) < self.threshold_y:
                self.get_logger().info('Target reached within threshold.')
                force = Vector3(x=0.0, y=0.0, z=0.0)
                self.power.publish(force)
                time.sleep(10)
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
            self.pos_log=True
            # Sleep for a short duration to allow the PID loop to function
            rclpy.spin_once(self, timeout_sec=0.1)

        # If loop exits unexpectedly, return failure
        self.get_logger().info('Failed to stabilize to target.')
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)

    node = NavYoloPID()
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