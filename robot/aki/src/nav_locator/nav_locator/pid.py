import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Path
from rcl_interfaces.msg import SetParametersResult


from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseArray, Vector3
from sensor_msgs.msg import Imu
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Empty, Float64, Bool, Float32, Int32
import time
import math
import random

class rovlocnav(Node):

    def __init__(self):
        super().__init__('rovloc_nav')

        # Declare PID gain parameters
        self.declare_parameter('kp_x', 0.3)
        self.declare_parameter('ki_x', 0.0)
        self.declare_parameter('kd_x', 0.0)

        self.declare_parameter('kp_y', 0.3)
        self.declare_parameter('ki_y', 0.0)
        self.declare_parameter('kd_y', 0.0)

        self.declare_parameter('kp_z', 4.0)
        self.declare_parameter('ki_z', 0.0)
        self.declare_parameter('kd_z', 0.0)

        # Get the PID parameters
        self.kp_x = self.get_parameter('kp_x').value
        self.ki_x = self.get_parameter('ki_x').value
        self.kd_x = self.get_parameter('kd_x').value

        self.kp_y = self.get_parameter('kp_y').value
        self.ki_y = self.get_parameter('ki_y').value
        self.kd_y = self.get_parameter('kd_y').value

        self.kp_z = self.get_parameter('kp_z').value
        self.ki_z = self.get_parameter('ki_z').value
        self.kd_z = self.get_parameter('kd_z').value

        self.declare_parameter('threshold_x', 0.75)
        self.declare_parameter('threshold_y', 0.75)

        self.threshold_x = self.get_parameter('threshold_x').value
        self.threshold_y = self.get_parameter('threshold_y').value



        self.init_yaw = 0.0 
        self.init_yaw_slam = 0.0

        # PID variables initialization
        self.error_sum_x = 0.0
        self.last_error_x = 0.0

        self.error_sum_y = 0.0
        self.last_error_y = 0.0

        self.error_sum_z = 0.0
        self.last_error_z = 0.0

        # Target positions
        self.target_x = -1.0
        self.target_y = 2.0
        self.target_z = 0.0
        self.table_id = 1  # Default table ID, can be changed by parameter

        self.declare_parameter('target_x', self.target_x)
        self.declare_parameter('target_y', self.target_y)
        self.declare_parameter('target_z', self.target_z)
        self.declare_parameter('init_yaw', self.init_yaw)

        self.force_x = 0.0
        self.force_y = 0.0

        self.dt = 0.1



        self.use_sim = False
        self.use_imu = True
        self.use_water_linked = True
        self.use_slam = False
        # Subscriptions

        # depth navigation
        self.locator_depth = 0.5  # this is also the yolo_depth. There is no change in depth between yolo and locator navigation. yolo then takes current depth and tries to stabilize around.
        # this value is hardcoded for opfikon. In field tests a locator_distance value of 3.2 (if same value as yolo_distance) or higher is used. But not suitable at opfikon.
        self.maintain_depth = False  # Indicator if depth_navigation is active
        self.declare_parameter('use_with_depth', True) # if we want to include depth navigation in the PID controller
        self.use_with_depth = self.get_parameter('use_with_depth').value


        self.log_locator_pid_publisher = self.create_publisher(Float32MultiArray, '/log/locator_pid', 10)
        self.param_map = {
            'kp_x': 'kp_x',
            'ki_x': 'ki_x',
            'kd_x': 'kd_x',
            'kp_y': 'kp_y',
            'ki_y': 'ki_y',
            'kd_y': 'kd_y',
            'kp_z': 'kp_z',
            'ki_z': 'ki_z',
            'kd_z': 'kd_z',
            'threshold_x': 'threshold_x',
            'threshold_y': 'threshold_y',
            'target_x': 'target_x',
            'target_y': 'target_y',
            'target_z': 'target_z',
            'init_yaw': 'init_yaw'
        }

 

        if self.use_sim == True:
            self.create_subscription(PoseArray, '/model/own/pose', self.pose_array_callback, 10)
        if self.use_water_linked == True: 
            self.create_subscription(Vector3, '/waterlinked/position', self.waterlinked_callback, 10)
        
        if self.use_imu and not self.use_sim:
            self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        if self.use_sim and self.use_imu:
            #IMU Datan for Yaw
            self.create_subscription(Imu, '/imu/data', self.imu_callback_sim, 10)
        
        if self.use_slam:
            #SLAM for Orientation
            self.create_subscription(Path, '/camera_path', self.slam_path_array_callback, 10)

        #self.create_subscription(Float64, '/waterlinked/orientation', self.waterlinked_callback_orientation, 10)

        # Publisher
        self.create_subscription(Int32, '/table_id', self.table_id_callback, 10)
        self.target_publisher = self.create_publisher(Vector3, '/target_locateur', 10)

        self.power_publisher = self.create_publisher(Vector3, '/power', 10)

        self.create_subscription(Bool, '/calibrate/imu', self.imu_calibration, 10)

        self.bcu_publisher = self.create_publisher(Float32, '/depth_desired', 10)
        self.add_on_set_parameters_callback(self.parameter_update_callback)



        # Action Server
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'rov_loc_nav_pid',
            self.execute_callback
        )

        # State variables
        self.yaw = 0.0  
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.get_logger().info(
            f'RovLocator PID Action server started\n'
            f'PID Gains:\n'
            f'X - kp: {self.kp_x}, ki: {self.ki_x}, kd: {self.kd_x}\n'
            f'Y - kp: {self.kp_y}, ki: {self.ki_y}, kd: {self.kd_y}\n'
            f'Z - kp: {self.kp_z}, ki: {self.ki_z}, kd: {self.kd_z}')
        self.get_logger().info(
            f'Thresholds set: threshold_x = {self.threshold_x}, threshold_y = {self.threshold_y}')

    def parameter_update_callback(self, params):
        for param in params:
            if param.name in self.param_map:
                setattr(self, self.param_map[param.name], param.value)
                self.get_logger().info(f"Updated {param.name} to {param.value}")
        return SetParametersResult(successful=True)
    def imu_calibration(self, msg):
        if msg.data:
            self.get_logger().info('IMU calibration requested, setting initial yaw to current yaw')


            self.get_logger().info(
                f"Init Yaw:{-1*(self.yaw + self.init_yaw)}\n"
                f"\n========== PID DEBUG INFO ==========\n"
                f"Current Position -> x: {self.x:.2f}, y: {self.y:.2f}, z: {self.z:.2f}\n"
                f"Target Position  -> x: {self.target_x:.2f}, y: {self.target_y:.2f}, z: {self.target_z:.2f}\n"
                f"Yaw (rad): {self.yaw:.2f} ({math.degrees(self.yaw):.2f} deg)\n\n"
                f"corrected yaw rad : {self.yaw-self.init_yaw:.2f} deg \n\n"

                f"=====================================", throttle_duration_sec=1.0
            )
            if self.use_imu:
                # Get the current yaw from the IMU
                self.init_yaw = -1*(self.yaw + self.init_yaw)
        if not msg.data:


            self.get_logger().info(
                f"\n========== PID DEBUG INFO ==========\n"
                f"Current Position -> x: {self.x:.2f}, y: {self.y:.2f}, z: {self.z:.2f}\n"
                f"Target Position  -> x: {self.target_x:.2f}, y: {self.target_y:.2f}, z: {self.target_z:.2f}\n"
                f"Yaw (rad): {self.yaw:.2f} ({math.degrees(self.yaw):.2f} deg)\n\n"
                f"corrected yaw rad : {self.yaw-self.init_yaw:.2f} deg \n\n"

                f"=====================================", throttle_duration_sec=1.0
            )
            self.get_logger().info(f'current init yaw = {self.init_yaw}')

    def slam_path_array_callback(self, msg):        
        if len(msg.poses) == 0:
            self.get_logger().warn('Receives an empty Path message')
            return

        newest_pose = msg.poses[-1] #newest pose is at last index of path
        #self.x = newest_pose.pose.position.x
        #self.y = newest_pose.pose.position.y
        #self.get_logger().info(f"actual pose received ({self.x:.2f}, {self.y:.2f})")
        #self.z = pose.z  # Update z position

        #orientation
        q = newest_pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp) - self.init_yaw_slam


    def waterlinked_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

    def table_id_callback(self, msg):
        self.table_id = msg.data
        self.get_logger().info(f'Table ID updated to: {self.table_id}')

        # Update target positions based on table ID
        if self.table_id == 1:
            self.target_x = -1.0
            self.target_y = 2.0
        elif self.table_id == 2:
            self.target_x = -5.0
            self.target_y = 3.0
        else:
            self.target_x = 0.0
            self.target_y = 0.0

        self.target_publisher.publish(Vector3(x=self.target_x, y=self.target_y, z=self.target_z))
        self.get_logger().info(f'Locateur target updated to: {self.target_x}, {self.target_y}, {self.target_z}')

    def imu_callback_sim(self, msg):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = -1*(math.atan2(siny_cosp, cosy_cosp) -self.init_yaw)

    def pose_array_callback(self, msg):
        pose = msg.poses[6].position
        self.x = pose.x
        self.y = pose.y
        self.z = pose.z

    def add_noise(self, value, variance=0.2):
        return value + random.uniform(-variance, variance)

    def publish_zero_power(self):
        power_msg = Vector3()
        power_msg.x = 0.0
        power_msg.y = 0.0
        power_msg.z = 0.0
        self.power_publisher.publish(power_msg)

    def execute_callback(self, goal_handle):
        #self.target_x = goal_handle.request.pose.pose.position.x
        #self.target_y = goal_handle.request.pose.pose.position.y
        self.target_z = -2.8

        self.get_logger().info(
            f'Executing goal...\n'
            f'Target X: {self.target_x:.2f}, Target Y: {self.target_y:.2f}'
        )
        self.target_publisher.publish(Vector3(x=self.target_x, y=self.target_y, z=self.target_z))


        # PID variables initialization
        self.error_sum_x = 0.0
        self.last_error_x = 0.0

        self.error_sum_y = 0.0
        self.last_error_y = 0.0

        self.error_sum_z = 0.0
        self.last_error_z = 0.0


        self.force_x = 0.0
        self.force_y = 0.0

        result = NavigateToPose.Result()
        hold_start_time = None

        try:
            while rclpy.ok():
                self.timer_callback(goal_handle)

                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal canceled.')
                    self.publish_zero_power()
                    goal_handle.canceled()
                    result.result = Empty()
                    return result

                in_target_x = abs(self.target_x - self.x) < self.threshold_x
                in_target_y = abs(self.target_y - self.y) < self.threshold_y
                in_target = in_target_x and in_target_y

                if self.use_with_depth:
                    if not self.maintain_depth:  
                        bcu_msg = Float32()
                        bcu_msg.data = self.locator_depth
                        self.maintain_depth = True
                        self.bcu_publisher.publish(bcu_msg)

                if in_target:
                    if hold_start_time is None:
                        hold_start_time = time.time()
                    elif time.time() - hold_start_time >= 5.0:
                        self.get_logger().info('Held target position for 10 seconds. Goal succeeded.')
                        self.publish_zero_power()
                        goal_handle.succeed()
                        self.maintain_depth = False
                        result.result = Empty()
                        return result
                else:
                    hold_start_time = None

                time.sleep(self.dt)
        except Exception as e:
            self.get_logger().error(f"Error during navigation: {e}")
            if not goal_handle.is_cancel_requested:
                goal_handle.abort()
            result.result = Empty()
            return result

    def timer_callback(self, goal_handle):
        if self.use_sim:
            error_x = self.target_x - self.add_noise(self.x)
            error_y = self.target_y - self.add_noise(self.y)
            error_z = self.target_z - self.add_noise(self.z)
        else:
            error_x = self.target_x - self.x
            error_y = self.target_y - self.y
            error_z = self.target_z - self.z

        # Proportional
        proportional_x = self.kp_x * error_x
        proportional_y = self.kp_y * error_y
        proportional_z = self.kp_z * error_z

        # Integral
        self.error_sum_x += error_x * self.dt
        self.error_sum_y += error_y * self.dt
        self.error_sum_z += error_z * self.dt

        if -1.0 < self.force_x < 1.0:
            self.error_sum_x += error_x * self.dt
        else:
            self.error_sum_x = 0.0
        if -1.0 < self.force_y < 1.0:
            self.error_sum_y += error_y * self.dt
        else:
            self.error_sum_y = 0.0
        self.error_sum_z += error_z * self.dt  # Assuming no windup protection for Z-axis

        integral_x = self.ki_x * self.error_sum_x
        integral_y = self.ki_y * self.error_sum_y
        integral_z = self.ki_z * self.error_sum_z

        # Derivative
        derivative_x = self.kd_x * (error_x - self.last_error_x) / self.dt
        derivative_y = self.kd_y * (error_y - self.last_error_y) / self.dt
        derivative_z = self.kd_z * (error_z - self.last_error_z) / self.dt

        self.last_error_x = error_x
        self.last_error_y = error_y
        self.last_error_z = error_z

        # World frame
        world_force_x = proportional_x + integral_x + derivative_x
        world_force_y = proportional_y + integral_y + derivative_y
        world_force_z = proportional_z  # Z-D only P control

        # Convert to local frame
        force_x = world_force_x * math.cos(self.yaw) + world_force_y * math.sin(self.yaw)
        force_y = -world_force_x * math.sin(self.yaw) + world_force_y * math.cos(self.yaw)

        self.force_x = max(-1.0, min(1.0, force_x))
        self.force_y = -max(-1.0, min(1.0, force_y))


        # Prepare and publish the error and yaw data
        log_msg = Float32MultiArray()
        log_msg.data = [error_x, error_y, error_z, self.yaw]
        self.log_locator_pid_publisher.publish(log_msg)

        
        # Publish
        power_msg = Vector3()
        power_msg.x = self.force_x
        power_msg.y = self.force_y
        power_msg.z = 0.0
        self.power_publisher.publish(power_msg)

        self.get_logger().info(
            f"\n========== PID DEBUG INFO ==========\n"
            f"Current Position -> x: {self.x:.2f}, y: {self.y:.2f}, z: {self.z:.2f}\n"
            f"Target Position  -> x: {self.target_x:.2f}, y: {self.target_y:.2f}, z: {self.target_z:.2f}\n"
            f"Yaw (rad): {self.yaw:.2f} ({math.degrees(self.yaw):.2f} deg)\n\n"
            f"corrected yaw rad : {self.yaw-self.init_yaw:.2f} deg \n\n"
            f"Errors:\n"
            f"  x: {error_x:.2f}\n"
            f"  y: {error_y:.2f}\n"
            f"  z: {error_z:.2f}\n\n"
            f"PID Components:\n"
            f"  Proportional -> x: {proportional_x:.2f}, y: {proportional_y:.2f}, z: {proportional_z:.2f}\n"
            f"  Integral     -> x: {integral_x:.2f}, y: {integral_y:.2f}, z: {integral_z:.2f}\n"
            f"  Derivative   -> x: {derivative_x:.2f}, y: {derivative_y:.2f}, z: {derivative_z:.2f}\n\n"
            f"World Frame Forces:\n"
            f"  x: {world_force_x:.2f}, y: {world_force_y:.2f}, z: {world_force_z:.2f}\n"
            f"Local Frame (Power Msg):\n"
            f"  x: {power_msg.x:.2f}, y: {power_msg.y:.2f}, z: {power_msg.z:.2f}\n"
            f"=====================================", throttle_duration_sec=1.0
        )

def main(args=None):
    rclpy.init(args=args)
    node = rovlocnav()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()