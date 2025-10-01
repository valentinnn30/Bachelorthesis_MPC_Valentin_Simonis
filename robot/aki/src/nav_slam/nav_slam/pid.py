import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup # if all callback function should run sequentially
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from slam_interface.action import ActivateSlam
import numpy as np

from std_msgs.msg import Float32MultiArray


from collections import deque
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math
from geometry_msgs.msg import Vector3
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

class overtable(Node):

    def __init__(self):
        super().__init__('nav_slam_pid_controller')

        self.cb_group_path = ReentrantCallbackGroup()
        self.cb_group_action = ReentrantCallbackGroup()
        self.cb_group_depth = ReentrantCallbackGroup()
        self.cb_group_distance = ReentrantCallbackGroup()

        self.declare_parameter('test?', False)
        self.test_without_depth = self.get_parameter('test?').get_parameter_value().bool_value

        self.subscription = self.create_subscription(
            Path,
            '/camera_path',
            self.path_array_callback,
            10,
            callback_group=self.cb_group_path
        )
        
        self.distance_subscription = self.create_subscription(
            Float32,
            'ultrasonic_distance',
            self.distance_callback,
            10,
            callback_group=self.cb_group_distance
        )

        self.create_subscription(Vector3, '/waterlinked/position', self.waterlinked_callback, 10)
        self.create_subscription(Vector3, '/target_locateur', self.targetlocateur_callback, 10)

        self.feed_client = ActionClient(self, ActivateSlam, 'feeding')
        self.locateur_action_client = ActionClient(self, NavigateToPose, 'rov_loc_nav_pid')



        self._action_server = ActionServer(
            self,
            NavigateToPose, 'slam_nav_pid', self.execute_callback,
            callback_group=self.cb_group_action)


        self.log_locator_pid_publisher = self.create_publisher(Float32MultiArray, '/log/slam_pid', 10)

        self.power_publisher = self.create_publisher(Vector3, '/power', 10)
        self.depth_publisher = self.create_publisher(Float32, '/depth_desired', 10)
        self.distance_publisher = self.create_publisher(Float32, '/desired_distance', 10)
        self.depth_subscriber = self.create_subscription(Float32, '/depth', self.depth_callback, 10, callback_group=self.cb_group_depth)

        # PID parameters
        self.declare_parameter('kp_x', 0.35) #0.35 #0.25 #0.7
        self.declare_parameter('ki_x', 0.01) #0.2 #0.005
        self.declare_parameter('kd_x', 0.15) #0.1 #0.15 
        self.declare_parameter('kp_y', 0.35) #0.35
        self.declare_parameter('ki_y', 0.01) #0.2
        self.declare_parameter('kd_y', 0.15) #0.1
        self.declare_parameter('delta_depth', 0.10)
        self.declare_parameter('slam_height', 0.7)
        self.declare_parameter('target_height', 0.3)
        self.declare_parameter('slam_threshold', 0.04)
        self.declare_parameter('height_threshold', 0.1)
        self.declare_parameter('feeding_depth', 2.5)



        self.kp_x = self.get_parameter('kp_x').get_parameter_value().double_value
        self.ki_x = self.get_parameter('ki_x').get_parameter_value().double_value
        self.kd_x = self.get_parameter('kd_x').get_parameter_value().double_value
        self.kp_y = self.get_parameter('kp_y').get_parameter_value().double_value
        self.ki_y = self.get_parameter('ki_y').get_parameter_value().double_value
        self.kd_y = self.get_parameter('kd_y').get_parameter_value().double_value
        self.delta_depth = self.get_parameter('delta_depth').get_parameter_value().double_value
        self.slam_height = self.get_parameter('slam_height').get_parameter_value().double_value
        self.target_height = self.get_parameter('target_height').get_parameter_value().double_value
        self.slam_threshold = self.get_parameter('slam_threshold').get_parameter_value().double_value
        self.height_threshold = self.get_parameter('height_threshold').get_parameter_value().double_value
        self.feeding_depth = self.get_parameter('feeding_depth').get_parameter_value().double_value

        self.param_map = {
            'kp_x': 'kp_x',
            'ki_x': 'ki_x',
            'kd_x': 'kd_x',
            'kp_y': 'kp_y',
            'ki_y': 'ki_y',
            'kd_y': 'kd_y',
            'delta_depth': 'delta_depth',
            'slam_height': 'slam_height',
            'target_height': 'target_height',
            'slam_threshold' : 'slam_threshold',
            'height_threshold' : 'height_threshold',
            'feeding_depth' : 'feeding_depth'
        }
        self.add_on_set_parameters_callback(self.parameter_update_callback)

        # State variables
        self.yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.dt = 0.1
        self.error_sum_x = 0.0
        self.last_error_x = 0.0
        self.error_sum_y = 0.0
        self.last_error_y = 0.0

        self.x_locateur = 0.0
        self.y_locateur = 0.0
        self.z_locateur = 0.0
        self.target_xlocateur = -1.0
        self.target_ylocateur = 2.0
        self.target_zlocateur = 0.0
        self.error_handling = False # this is to check if the error handling is active, so we can stop the PID controller
        self.out_of_bounds = False # this is to check if the robot is out of bounds, so we can stop the PID controller

        # Depth logic
        #self.target_height = 0.3 #this is the height we want to move above the corals imediately before feeding
        #self.slam_height = 0.7 #we want to move to this distance of the table
        self.mean_value = 0.0
        self.values = deque(maxlen=40) #take all values from the last 4 seconds (robot is directly above table so we assume many values but from floor and grid)
        self.delta_height = 0.0 
        self.depthnav_active = False
        self.depth_reached = False #this is sent by the STM32 when the depth is reached -> unfortenately not the case...
        self.depth_reached_stable = False #this is to check if the depth is reached stable, so we can start feeding
        #self.depth_reached_after = False #this is to check if the depth is reached after the feeding
        self.depth = 0.0 #the currernt depth of our robot
        self.start_depth = 0.0 #depth at the moment where the PID in z-dir to go to a distance of 30cm above the corals is started
        self.feeding_done = False
        self.counter = 0
        self.goal_depth = 0.0 # new version of the depth that we send to STM, which is the delta height + the actual depth: the desired depth
        self.goal_distance = 0.0 #this is the distance we want to reach with the ultrasonic sensor
        self.feeding_in_progress = False
    
    def parameter_update_callback(self, params):
        for param in params:
            if param.name in self.param_map:
                setattr(self, self.param_map[param.name], param.value)
                self.get_logger().info(f"Updated {param.name} to {param.value}")
        return SetParametersResult(successful=True)

    def path_array_callback(self, msg):
        if len(msg.poses) == 0:
            self.get_logger().warn('Receives an empty Path message')
            return

        newest_pose = msg.poses[-1] #newest pose is at last index of path
        self.x = newest_pose.pose.position.x
        self.y = newest_pose.pose.position.y
        #self.get_logger().info(f"actual pose received ({self.x:.2f}, {self.y:.2f})")
        #self.z = pose.z  # Update z position

        #orientation
        q = newest_pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    
    def waterlinked_callback(self, msg):
        self.x_locateur = msg.x
        self.y_locateur = msg.y
        self.z_locateur = msg.z

    def targetlocateur_callback(self, msg):
        self.target_xlocateur = msg.x
        self.target_ylocateur = msg.y
        self.target_zlocateur = msg.z

        #self.get_logger().info(f"Target locateur received ({self.target_xlocateur:.2f}, {self.target_ylocateur:.2f})")
        #self.get_logger().info(f"Actual locateur received ({self.x_locateur:.2f}, {self.y_locateur:.2f})")


    def distance_callback(self, msg):
        if msg.data == 0.0:
            return  # Ignore zero distance
        self.values.append(msg.data)
        # if len(self.values) == 10:  
        # Always compute mean, otherwise at the beginning mean is zero
        """self.mean_value = np.mean(self.values)
        #self.get_logger().info(f"Mean of last 10 values: {self.mean_value:.2f}")"""
        # Robust min-like filter
        sorted_vals = sorted(self.values)
        lowest_vals = sorted_vals[:5]  # Take the lowest 5 values
        self.mean_value = sum(lowest_vals) / len(lowest_vals) 
        #self.get_logger().info(f"Mean of lowest 5 values: {self.mean_value:.2f}") # check if this value is suitable, otherwise maybe use filter with hardcoded tableheight information

    def depth_callback(self, msg):
        self.depth = msg.data




    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        target_pose = goal_handle.request.pose
        self.target_x = target_pose.pose.position.x
        self.target_y = target_pose.pose.position.y

        self.get_logger().info(f"Target pose received ({self.target_x:.2f}, {self.target_y:.2f})")

        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()

        # Run the timer logic inside the action loop
        self.get_logger().info('Starting timer-based navigation...')
        start_time = time.time()
        try:
            while rclpy.ok():
                
                if self.error_handling:
                    if (abs(self.target_xlocateur - self.x_locateur) > 1.5 or 
                        abs(self.target_ylocateur - self.y_locateur) > 1.5):
                        if not self.out_of_bounds:
                            self.get_logger().info('Out of bounds detected')
                            self.out_of_bounds = True
                            self.threshold_start_time = time.time()
                        elif time.time() - self.threshold_start_time >= 5.0:  # Check if 5 seconds have passed
                            self.get_logger().info('Target out of bounds for 5 seconds.')
                            # Create an empty goal for the action client
                            goal_msg = NavigateToPose.Goal()  # You can use a specific goal if needed, here it's empty

                            # Send the goal to the action server
                            self.locateur_action_client.wait_for_server()  # Make sure the action server is ready
                            self.get_logger().info("Sending action to 'rov_loc_nav_pid' to resolve out of bounds.")

                            # Send the goal asynchronously
                            goal_future = self.locateur_action_client.send_goal_async(goal_msg)
                            rclpy.spin_until_future_complete(self, goal_future)

                            goal_handle = goal_future.result()

                            if not goal_handle.accepted:
                                self.get_logger().error(f"Locateur action was not accepted!")
                                continue

                            bcu_msg = Float32()
                            bcu_msg.data = 2.3
                            self.depth_publisher.publish(bcu_msg)

                            result_future = goal_handle.get_result_async()
                            self.get_logger().info("Waiting for the goal to be completed...")
                            rclpy.spin_until_future_complete(self, result_future)
                            result = result_future.result()

                            if result.status != 3:  # 3 indicates "SUCCEEDED" in ROS 2
                                self.get_logger().error(f"Failed to resolve out of bounds. Status: {result.status}")
                            else:
                                self.get_logger().info("Successfully resolved out of bounds.")
                            
                            bcu_msg = Float32()
                            bcu_msg.data = 2.3
                            self.depth_publisher.publish(bcu_msg)
                            #self.out_of_bounds_stable = True
                    else:
                        self.out_of_bounds = False
                
                
                # Timer callback logic (PID control) is executed here
                self.timer_callback()

                # Check if goal has been canceled
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal canceled.')
                    goal_handle.canceled()
                    return result
                
                # This is the test mode, where we only check for horizontal position
                if self.test_without_depth:
                    
                    if (abs(self.target_x - self.x) < self.slam_threshold and
                        abs(self.target_y - self.y) < self.slam_threshold  
                    ):  
                        self.get_logger().info('Target position reached.')
                        goal_handle.succeed()
         

                        return result
                #this is the normal mode, where we check for depth and horizontal position, including feeding
                else:
                    
                    #while horizontal position not reached yet
                    if((abs(self.target_x - self.x) > self.slam_threshold or
                        abs(self.target_y - self.y) > self.slam_threshold) and 
                        not self.depthnav_active and not self.feeding_in_progress
                    ):
                        #self.get_logger().info('Continue SLAM and z-PID')
                        if self.counter == 0:  
                            bcu_msg = Float32()
                            bcu_msg.data = 2.2
                            distance_msg = Float32()
                            distance_msg.data = self.slam_height
                            self.counter = 1
                        
                            #self.depth is actual depth, could send this or only 0 (the delta)
                            self.depth_publisher.publish(bcu_msg)
                            self.distance_publisher.publish(distance_msg)
                    
                    # # Check if we have reached the target, first check depth, then position
                    # if (abs(self.depth - self.goal_depth) <= 0.1):#mit Toleranz von 0.01 m, 30cm über den Korallen
                    #     self.depth_reached = True
                    # else:
                    #     self.depth_reached = False

                    if (abs(self.depth - self.goal_depth) <= self.height_threshold):
                        if not self.depth_reached:
                            self.get_logger().info('depth reached')
                            self.depth_reached = True
                            self.threshold_start_time = time.time()
                        elif time.time() - self.threshold_start_time >= 5.0:  # Check if 5 seconds have passed
                            self.get_logger().info('Target  depth reached within threshold for 5 seconds.')
                            self.depth_reached_stable = True
                    else:
                        self.depth_reached = False

                            
                                
                
                    """if (abs(self.mean_value - self.goal_distance) <= self.height_threshold):#mit Toleranz von 0.01 m, 30cm über den Korallen
                        self.depth_reached = True
                    else:
                        self.depth_reached = False"""
                    
                    if (#not important if within threshold, PID continues trying to hold position, this is enough
                        self.depth_reached and 
                        self.feeding_done and not self.feeding_in_progress 
                    ): 
                        self.get_logger().info('Process finished, move to next target.')
                        bcu_msg = Float32()
                        distance_msg = Float32()
                        #bcu_msg.data = self.depth # now back to SLAM-height, now important to remain at this height
                        bcu_msg.data = 2.3 # worked with 2.0 but large distance
                        distance_msg.data = self.slam_height
                        self.depth_publisher.publish(bcu_msg)
                        self.distance_publisher.publish(distance_msg)
                        goal_handle.succeed()
                        
                        # Reset all variables
                        self.delta_height = 0.0 
                        self.depthnav_active = False
                        self.depth_reached = False 
                        self.start_depth = 0.0 
                        self.feeding_done = False
                        #self.counter = 0
                        self.goal_depth = 0.0 
                        self.goal_distance = 0.0

                        return result

                    if (
                        abs(self.target_x - self.x) < self.slam_threshold + 1000.0  and
                        abs(self.target_y - self.y) < self.slam_threshold + 1000.0 and 
                        self.depth_reached_stable and not self.feeding_done and not self.feeding_in_progress

                    ):  
                        self.get_logger().info('Target depth and position reached.') #now feeding
                        self.feeding_in_progress = True
                        #now stay at this detph
                        bcu_msg = Float32()
                        distance_msg = Float32()
                        #bcu_msg.data = self.depth
                        bcu_msg.data = self.feeding_depth +0.15
                        distance_msg.data = self.target_height
                        self.depth_publisher.publish(bcu_msg) #additionally min distance with distance sensor, if possible
                        self.distance_publisher.publish(distance_msg)
                        time.sleep(0.1)
                        
                        #stop moving before feeding
                        # self.get_logger().info('start feeding started')
                        # force = Vector3(x=0.0, y=0.0, z=0.0)
                        # self.power_publisher.publish(force)
                        # time.sleep(1.0)
                        #Pause the loop by waiting for a new action to succeed

                        food_goal = ActivateSlam.Goal() #use activateSlam action, no need to define new one
                        self.feed_client.wait_for_server()
                        food_goal.activate = True
                        future = self.feed_client.send_goal_async(food_goal)
                        self.get_logger().info('start feeding started')
                        future.add_done_callback(self._feeding_goal_response_callback)
                        
                        # self.feeding_done = True
                        # self.delta_height = self.mean_value - self.slam_height
                        # self.goal_depth = self.delta_height + self.depth
                        # self.goal_distance = self.slam_height
                        # bcu_msg = Float32()
                        # distance_msg = Float32()
                        # self.goal_depth = 2.0
                        # bcu_msg.data = self.goal_depth
                        # distance_msg.data = self.slam_height
                        # self.depth_publisher.publish(bcu_msg)
                        # self.distance_publisher.publish(distance_msg)
                        # self.depth_reached = False
                        # self.depth_reached_stable = False
                        # self.feeding_in_progress = False
                        # self.get_logger().info('Feeding completed.')



                        # Wait for the result of the goal, proceeded in helper function

                    if (
                        abs(self.target_x - self.x) < self.slam_threshold and
                        abs(self.target_y - self.y) < self.slam_threshold and 
                        not self.depthnav_active and not self.feeding_in_progress
                        #and
                        #abs(self.target_z - self.z) < 0.01
                    ):
                        self.get_logger().info('Target position reached.') # now go to depth for feeding
                        
                        self.delta_height = self.mean_value - self.target_height
                        self.goal_depth = self.delta_height + self.depth # this is the depth we want to reach
                        self.goal_distance = self.target_height # this is the distance we want to reach with the ultrasonic sensor
                        bcu_msg = Float32()
                        distance_msg = Float32()
                        self.goal_depth = self.feeding_depth + 0.1  # worked with 2.5 but distance lil to high
                        bcu_msg.data = self.feeding_depth
                        distance_msg.data = self.target_height
                        #self.start_depth = self.depth
                        self.depth_publisher.publish(bcu_msg)
                        self.distance_publisher.publish(distance_msg)
                        self.depthnav_active = True
                # Sleep for the time step
                time.sleep(self.dt)
        except Exception as e:
            self.get_logger().error(f"Error during navigation: {e}")
            if not goal_handle.is_cancel_requested:
                goal_handle.abort()
            return result

    def _feeding_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Feeding goal was not accepted!")
            return

        self.get_logger().info("Feeding goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._feeding_result_callback)

    def _feeding_result_callback(self, future):
        result_feeding = future.result().result
        if result_feeding.success:
            self.get_logger().info("Successfully fed.")
            self.feeding_done = True
        else:
            self.get_logger().error("Feeding failed.")

        # Wait briefly, then return to SLAM height
        time.sleep(0.1)  # Optional, not great in async code

        self.delta_height = self.mean_value - self.slam_height
        self.goal_depth = self.delta_height + self.depth
        self.goal_distance = self.slam_height
        bcu_msg = Float32()
        distance_msg = Float32()
        self.goal_depth = 2.3 # worked with 2.0 but large distance
        bcu_msg.data = self.goal_depth
        distance_msg.data = self.slam_height
        self.depth_publisher.publish(bcu_msg)
        self.distance_publisher.publish(distance_msg)
        self.depth_reached = False
        self.depth_reached_stable = False
        self.feeding_in_progress = False



    def timer_callback(self):
        # Calculate errors
            error_x = self.target_x - self.x
            error_y = self.target_y - self.y
            #error_z = self.target_z - self.z

            # Proportional terms
            proportional_x = self.kp_x * error_x
            proportional_y = self.kp_y * error_y
            #proportional_z = self.kp_z * error_z

            # Integral terms
            self.error_sum_x += error_x * self.dt
            self.error_sum_y += error_y * self.dt
            #self.error_sum_z += error_z * self.dt
            integral_x = self.ki_x * self.error_sum_x
            integral_y = self.ki_y * self.error_sum_y
            #integral_z = self.ki_z * self.error_sum_z

            # Derivative terms
            derivative_x = self.kd_x * (error_x - self.last_error_x) / self.dt
            derivative_y = self.kd_y * (error_y - self.last_error_y) / self.dt
            #derivative_z = self.kd_z * (error_z - self.last_error_z) / self.dt
            self.last_error_x = error_x
            self.last_error_y = error_y
            #self.last_error_z = error_z

            # Compute PID outputs for world frame forces
            world_force_x = proportional_x + integral_x + derivative_x
            world_force_y = proportional_y + integral_y + derivative_y
            #world_force_z = proportional_z 
            
            # Rotate forces to robot's local frame
            force_x = world_force_x * math.cos(self.yaw) + world_force_y * math.sin(self.yaw)
            force_y = -world_force_x * math.sin(self.yaw) + world_force_y * math.cos(self.yaw)

            # Cap forces at [-1, 1]
            force_x = max(-0.3, min(0.3, force_x)) 
            force_y = max(-0.3, min(0.3, force_y))


            # Log error_x and error_y to /log/slam_pid
            log_msg = Float32MultiArray()
            log_msg.data = [error_x, error_y]
            self.log_locator_pid_publisher.publish(log_msg)

            # Publish force as power
            power_msg = Vector3()
            power_msg.x = force_x
            power_msg.y = -force_y
            power_msg.z = 0.0
            self.power_publisher.publish(power_msg)

            # Log PID details
            
            """self.get_logger().info(
                f"PID Output: x={force_x}, y={force_y}, z={world_force_z}, "
                f"Errors -> x: {error_x}, y: {error_y}, z: {error_z}, "
                f"P -> x: {proportional_x}, y: {proportional_y}, z: {proportional_z}, "
                f"I -> x: {integral_x}, y: {integral_y}, z: {integral_z}, "
                f"D -> x: {derivative_x}, y: {derivative_y}, z: {derivative_z}"
            )"""

            self.get_logger().info(
                f"PID Output: x={force_x}, y={force_y}"
            )
            self.get_logger().info(
                f"Position -> x: {self.x}, y: {self.y}"
            )
            self.get_logger().info(
                f"Target -> x: {self.target_x}, y: {self.target_y}"
            )
            self.get_logger().info(
                f" Errors -> x: {error_x}, y: {error_y}"
            )
            

def main(args=None):
    rclpy.init(args=args)
    node = overtable()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()