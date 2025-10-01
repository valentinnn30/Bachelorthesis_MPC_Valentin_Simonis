import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor

from move_interface.action import Move


from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray
import numpy as np
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix, Imu
from nav2_msgs.action import NavigateToPose

import math


class overtable(Node):

    def __init__(self):
        super().__init__('over_table')
        

        self.subscription = self.create_subscription(
            Path,
            '/camera_path',
            self.path_array_callback,
            10
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        

        self._action_server = ActionServer(
            self,
            NavigateToPose, 'navigate_to_pose', self.execute_callback)

        self.power_publisher = self.create_publisher(Vector3, '/power', 10)

        self.yaw = 0.0  # Orientation (yaw angle)
        self.x = 0.0
        self.y = 0.0
        #self.z = 0.0

        self.dt = 0.1  # Time step

        # PID gains for x-direction
        self.kp_x = 0.3
        self.ki_x = 0.0
        self.kd_x = 0.0

        # PID variables for x-direction
        self.error_sum_x = 0.0
        self.last_error_x = 0.0

        # PID gains for y-direction
        self.kp_y = 0.3
        self.ki_y = 0.0
        self.kd_y = 0.0

        # PID variables for y-direction
        self.error_sum_y = 0.0
        self.last_error_y = 0.0

        # PID gains for z-direction
        """self.kp_z = 4
        self.ki_z = 0.0
        self.kd_z = 0.0

        # PID variables for z-direction
        self.error_sum_z = 0.0
        self.last_error_z = 0.0"""

        # Target positions
        self.target_x = 0.0
        self.target_y = 0.0
        #self.target_z = 0.0  # Target z-height


    def path_array_callback(self, msg):
        if len(msg.poses) == 0:
            self.get_logger().warn('Receives an emtpy Path message')
            return
        
        newest_pose = msg.poses[-1] #newest pose is at index 0 of path
        self.x = newest_pose.pose.position.x
        self.y = newest_pose.pose.position.y
        #self.get_logger().info(f"actual pose received ({self.x:.2f}, {self.y:.2f})")
        #self.z = pose.z  # Update z position

    def imu_callback(self, msg):
        # Extract yaw from quaternion
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        target_pose = goal_handle.request.pose  # The goal request has a `pose` field of type PoseStamped
        self.target_x = target_pose.pose.position.x
        self.target_y = target_pose.pose.position.y

        self.get_logger().info(f"target pose received ({self.target_x:.2f}, {self.target_y:.2f})")

        #self.target_z = -2.8
        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()

        # Run the timer logic inside the action loop
        self.get_logger().info('Starting timer-based navigation...')
        start_time = time.time()
        try:
            while rclpy.ok():
                # Timer callback logic (PID control) is executed here
                self.timer_callback()
                
                


                # Update feedback for the action client
                #feedback_msg.currx = self.x
                #feedback_msg.curry = self.y
                #goal_handle.publish_feedback(feedback_msg)
                
                # Check if goal has been canceled
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal canceled.')
                    goal_handle.canceled()
                    return result
                
                # Check if we have reached the target
                if (
                    abs(self.target_x - self.x) < 0.01 and
                    abs(self.target_y - self.y) < 0.01 #and
                    #abs(self.target_z - self.z) < 0.01
                ):
                    self.get_logger().info('Target position reached.')
                    goal_handle.succeed()
         

                    return result
                
                # Sleep for the time step
                time.sleep(self.dt)
        except Exception as e:
            self.get_logger().error(f"Error during navigation: {e}")
            if not goal_handle.is_cancel_requested:
                goal_handle.abort()
            return result
        
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
            force_x = max(-0.5, min(0.5, force_x))
            force_y = max(-0.5, min(0.5, force_y))

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
                f" Errors -> x: {error_x}, y: {error_y}"
                f" Position -> x: {self.x}, y: {self.y}"
                #f"P -> x: {proportional_x}, y: {proportional_y}"
                #f"I -> x: {integral_x}, y: {integral_y}"
                #f"D -> x: {derivative_x}, y: {derivative_y}"
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
        
    node.destroy_node()
    rclpy.shutdown()
