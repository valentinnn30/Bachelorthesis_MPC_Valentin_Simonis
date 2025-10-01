#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import SetParametersResult

# ROS 2 message types
from std_msgs.msg import Float64, Float32, Empty, Header, Int32
from geometry_msgs.msg import Vector3, Pose, Twist
from std_msgs.msg import Float64MultiArray # For thruster commands

# do-mpc and CasADi imports
from casadi import *
import numpy as np
import casadi as ca
import do_mpc
import math
import cv2

# Import your do_mpc model and controller setup functions
# Ensure mpc_config.py is in your package's root directory (your_robot_mpc_controller/mpc_config.py)
from your_robot_mpc_controller.mpc_config import get_model, get_mpc

# Action message type (assuming you copied nav2_msgs.action.NavigateToPose)
from nav2_msgs.action import NavigateToPose # Or define your own custom action if this doesn't fully fit


class NavYoloMPC(Node):

    def __init__(self):
        super().__init__('nav_yolo_mpc_server')
        self.get_logger().info('NavYoloMPC Node has been started.')

        self.cb_group_state_feedback = ReentrantCallbackGroup() # For sensor callbacks
        self.cb_group_action = ReentrantCallbackGroup() # For action server callbacks

        # --- MPC Initialization ---
        self.model = get_model()
        self.mpc = get_mpc(self.model)

        # Declare ROS 2 parameters for MPC tuning and model parameters
        self._declare_mpc_parameters()

        # Initialize do-mpc parameters using ROS parameters
        self._set_mpc_parameters_from_ros() # Call this initially to setup MPC

        # Initial state for MPC: Match the state vector order of your do-mpc model
        # eta[x,y,z,psi], nu[u,v,w,r], BCU_position
        self.current_x0 = np.array([
            0.0, 0.0, 0.0, 0.0,  # x, y, z, psi (Global position and yaw)
            0.0, 0.0, 0.0, 0.0,  # u, v, w, r (Body-fixed velocities)
            75                 # BCU_position (initial neutral)
        ])
        self.mpc.x0 = self.current_x0
        self.mpc.set_initial_guess()

        # Reference values for MPC (will be updated based on mission logic)
        self.mpc_ref_eta = ca.DM([0, 0, 0, 0]).reshape((4, 1)) # Global X, Y, Z, Yaw
        self.mpc_ref_nu = ca.DM([0, 0, 0, 0]).reshape((4, 1)) # Body U, V, W, R

        # --- ROS 2 Subscribers for State Feedback ---
        # Position feedback (from e.g., localization node, providing global Pose)
        # BCU actual position feedback -> need to fix
        self.bcu_pos_sub = self.create_subscription(
            Int32, '/bcu_position_feedback', self._bcu_position_callback, 10,
            callback_group=self.cb_group_state_feedback
        )
        # Depth sensor feedback
        self.depth_subscriber = self.create_subscription(
            Float32, '/depth', self._depth_callback, 10,
            callback_group=self.cb_group_state_feedback
        )
        # Yolo table detection (pixel coordinates)
        self.detection_subscription = self.create_subscription(
            Vector3, '/tracked/table', self._detection_callback, 1,
            callback_group=self.cb_group_state_feedback
        )


        # --- ROS 2 Publishers for Control Commands ---
        self.thruster_cmd_publisher = self.create_publisher(Float64MultiArray, '/thruster_commands', 10)
        self.bcu_cmd_publisher = self.create_publisher(Int32, '/bcu/manual', 10) # BCU desired position

        # --- Action server for GoTable action (kept from original node) ---
        self._action_server = ActionServer(
            self,
            NavigateToPose, # Consider renaming/defining a custom action for clarity
            'yolo_nav_mpc',
            self.execute_callback,
            callback_group=self.cb_group_action,
        )

        # --- Mission State Variables (from your original logic) ---
        self.depth = 0.0 # Current depth from sensor

        self.current_yolo_x_px = 0.0 # Yolo detected X pixel
        self.current_yolo_y_px = 0.0 # Yolo detected Y pixel

        self.in_yolo_threshold = False
        self.yolo_threshold_start_time = 0.0

        self.depth_nav_active = False # Flag for vertical navigation phase
        self.target_global_depth = 0.0 # Target depth to send to MPC

        self.depth_reached_threshold = 0.05 # Tolerance for depth
        self.depth_is_reached = False # Flag if global depth is reached

        # Parameters for your mission logic (adapted from original PID node)
        self.declare_parameter('test_mode', False)
        self.test_mode = self.get_parameter('test_mode').get_parameter_value().bool_value

        self.declare_parameter('camera_middle_x_px', 320.0) # Image center X
        self.declare_parameter('camera_middle_y_px', 240.0) # Image center Y
        self.declare_parameter('yolo_threshold_x_px', 50.0) # Pixel tolerance for Yolo X centering
        self.declare_parameter('yolo_threshold_y_px', 50.0) # Pixel tolerance for Yolo Y centering

        self.declare_parameter('surface_distance_above_table_m', 1.8) # Desired distance to surface above table
        self.declare_parameter('surface_distance_above_ground_m', 3.0) # Desired distance to surface above ground (for target_heightground)
        self.declare_parameter('pool_floor_depth_m', 4.0) # Hardcoded pool floor depth

        self.camera_middle_x_px = self.get_parameter('camera_middle_x_px').get_parameter_value().double_value
        self.camera_middle_y_px = self.get_parameter('camera_middle_y_px').get_parameter_value().double_value
        self.yolo_threshold_x_px = self.get_parameter('yolo_threshold_x_px').get_parameter_value().double_value
        self.yolo_threshold_y_px = self.get_parameter('yolo_threshold_y_px').get_parameter_value().double_value
        self.surface_distance_above_table_m = self.get_parameter('surface_distance_above_table_m').get_parameter_value().double_value
        self.surface_distance_above_ground_m = self.get_parameter('surface_distance_above_ground_m').get_parameter_value().double_value
        self.pool_floor_depth_m = self.get_parameter('pool_floor_depth_m').get_parameter_value().double_value

        self.get_logger().info(f'NavYoloMPC: Initialized with Camera Center X={self.camera_middle_x_px}, Y={self.camera_middle_y_px}')

        # Add parameter update callback (for dynamic parameter changes)
        self.add_on_set_parameters_callback(self._parameter_update_callback)

        # Internal flag to signal if the action is currently active
        self._action_active = False


    def _declare_mpc_parameters(self):
        # MPC specific parameters
        self.declare_parameter('mpc.n_horizon', 20)
        self.declare_parameter('mpc.t_step', 0.2) # 0.2 seconds, also 5 Hz control frequency
        self.declare_parameter('mpc.state_discretization', 'collocation')
        self.declare_parameter('mpc.collocation_type', 'radau')
        self.declare_parameter('mpc.collocation_deg', 3)
        self.declare_parameter('mpc.collocation_ni', 2)
        self.declare_parameter('mpc.store_full_solution', True) # Store full solution for debugging
        self.declare_parameter('mpc.n_robust', 1)
        self.declare_parameter('mpc.z_force_penalty_weight', 50.0)
        self.declare_parameter('mpc.thruster_use_penalty_weight', 0.01)
        self.declare_parameter('mpc.w_nu_z', 5) # Z-velocity penalty
        self.declare_parameter('mpc.target_depth_initial', 1.0) # Initial target depth for MPC reference

        # Model parameters (mirroring your do-mpc model.set_parameter calls)
        self.declare_parameter('model.mass', 22.5)
        self.declare_parameter('model.izz', 0.335)
        self.declare_parameter('model.g_accel', 9.81)
        self.declare_parameter('model.rho_water', 1025.0)
        self.declare_parameter('model.v_fixed_buoyancy', 0.0174)
        self.declare_parameter('model.x_udot', -18.0)
        self.declare_parameter('model.y_vdot', -18.0)
        self.declare_parameter('model.z_wdot', -25.0)
        self.declare_parameter('model.n_rdot', -0.5)
        self.declare_parameter('model.d_lin_0', 2.0)
        self.declare_parameter('model.d_lin_1', 2.0)
        self.declare_parameter('model.d_lin_2', 3.0)
        self.declare_parameter('model.d_lin_3', 0.1)
        self.declare_parameter('model.d_quad_0', 13.0)
        self.declare_parameter('model.d_quad_1', 13.0)
        self.declare_parameter('model.d_quad_2', 28.0)
        self.declare_parameter('model.d_quad_3', 0.1)
        self.declare_parameter('model.k_rate', 0.25)
        self.declare_parameter('model.bcu_force_factor', 0.0984)

        # MPC Bounds
        self.declare_parameter('mpc.max_thruster_force', 5.58)
        self.declare_parameter('mpc.min_bcu_input', 50.0)
        self.declare_parameter('mpc.max_bcu_input', 100.0)


    def _set_mpc_parameters_from_ros(self):
        # Retrieve MPC setup parameters
        n_horizon = self.get_parameter('mpc.n_horizon').value
        t_step = self.get_parameter('mpc.t_step').value
        state_discretization = self.get_parameter('mpc.state_discretization').value
        collocation_type = self.get_parameter('mpc.collocation_type').value
        collocation_deg = self.get_parameter('mpc.collocation_deg').value
        collocation_ni = self.get_parameter('mpc.collocation_ni').value
        store_full_solution = self.get_parameter('mpc.store_full_solution').value
        n_robust = self.get_parameter('mpc.n_robust').value
        self.mpc.set_param(n_horizon=n_horizon, t_step=t_step, state_discretization=state_discretization, collocation_type=collocation_type, collocation_deg=collocation_deg, collocation_ni=collocation_ni, store_full_solution=store_full_solution, n_robust=n_robust)

        # Update model parameters for MPC's internal model
        p_template = self.model.get_p_template()
        p_template['mass'] = self.get_parameter('model.mass').value
        p_template['Izz'] = self.get_parameter('model.izz').value
        p_template['g'] = self.get_parameter('model.g_accel').value
        p_template['rho_water'] = self.get_parameter('model.rho_water').value
        p_template['V_fixed_buoyancy'] = self.get_parameter('model.v_fixed_buoyancy').value
        p_template['X_udot'] = self.get_parameter('model.x_udot').value
        p_template['Y_vdot'] = self.get_parameter('model.y_vdot').value
        p_template['Z_wdot'] = self.get_parameter('model.z_wdot').value
        p_template['N_rdot'] = self.get_parameter('model.n_rdot').value
        p_template['d_lin_0'] = self.get_parameter('model.d_lin_0').value
        p_template['d_lin_1'] = self.get_parameter('model.d_lin_1').value
        p_template['d_lin_2'] = self.get_parameter('model.d_lin_2').value
        p_template['d_lin_3'] = self.get_parameter('model.d_lin_3').value
        p_template['d_quad_0'] = self.get_parameter('model.d_quad_0').value
        p_template['d_quad_1'] = self.get_parameter('model.d_quad_1').value
        p_template['d_quad_2'] = self.get_parameter('model.d_quad_2').value
        p_template['d_quad_3'] = self.get_parameter('model.d_quad_3').value
        p_template['k_rate'] = self.get_parameter('model.k_rate').value
        p_template['bcu_force_factor'] = self.get_parameter('model.bcu_force_factor').value
        p_template['d_ext'] = ca.DM([0,0,0,0]).reshape((4,1)) # Nominal disturbance for MPC's internal model
        self.mpc.set_p_fun(lambda_p=lambda: p_template) # MPC uses these parameters

        # Update MPC objective weights (this requires re-setting the objective)
        z_force_penalty_weight = self.get_parameter('mpc.z_force_penalty_weight').value
        thruster_use_penalty_weight = self.get_parameter('mpc.thruster_use_penalty_weight').value
        w_nu_z = self.get_parameter('mpc.w_nu_z').value

        # Reconstruct objective: assumes get_mpc defines the objective with dummy references initially
        # and then set_r updates them dynamically.
        # This part might need adjustment if your get_mpc doesn't allow dynamic weight updates easily
        # A common way is to make the weights part of the model's parameters and use them in the objective
        # (model.set_parameter('W_NU_Z') and then mpc.objective = ... model.p['W_NU_Z'] * nu[2]**2)
        # For simplicity, if get_mpc recreates the objective each time it's called, you could do:
        # self.mpc = get_mpc(self.model, z_force_penalty_weight, thruster_use_penalty_weight, w_nu_z)
        # For now, let's assume the parameters are passed to get_mpc() or the default values in mpc_config are used initially.
        # If your get_mpc() sets objectives with parameters directly like model.p['my_weight_param'], then
        # updating p_template will update the objective automatically.

        # Update MPC Bounds
        self.mpc.bounds['lower','_u','T'] = 0.0
        self.mpc.bounds['upper','_u','T'] = self.get_parameter('mpc.max_thruster_force').value
        self.mpc.bounds['lower','_u','BCU_INPUT'] = self.get_parameter('mpc.min_bcu_input').value
        self.mpc.bounds['upper','_u','BCU_INPUT'] = self.get_parameter('mpc.max_bcu_input').value

        self.mpc.setup() # Re-setup MPC after parameter changes

        # Reinitialize timer if t_step changes
        if hasattr(self, 'timer'):
            self.timer.destroy()
        self.timer = self.create_timer(t_step, self._mpc_timer_callback)
        self.get_logger().info(f'MPC re-setup with N_horizon: {n_horizon}, T_step: {t_step}')

    def _parameter_update_callback(self, params):
        # This callback is triggered when ROS parameters are set dynamically
        if self._action_active: # Prevent changing critical MPC params while active
            self.get_logger().warn("Attempted to change MPC parameters while action is active. Changes might not take full effect until action completes.")
            # For this simple example, we'll still apply them but warn the user.
            # In a real system, you might reject the parameter change or queue it.
        self._set_mpc_parameters_from_ros() # Re-apply all parameters
        for param in params: # Log only changed parameters for user
            self.get_logger().info(f"Updated ROS parameter: {param.name} to {param.value}")
        return SetParametersResult(successful=True)


    # --- State Feedback Callbacks ---x
    def _bcu_position_callback(self, msg: Int32):
        # Update current_x0 with latest BCU actual position feedback
        self.current_x0[8] = msg.data

    def _depth_callback(self, msg: Float32):
        self.depth = msg.data
        self.current_x0[8] = -self.depth # Update current_x0 with latest depth feedback, negative for depth below surface

    def _detection_callback(self, detection: Vector3):
        # Update current Yolo pixel coordinates
        self.current_yolo_x_px = detection.x
        self.current_yolo_y_px = detection.y

        self.yolo_pixel = [detection.x, detection.y]

        # Camera intrinsics (fisheye model)
        fx = 573.26
        fy = 564.92
        cx = 389.47
        cy = 240.04

        camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0,  0,  1]
        ])
        dist_coeffs = np.array([-0.098, 0.388, -0.768, 0.656])  # fisheye distortion

        u = detection.x
        v = detection.y

        # Use depth value: Z = 3.5 - depth
        if math.isnan(self.latest_depth):
            self.table_position = np.array([float('nan')] * 3)
            return

        # Undistort to normalized image coordinates (no P!)
        distorted = np.array([[[u, v]]], dtype=np.float32)
        undistorted = cv2.fisheye.undistortPoints(distorted, camera_matrix, dist_coeffs)

        x_norm, y_norm = undistorted[0, 0]

        # Direction vector from camera center
        direction = np.array([x_norm, y_norm, 1.0])
        direction /= np.linalg.norm(direction)

        # Project to 3D
        position_3d = direction * Z

        # Store 3D table position in camera frame
        self.table_position = position_3d
        # No logging here, as it happens in the main MPC loop if needed for debug


    def _mpc_timer_callback(self):
        """
        This timer callback runs at MPC's t_step.
        It updates MPC's current state and reference, then performs one MPC step.
        """
        if not self._action_active: # Only run MPC if an action is active
            return

        # Always update MPC's state with the latest feedback
        self.mpc.x0 = self.current_x0

        # --- MPC Reference Setting based on mission phase ---
        # Initialize references to maintain current position/velocity (stop/hover)
        target_global_x = self.current_x0[0]
        target_global_y = self.current_x0[1]
        target_global_yaw = self.current_x0[3] # Maintain current yaw (straight path)
        target_linear_x_vel = 0.0
        target_linear_y_vel = 0.0
        target_linear_z_vel = 0.0
        target_yaw_rate = 0.0

        x_error_px = self.camera_middle_x_px - self.current_yolo_x_px
        y_error_px = self.camera_middle_y_px - self.current_yolo_y_px

        # --- Phase 1: Center on Yolo Target (X, Y) ---
        if not self.depth_nav_active:
            # Check if within Yolo pixel thresholds
            if abs(x_error_px) < self.yolo_threshold_x_px and \
               abs(y_error_px) < self.yolo_threshold_y_px:
                if not self.in_yolo_threshold:
                    self.in_yolo_threshold = True
                    self.yolo_threshold_start_time = self.get_clock().now().nanoseconds / 1e9
                    self.get_logger().info('Entered Yolo centering threshold. Waiting for 5s.')
                elif (self.get_clock().now().nanoseconds / 1e9) - self.yolo_threshold_start_time >= 5.0:
                    self.get_logger().info('Yolo centering successful for 5 seconds. Initiating depth navigation.')
                    self.in_yolo_threshold = False
                    self.depth_nav_active = True # Move to next phase

                    
                    # Fallback for ground if no ultrasonic data (e.g., if finding coral on ground)
                    self.target_global_depth = self.pool_floor_depth_m - self.surface_distance_above_ground_m
                    self.get_logger().info(f'Target global depth to: {self.target_global_depth:.2f} m (above ground)')

            else: # Not within thresholds, so actively center using Yolo errors
                self.in_yolo_threshold = False
                # The MPC will drive x,y to 0 if their references are 0 and they are penalized.
                # However, Yolo provides pixel errors. We need to translate pixel errors to global velocities/forces.
                # Simplest for MPC: set target global X,Y to current, and let the objective's position penalty handle it.
                # Or, if you want direct velocity commands in X/Y based on Yolo errors:
                # This needs to be carefully tuned. A rough mapping:
                # target_linear_x_vel = - (x_error_px / 100.0) * 0.1 # Example: scale pixel error to m/s
                # target_linear_y_vel = - (y_error_px / 100.0) * 0.1 # Example: scale pixel error to m/s
                # For initial setup, let MPC stabilize current X,Y and rely on external planner for movement
                # Or, more directly, command surge/sway velocities to drive pixel errors to zero.
                # THIS IS THE PART WHERE YOU ADAPT YOLOV VELOCITY COMMANDING TO MPC
                # For example, to move forward if x_error is negative, you'd want a positive surge (u) velocity:
                
                # Simple P-control for target velocity from pixel error
                P_gain_yolo_x = 0.0005 # Tune this
                P_gain_yolo_y = 0.0005 # Tune this

                target_linear_y_vel = x_error_px * P_gain_yolo_x # X pixel error drives sway (y-velocity)
                target_linear_x_vel = y_error_px * P_gain_yolo_y # Y pixel error drives surge (x-velocity)

                # Clamp max velocity commands
                max_xy_vel = 0.1 # m/s
                target_linear_x_vel = np.clip(target_linear_x_vel, -max_xy_vel, max_xy_vel)
                target_linear_y_vel = np.clip(target_linear_y_vel, -max_xy_vel, max_xy_vel)

                self.get_logger().info(f'Yolo X error: {x_error_px:.1f}, Y error: {y_error_px:.1f} -> Target U: {target_linear_x_vel:.2f}, V: {target_linear_y_vel:.2f}')


        # --- Phase 2: Depth Navigation ---
        elif self.depth_nav_active:
            # Set target global depth for MPC
            target_global_depth = self.target_global_depth
            self.depth_is_reached = abs(self.depth - self.target_global_depth) <= self.depth_reached_threshold

            if self.depth_is_reached:
                self.get_logger().info('Target depth reached within threshold.')
                # Now try to hold depth, and stop horizontal motion (set ref vel to 0)
                target_linear_z_vel = 0.0 # Maintain depth
                target_linear_x_vel = 0.0
                target_linear_y_vel = 0.0
            else:
                # Command vertical velocity to reach target depth
                P_gain_depth = 0.05 # Tune this
                target_linear_z_vel = (self.target_global_depth - self.depth) * P_gain_depth
                max_z_vel = 0.05 # m/s
                target_linear_z_vel = np.clip(target_linear_z_vel, -max_z_vel, max_z_vel)
                self.get_logger().info(f'Navigating to depth {self.target_global_depth:.2f} m. Current: {self.depth:.2f} m. Target W: {target_linear_z_vel:.2f}')

        # --- Final Mission Success Check ---
        if self.in_yolo_threshold and self.depth_nav_active and self.depth_is_reached:
            self.get_logger().info('Mission successful: Yolo centered and target depth reached.')
            # Stop all motion
            target_linear_x_vel = 0.0
            target_linear_y_vel = 0.0
            target_linear_z_vel = 0.0
            target_yaw_rate = 0.0
            # Signal action success
            self._action_active = False # Stop MPC from running until next action goal
            self.get_logger().info('Stopping all motion and completing action.')
            self._action_server.send_goal_response() # Placeholder, correct method to succeed action
            self._goal_handle.succeed() # Assuming _goal_handle is stored
            return # Exit timer callback

        # --- Construct MPC References ---
        self.mpc_ref_eta = ca.DM([target_global_x, target_global_y, target_global_depth, target_global_yaw]).reshape((4, 1))
        self.mpc_ref_nu = ca.DM([target_linear_x_vel, target_linear_y_vel, target_linear_z_vel, target_yaw_rate]).reshape((4, 1))

        self.mpc.set_r(mpc_ref_eta=self.mpc_ref_eta, mpc_ref_nu=self.mpc_ref_nu)

        # --- Perform MPC Step and Publish Commands ---
        try:
            u0 = self.mpc.make_step(self.current_x0)

            # Publish Thruster Commands
            thruster_cmd_msg = Float64MultiArray()
            # Ensure u0's structure matches your model's input structure [T1,T2,T3,T4, BCU_INPUT]
            thruster_cmd_msg.data = u0[:4].full().flatten().tolist()
            self.thruster_cmd_publisher.publish(thruster_cmd_msg)

            # Publish BCU Command
            bcu_cmd_msg = Float64()
            bcu_cmd_msg.data = float(u0[4].full().flatten()[0]) # The BCU_INPUT from MPC
            self.bcu_cmd_publisher.publish(bcu_cmd_msg)

            # Log commands (for debugging)
            # self.get_logger().info(
            #     f"MPC Commands -> Thrusters: {thruster_cmd_msg.data}, BCU: {bcu_cmd_msg.data}"
            # )

        except Exception as e:
            self.get_logger().error(f"MPC calculation failed: {e}")
            # In case of MPC failure, send zero commands or safe state
            self._publish_zero_commands()


    def _publish_zero_commands(self):
        thruster_cmd_msg = Float64MultiArray()
        thruster_cmd_msg.data = [0.0, 0.0, 0.0, 0.0]
        self.thruster_cmd_publisher.publish(thruster_cmd_msg)

        bcu_cmd_msg = Float64()
        bcu_cmd_msg.data = 75.0 # Neutral BCU
        self.bcu_cmd_publisher.publish(bcu_cmd_msg)
        self.get_logger().warn("Published zero commands due to MPC error or action completion.")


    # --- Action Server Callback ---
    def execute_callback(self, goal_handle):
        self.get_logger().info('Received NavYoloMPC action goal.')
        self._goal_handle = goal_handle # Store goal handle to succeed/abort later
        self._action_active = True # Activate MPC timer

        # Reset mission state flags at the start of a new action
        self.in_yolo_threshold = False
        self.yolo_threshold_start_time = 0.0
        self.depth_nav_active = False
        self.depth_is_reached = False
        self.ultrasonic_distance.clear() # Clear old ultrasonic readings

        # The actual control is handled by the _mpc_timer_callback.
        # This execute_callback primarily sets up the action state and then waits.
        # The _mpc_timer_callback will set _action_active to False and call goal_handle.succeed()
        # or goal_handle.abort() when the mission phase completes or fails.

        result = NavigateToPose.Result() # Assuming Empty result as per your original node

        # Keep this loop to check for cancel requests
        while rclpy.ok() and self._action_active:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Action canceled by client.')
                self._action_active = False
                self._publish_zero_commands()
                goal_handle.canceled()
                result.result = Empty()
                return result
            #rclpy.spin_once(self, timeout_sec=0.1) # Spin here to process callbacks
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1)) # Use rclpy sleep

        # If action becomes inactive (succeeded or aborted by timer callback)
        if not self._action_active:
            # If the goal was succeeded by the timer callback, this execute_callback returns here
            # and the `succeed()` or `abort()` method would have already been called on `goal_handle`.
            self.get_logger().info('Action loop finished.')
            # No need to call goal_handle.succeed() or abort() here again as it's done in timer
            # Just return the result
            return result
        else: # Should not be reached if _action_active is correctly managed
            self.get_logger().error('Action loop exited unexpectedly.')
            self._publish_zero_commands()
            goal_handle.abort()
            result.result = Empty()
            return result


def main(args=None):
    rclpy.init(args=args)
    node = NavYoloMPC()
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