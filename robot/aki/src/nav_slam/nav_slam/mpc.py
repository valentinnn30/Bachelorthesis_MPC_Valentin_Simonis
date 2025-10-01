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
# from slam_interface.action import ActivateSlam
import numpy as np
from collections import deque
import matplotlib.pyplot as plt

from std_msgs.msg import Float32MultiArray

from nav_slam.mpc_config import get_model, get_mpc

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math
from geometry_msgs.msg import Vector3
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Float32, Float64MultiArray, Int32
from rcl_interfaces.msg import SetParametersResult

class MPCControlNode(Node):
    def __init__(self):
        super().__init__('mpc_control_node')
        self.get_logger().info('MPC Control Node starting...')

        # Callback groups for concurrency (optional, but good practice)
        self.cb_group_path = ReentrantCallbackGroup()
        self.cb_group_depth = ReentrantCallbackGroup()
        self.cb_group_control = MutuallyExclusiveCallbackGroup() # For the main control loop

        # --- Subscriptions ---
        self.subscription = self.create_subscription(
            Path,
            '/camera_path',
            self.path_array_callback,
            10, # QoS history depth
            callback_group=self.cb_group_path
        )
        self.depth_subscriber = self.create_subscription(
            Float32,
            '/depth',
            self.depth_callback,
            10, # QoS history depth
            callback_group=self.cb_group_depth
        )

        # --- Publishers ---
        self.thruster_cmd_publisher = self.create_publisher(Float64MultiArray, '/thruster_commands', 10)
        self.bcu_manual_publisher = self.create_publisher(Int32, '/bcu/manual', 10)

        # --- State Variables from Subscriptions ---
        self.x = None
        self.y = None
        self.yaw = None
        self.depth = None
        self.previous_depth = None # To store previous depth for potential use
        self.previous_time = None # To store previous time for potential use
        self.time = None  # Initialize time for the latest depth reading
        self.z_velocity = 0.0  # Initialize z_velocity, can be updated in depth_callback

        self.previous_x = None
        self.previous_y = None
        self.previous_yaw = None
        self.previous_path_time = None # Keep a separate timestamp for path messages

        self.vx = 0.0  # Initialize velocities to 0
        self.vy = 0.0
        self.yaw_rate = 0.0 # Angular velocity around Z axis

        self.bcu_actual_pos = 75.0 # Initial BCU position, can be updated later

        self.declare_parameter('z_velocity_filter_window_size', 5) # Default to 5 samples
        self.z_velocity_filter_window_size = self.get_parameter('z_velocity_filter_window_size').value
        
        # Initialize the deque buffer for z_velocity samples
        self.z_velocity_buffer = deque(maxlen=self.z_velocity_filter_window_size)

        # --- Reference Goal (Fixed for now) ---
        # Define your target state (x, y, depth, yaw, etc.)
        # This will be used by the MPC to calculate control inputs
        self.reference_goal = {
            'x': 0.25,
            'y': -0.09,
            'depth': -2.0, # Example target depth
            'yaw': 0.0,
            'bcu_pos': 75.0 # Target BCU position
        }
        self.get_logger().info(f"Reference Goal: {self.reference_goal}")


        # --- MPC and Simulator Initialization ---
        self.model = get_model()
        self.mpc = get_mpc(self.model, self.reference_goal)
        # use below to get the model with BCU only
        # self.mpc = get_mpc_bcuonly(self.model, self.reference_goal)

        # Initial state for MPC and simulator (9 DoF assumed based on original code)
        # x, y, z, yaw, x_vel, y_vel, z_vel, vy, BCU_pos
        # Update later based on actual sensor data
        #self.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 75.0]) # Initial state including BCU
        self.current_x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 75.0]) # Initial state including BCU
        self.mpc.x0 = self.current_x0 # Set initial state for MPC
        #self.mpc.set_initial_guess()
        self.first_mpc_run = True


        # --- D matrix for numpy (used for f_xyz in NumPy context) ---
        theta_rad = np.deg2rad(20)
        azimuths = [0, 2*np.pi/3, 4*np.pi/3]
        dirs_np = []
        for a in azimuths:
            dir_x = np.cos(a) * np.cos(theta_rad)
            dir_y = np.sin(a) * np.cos(theta_rad)
            dir_z = np.sin(theta_rad)
            dirs_np.append([dir_x, dir_y, dir_z])
        dirs_np.append([0, 0, -1]) # Assuming this is for T4 (vertical thrust)
        self.D_mat_np = np.array(dirs_np).T

        # --- Logging (can be adapted for ROS 2 logging or data saving) ---
        self.trajectory = []
        self.u_T_log = []
        self.u_BCU_log = []
        self.u_force_log = []
        self.pos_bcu_log = []
        self.vel_bcu_log = []
        self.force_bcu_log = []

        # --- Control Loop Timer ---
        # This timer will trigger the MPC calculation and publishing
        self.control_timer = self.create_timer(0.2, self.control_loop_callback, callback_group=self.cb_group_control) # 5 Hz control loop

    # In your path_array_callback(self, msg):
    def path_array_callback(self, msg):
        if len(msg.poses) == 0:
            self.get_logger().warn('Received an empty Path message.')
            return

        newest_pose = msg.poses[-1]
        current_x = newest_pose.pose.position.x
        current_y = newest_pose.pose.position.y
        
        # Orientation (quaternion to Euler yaw)
        q = newest_pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        current_path_time = self.get_clock().now() # Get timestamp for this specific path message

        # Calculate velocities if previous data exists
        if self.previous_x is not None and self.previous_path_time is not None:
            dt_ns = current_path_time.nanoseconds - self.previous_path_time.nanoseconds
            dt_s = dt_ns * 1e-9 # Convert nanoseconds to seconds

            if dt_s > 0: # Avoid division by zero or negative time
                self.vx = (current_x - self.previous_x) / dt_s
                self.vy = (current_y - self.previous_y) / dt_s

                # For yaw rate, handle angle wrap-around (-pi to pi)
                yaw_diff = current_yaw - self.previous_yaw
                # Normalize angle difference to be within [-pi, pi]
                yaw_diff = (yaw_diff + np.pi) % (2 * np.pi) - np.pi
                self.yaw_rate = yaw_diff / dt_s
                
                # self.get_logger().info(f"Calculated VX: {self.vx:.2f}, VY: {self.vy:.2f}, Yaw Rate: {self.yaw_rate:.2f}")
            else:
                self.vx, self.vy, self.yaw_rate = 0.0, 0.0, 0.0 # No time elapsed, or invalid time
        else:
            # First data point for path, velocities are 0
            self.vx, self.vy, self.yaw_rate = 0.0, 0.0, 0.0

        # Update the node's current position/orientation (self.x, self.y, self.yaw)
        # and previous values for the next iteration
        self.x = current_x
        self.y = current_y
        self.yaw = current_yaw
        
        self.previous_x = current_x
        self.previous_y = current_y
        self.previous_yaw = current_yaw
        self.previous_path_time = current_path_time

    def depth_callback(self, msg):
        """
        Callback for the /depth topic.
        Calculates instantaneous z_velocity and applies a moving average filter.
        """
        current_depth = msg.data
        current_time = self.get_clock().now() # Get the timestamp of the current message

        # Only calculate velocity if we have a previous reading and timestamp
        if self.previous_depth is not None and self.previous_depth_time is not None:
            dt_ns = current_time.nanoseconds - self.previous_depth_time.nanoseconds
            dt_s = dt_ns * 1e-9 # Convert nanoseconds to seconds

            if dt_s > 0: # Ensure a valid time difference to avoid division by zero
                # Calculate instantaneous z_velocity
                # Assuming depth increases downwards, and positive z_velocity is UPWARDS
                instantaneous_z_velocity = -(current_depth - self.previous_depth) / dt_s
                
                # Add the instantaneous velocity to the buffer
                self.z_velocity_buffer.append(instantaneous_z_velocity)
                
                # Calculate the moving average and update self.vz (the filtered velocity)
                self.vz = np.mean(self.z_velocity_buffer)
                
                # Optional: Log the difference to see the effect of the filter
                # self.get_logger().info(f"Inst. Z-vel: {instantaneous_z_velocity:.3f} m/s, Filtered Z-vel: {self.vz:.3f} m/s")
            else:
                # If dt_s is 0 or negative (e.g., duplicate timestamp or error),
                # do not update velocity, keep the last filtered value.
                self.get_logger().warn("Zero or negative dt in depth_callback. Z-velocity not updated.")
        else:
            # This is the very first depth reading. Cannot calculate velocity yet.
            # Initialize filtered velocity to 0 and pre-fill the buffer with 0s
            self.vz = 0.0
            self.z_velocity_buffer.clear() # Ensure it's empty
            # Add initial zeros to the buffer to fill it up to maxlen for the first few samples
            for _ in range(self.z_velocity_filter_window_size):
                self.z_velocity_buffer.append(0.0)


        # Store current values to become 'previous' for the next iteration
        self.depth = current_depth
        self.previous_depth = current_depth
        self.previous_depth_time = current_time


    def control_loop_callback(self):
        # Ensure all sensor data is available before running MPC
        if self.x is None or self.y is None or self.yaw is None or self.depth is None:
            self.get_logger().warn("Waiting for all sensor data (x, y, yaw, depth)...")
            return

        # --- Update the MPC's current state (x0) with sensor data ---
        # THIS IS CRUCIAL: Map your sensor readings to your model's state vector (x0)
        # Adjust indices and values based on your actual model definition
        # Assuming x0 = [x, y, z(depth), roll, pitch, yaw, vx, vy, BCU_pos]
        """current_x0 = self.x0.copy() # Start with the last simulated state or a fresh one
        current_x0[0] = self.x
        current_x0[1] = self.y
        current_x0[2] = -self.depth # Assuming 3rd element is depth/z, negative for depth below surface
        current_x0[3] = self.yaw # 4th element is yaw
        current_x0[4] = self.vx # Assuming 5th element is x_velocity, can be estimated or left at 0 if not used by MPC
        current_x0[5] = self.vy # Assuming 6th element is
        current_x0[6] = self.z_velocity # velocity for z
        current_x0[7] = self.yaw_rate # Assuming 7th element is y_velocity, can be estimated or left at 0 if not used by MPC
        current_x0[8] = self.bcu_actual_pos # Assuming
        """

        """print("Types and shapes of inputs to current_x0:")
        for i, val in enumerate([
            self.x,
            self.y,
            -self.depth,
            self.yaw,
            self.vx,
            self.vy,
            self.z_velocity,
            self.yaw_rate,
            self.bcu_actual_pos
        ]):
            print(f"{i}: {val} (type: {type(val)}, shape: {getattr(val, 'shape', 'n/a')})")"""


        self.current_x0 = np.array([
            self.x,
            self.y,
            -self.depth, # Assuming z-position, negative for depth below surface
            self.yaw,
            self.vx,
            self.vy,
            self.z_velocity,
            self.yaw_rate,
            self.bcu_actual_pos
        ])

        # --- Initialize MPC for the very first time with actual sensor data ---
        if self.first_mpc_run:
            self.mpc.x0 = self.current_x0
            self.mpc.set_initial_guess() # Set initial guess for the solver based on the true initial state
            self.first_mpc_run = False # Set flag to false so this block only runs once
            self.get_logger().info(f"MPC initialized with actual starting state: {self.current_x0}")
        else:
            # For subsequent runs, simply update mpc.x0 with the latest sensor data
            # do-mpc will automatically warm-start the solver from the previous solution
            self.mpc.x0 = self.current_x0


        # --- MPC Step ---
        u0 = self.mpc.make_step(self.current_x0) # MPC calculates optimal control inputs
        
        # Ensure u0 is a numpy array
        if not isinstance(u0, np.ndarray):
            self.get_logger().error(f"MPC returned non-numpy array: {type(u0)}")
            return

        # Extract inputs and log them
        # Assuming u0 is [T1, T2, T3, T4, BCU_input] based on your original code
        if len(u0) != 5:
            self.get_logger().error(f"MPC output u0 has unexpected length: {len(u0)}. Expected 5 (4 thrusts + 1 BCU).")
            return

        T1234 = u0[:4].flatten() # T1, T2, T3, T4
        BCU_input = u0[4].flatten() # Single BCU input value

        bcu_pos = self.current_x0[8].flatten() # Current BCU position from state
        bcu_vel = self.current_x0[6].flatten() 
        force_bcu = (self.current_x0[8] - 75) * 0.0984 # real force that acts, not only desired input

        self.u_T_log.append(T1234)
        self.u_BCU_log.append(BCU_input)
        self.pos_bcu_log.append(bcu_pos)
        self.vel_bcu_log.append(bcu_vel)
        self.force_bcu_log.append(force_bcu)

        f_xyz = self.D_mat_np @ T1234.reshape((4, 1))
        self.u_force_log.append(f_xyz.flatten())

        # --- BCU position---
        try:
            # Assuming k_rate is defined as a parameter in your do-mpc model
            # and set as a nominal value within mpc.set_uncertainty_values()
            k_rate_val = self.mpc.set_uncertainty_values['k_rate']
        except Exception as e:
            self.get_logger().error(f"Could not retrieve 'k_rate' from MPC model parameters: {e}. Using default 0.25.")
            k_rate_val = 0.25 # Fallback to a hardcoded default if retrieval fails
        # The time step (dt) for integration
        dt = 0.2  # Use the MPC's control interval as your integration time step
        # hardcoded t_step of mpc_config
        # Calculate BCU_dot (rate of change of BCU position)
        # This uses the BCU_INPUT commanded by the MPC and the current estimated BCU position
        bcu_dot = k_rate_val * (BCU_input - self.bcu_actual_pos)

        # Integrate (Euler method) to get the new estimated BCU position
        # self.bcu_actual_pos += bcu_dot * dt
        self.bcu_actual_pos = float(self.bcu_actual_pos + bcu_dot * dt)
        # This is essential to keep the estimate within realistic bounds.
        self.bcu_actual_pos = np.clip(self.bcu_actual_pos, 75.0, 100.0)

        # --- Publish Thruster Commands ---
        thruster_msg = Float64MultiArray()
        Thruster_downscaled = np.array(T1234)
        Thruster_downscaled = Thruster_downscaled/5.58
        Thruster_downscaled_clip = np.clip(Thruster_downscaled, 0.0, 1.0)
        thruster_msg.data = Thruster_downscaled_clip.tolist() # Convert numpy array to list
        self.thruster_cmd_publisher.publish(thruster_msg)
        self.get_logger().info(f"Published Thruster Commands: {thruster_msg.data}")

        # --- Publish BCU Manual Input ---
        bcu_msg = Int32()
        bcu_msg.data = round(int(BCU_input)) # BCU input typically an integer
        self.bcu_manual_publisher.publish(bcu_msg)
        self.get_logger().info(f"Published BCU Manual Input: {bcu_msg.data}")

        # --- Simulator Step ---
        self.trajectory.append(self.current_x0[:4].flatten())  # Store eta only (first 4 DoF)

        # You can add more complex logging or visualization here if needed
        # For example, publishing a MarkerArray for trajectory visualization in RViz
        # or logging the full state vector.

    def plot_results(self):
        """
        Generates and displays plots of the robot's trajectory,
        MPC inputs, and resulting forces.
        This method should be called after the control loop has finished.
        """
        self.get_logger().info("Generating plots...")

        # Convert logged lists to NumPy arrays for plotting
        try:
            trajectory = np.array(self.trajectory)
            u_T_log = np.array(self.u_T_log)
            u_BCU_log = np.array(self.u_BCU_log).flatten()
            pos_bcu_log = np.array(self.pos_bcu_log).flatten()
            u_force_log = np.array(self.u_force_log)
            vel_bcu_log = np.array(self.vel_bcu_log).flatten()
            force_bcu_log = np.array(self.force_bcu_log).flatten()
            # f_global_log will be calculated below, no need to convert here
        except Exception as e:
            self.get_logger().error(f"Error converting logged data to NumPy arrays for plotting: {e}")
            self.get_logger().warn("Skipping plots due to data conversion error.")
            return

        # Ensure there's enough data to plot
        if len(trajectory) == 0 or len(u_force_log) == 0:
            self.get_logger().warn("Not enough data to plot results.")
            return

        # --- Calculate global force (f_global_log) using your provided code ---
        # Assuming yaw is at index 5 of the trajectory's state (x,y,z,yaw)
        yaw_angles_rad = trajectory[:, 3]

        f_global_log_calculated = [] # Temporary list for collection

        for i in range(len(u_force_log)):
            fx_body = u_force_log[i, 0]
            fy_body = u_force_log[i, 1]
            fz_body = u_force_log[i, 2]

            psi = yaw_angles_rad[i]

            # Rotation from body to global frame for X and Y components
            fx_global = fx_body * np.cos(psi) - fy_body * np.sin(psi)
            fy_global = fx_body * np.sin(psi) + fy_body * np.cos(psi)
            fz_global = fz_body # Z-force is unchanged by yaw rotation (assuming no roll/pitch)

            f_global_log_calculated.append([fx_global, fy_global, fz_global])

        f_global_log = np.array(f_global_log_calculated) # Convert to NumPy array for plotting


        # --- Now, proceed with your plotting code, using the calculated f_global_log ---

        # Plot 1: Position and Orientation (eta)
        fig, axs = plt.subplots(3, 1, figsize=(10, 12))
        axs[0].plot(trajectory[:, 0], label='x')
        axs[0].plot(trajectory[:, 1], label='y')
        axs[0].plot(trajectory[:, 2], label='z')
        # ... (target lines for x, y, z from self.reference_goal) ...
        axs[0].axhline(y=self.reference_goal['x'], color='r', linestyle=':', label=f'Target x: {self.reference_goal["x"]}')
        axs[0].axhline(y=self.reference_goal['y'], color='g', linestyle=':', label=f'Target y: {self.reference_goal["y"]}')
        axs[0].axhline(y=self.reference_goal['depth'], color='b', linestyle=':', label=f'Target z: {self.reference_goal["depth"]}')
        axs[0].legend()
        axs[0].set_title('Position [m]')
        axs[0].set_ylabel('Position [m]')
        axs[0].set_xlabel('Time Step')
        axs[0].grid(True)

        axs[1].plot(trajectory[:, 5], label='yaw')
        axs[1].axhline(y=self.reference_goal['yaw'], color='m', linestyle=':', label=f'Target Yaw: {self.reference_goal["yaw"]:.2f}')
        axs[1].legend()
        axs[1].set_title('Orientation (Yaw) [rad]')
        axs[1].set_ylabel('Angle [rad]')
        axs[1].set_xlabel('Time Step')
        axs[1].grid(True)

        ref_pos_array = np.array([self.reference_goal['x'], self.reference_goal['y'], self.reference_goal['depth']])
        axs[2].plot(np.linalg.norm(trajectory[:, :3] - ref_pos_array, axis=1))
        axs[2].set_title('Position Error Norm (||Pos - TargetPos||)')
        axs[2].set_ylabel('Error Norm [m]')
        axs[2].set_xlabel('Time Step')
        axs[2].grid(True)
        plt.tight_layout()

        # Plot 2: MPC Commanded Thrusts
        plt.figure(figsize=(10, 6))
        plt.plot(u_T_log[:, 0], label='T1')
        plt.plot(u_T_log[:, 1], label='T2')
        plt.plot(u_T_log[:, 2], label='T3')
        plt.plot(u_T_log[:, 3], label='T4')
        plt.xlabel('Time Step')
        plt.ylabel('Thrust [N]')
        plt.title('MPC Commanded Thrusts (Raw MPC Output)')
        plt.legend()
        plt.grid(True)

        # Plot 3: MPC Commanded BCU Input and Actual BCU Position
        plt.figure(figsize=(10, 6))
        plt.plot(u_BCU_log, label='BCU_INPUT (MPC Command)', color='tab:blue')
        plt.plot(pos_bcu_log, label='BCU_position (Actual State)', color='tab:red')
        min_bcu_input = 50
        max_bcu_input = 100
        plt.axhline(y=min_bcu_input, color='darkred', linestyle='--', label=f'Min BCU Input: {min_bcu_input:.0f}')
        plt.axhline(y=max_bcu_input, color='darkgreen', linestyle='--', label=f'Max BCU Input: {max_bcu_input:.0f}')
        plt.axhline(y=self.reference_goal['bcu_pos'], color='purple', linestyle=':', label=f'Target BCU Pos: {self.reference_goal["bcu_pos"]:.0f}')
        plt.xlabel('Time Step')
        plt.ylabel('BCU Value')
        plt.title('MPC Commanded BCU Input vs. Actual BCU Position')
        plt.legend()
        plt.grid(True)

        # Plot 4: Resultant Forces in Body Frame
        plt.figure(figsize=(10, 6))
        plt.plot(u_force_log[:, 0], label='Fx (Body)')
        plt.plot(u_force_log[:, 1], label='Fy (Body)')
        plt.plot(u_force_log[:, 2], label='Fz (Body)')
        plt.xlabel('Time Step')
        plt.ylabel('Force [N]')
        plt.title('Resultant Forces from Thrusters (Body Frame)')
        plt.legend()
        plt.grid(True)

        # --- Plot 5: Resultant Forces in Global Frame (using the newly calculated f_global_log) ---
        plt.figure(figsize=(10, 6))
        plt.plot(f_global_log[:, 0], label='Fx (Global)')
        plt.plot(f_global_log[:, 1], label='Fy (Global)')
        plt.plot(f_global_log[:, 2], label='Fz (Global)')
        plt.xlabel('Time Step')
        plt.ylabel('Force [N]')
        plt.title('Resultant Forces from Thrusters (Global Frame)')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()

        # Plot 6: Velocity in vertical direction
        plt.figure(figsize=(10, 6))
        plt.plot(vel_bcu_log, label='Vertical Velocity (BCU related)')
        plt.xlabel('Time Step')
        plt.ylabel('Velocity [m/s]')
        plt.title('Velocity in Vertical Direction')
        plt.legend()
        plt.grid(True)

        # Plot 7: Vertical force due to BCU
        plt.figure(figsize=(10, 6))
        plt.plot(force_bcu_log, label='BCU Force (Fz_x)')
        plt.xlabel('Time Step')
        plt.ylabel('Force [N]')
        plt.title('Vertical Force due to Buoyancy Control Unit (BCU)')
        plt.legend()
        plt.grid(True)

        plt.show() # Display all generated figures

def main(args=None):
    rclpy.init(args=args)
    node = MPCControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("MPC Control Node stopped by user.")
    finally:
        # Save logs if desired (e.g., to a CSV file)
        # np.savetxt('trajectory.csv', node.trajectory, delimiter=',')
        # np.savetxt('u_T_log.csv', node.u_T_log, delimiter=',')
        # folgendes auskommentieren um zu plotten
        #node.plot_results()
        #print("Plots generated. Close the plot windows to terminate program.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()