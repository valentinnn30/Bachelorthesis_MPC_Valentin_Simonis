import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
import numpy as np
from scipy.optimize import nnls
from scipy.optimize import minimize
import serial


class Commander(Node):
    def __init__(self):
        super().__init__('commander')
        
        # Subscribe to IMU data
        self.subscription = self.create_subscription(Imu, '/imu/data', self.listener_callback, 10)
        
        # Subscribe to the "power" topic to get desired force vector
        self.force_subscription = self.create_subscription(Vector3, 'power', self.force_callback, 10)
        
        # Subscribe to the "torque" topic to get desired torque vector
        self.torque_subscription = self.create_subscription(Vector3, 'torque', self.torque_callback, 10)
        
        # Buoyancy publishers for each BCU
        self.t1pub = self.create_publisher(Float64, '/model/tethys/joint/umdrehung_1/cmd_thrust', 1)
        self.t2pub = self.create_publisher(Float64, '/model/tethys/joint/umdrehung_2/cmd_thrust', 1)
        self.t3pub = self.create_publisher(Float64, '/model/tethys/joint/umdrehung_3/cmd_thrust', 1)
        self.t4pub = self.create_publisher(Float64, '/model/tethys/joint/umdrehung_4/cmd_thrust', 1)
        self.pid_log = self.create_publisher(Float64, '/logpid', 10)
        
        # Combined force and torque matrix
        self.A = np.array([
            [0.43, 0.43, -0.87, 0.0],          # Force in x-direction
            [-0.75, 0.75, 0.0, 0.0],           # Force in y-direction
            [0.5, 0.5, 0.5, -1.0],             # Force in z-direction
  
        ])


        """[-0.106, 0.1055, 0.008, 0.03],     # Torque in x-direction
            [-0.04238, -0.06032, 0.11964, -0.053],  # Torque in y-direction
            [0.02759, -0.00025, 0.01392, 0.0]  # Torque in z-direction
                      [0.225, -0.225, 0.0, 0.0],     # Torque in x-direction
            [0.13, 0.13, -0.26, 0.0],  # Torque in y-direction
            [0.0, 0.0, 0.0, 0.0]  # Torque in z-direction"""
        
        # Initialize desired force-torque vector
        self.FT_desired = np.zeros(3)

    def listener_callback(self, msg):
        # Read the quaternion orientation from the IMU message
        ox, oy, oz, ow = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        
        # Convert quaternion to roll, pitch, yaw for stabilization
        roll, pitch, yaw = self.quaternion_to_euler(ox, oy, oz, ow)
        
        # Calculate and apply thrust commands
        #self.move()

    def force_callback(self, msg):
        # Update the force part of the desired force-torque vector
        self.FT_desired = [msg.x, msg.y, msg.z]
        self.move()

    def torque_callback(self, msg):
        # Update the torque part of the desired force-torque vector
        print("lol")
        # self.FT_desired[:3] = [msg.x, msg.y, msg.z]

    def quaternion_to_euler(self, x, y, z, w):
        # Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        
        return roll, pitch, yaw

    def objective(self,T):
        return np.linalg.norm(self.A @ T - self.FT_desired)**2
    def move(self):
        # Calculate thrust commands using the updated FT_desired
        #A_pseudo_inv = np.linalg.pinv(self.A)
        #self.T = A_pseudo_inv @ self.FT_desired
        #self.T, _ = nnls(self.A, self.FT_desired)
        T_initial = np.zeros(self.A.shape[1])
        bounds = [(0, 1)] * self.A.shape[1]  # (0, None) means T >= 0
        result = minimize(self.objective, T_initial, bounds=bounds)
        self.T=  result.x
        print("Optimized Thrust Values:", self.T)
        # Scale thrust values to 0-255
        T_scaled = (self.T * 255).astype(int)
        # Prepare the bytes to send
        message_bytes = bytes(T_scaled.tolist())  # plus ten
        print("Message to send (bytes):", message_bytes)
        # Send the values over serial
        try:
            # Initialize serial communication (adjust port and baudrate as needed)
            ser = serial.Serial('/dev/ttyACM0', 115200)  # Replace 'COM3' with the correct port
            ser.write(message_bytes)  # Send bytes
            ser.close()
            print("Message sent successfully!")
        except Exception as e:
            print("Error sending message:", e)

        #print("Thrust Commands:", self.T)

        # Publish thrust commands
        self.t1pub.publish(Float64(data=-self.T[0]))
        self.t2pub.publish(Float64(data=-self.T[1]))
        self.t3pub.publish(Float64(data=-self.T[2]))
        self.t4pub.publish(Float64(data=self.T[3]))

def main(args=None):
    rclpy.init(args=args)
    commander_ = Commander()
    rclpy.spin(commander_)
    commander_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
