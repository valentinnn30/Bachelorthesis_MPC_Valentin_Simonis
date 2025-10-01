from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():


    gui_enabled = True  # Directly set whether to use GUI for joint_state_publisher
    params_file = '/home/reefranger/Desktop/ReefRanger/robot/robot/src/usb_cam/config/params_1.yaml'

    return LaunchDescription([

        # YOLOv8 Bringup Launch File
        ExecuteProcess(
            cmd=['ros2', 'launch', 'yolov8_bringup', 'yolov8.launch.py', 'device:=cpu', 'model:=/home/reefranger/Desktop/ReefRanger/robot/aki/src/yolov8_ros/best.pt'],
            output='screen',
            #parameters=[{"device": "cpu"}]
        ), 

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
        ),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[params_file],
            remappings=[('/image_raw', '/camera/rgb/image_raw')]
         ),

        Node(
            package='nav_yolo',
            executable='tables',
            name='nav_yolo_tables',
            output='screen',
        ),
        Node(
            package='nav_tools',
            executable='STMcom',
            name='STMcom',
            output='screen',
        ),

        Node(
            package='nav_yolo',  # Replace with your package name
            executable='pid',  # Replace with your node executable name
            name='nav_yolo_pid',  # Node name
            output='screen',
        ),     
        # Node(
        #     package='nav_tools',  # Replace with your package name
        #     executable='imu',  # Replace with your node executable name
        #     name='nav_tools_imu',  # Node name
        #     output='screen',
        # ),
        # Node(
        #     package='nav_tools',  # Replace with your package name
        #     executable='thruster_torque',  # Replace with your node executable name
        #     name='nav_tools_thruster_torque',  # Node name
        #     output='screen',
        # ),
        # Node(
        #     package='nav_tools',
        #     executable='orientation_pid',
        #     name='nav_tools_orientation_pid',
        #     output='screen',
        # )
        
    ])
    

