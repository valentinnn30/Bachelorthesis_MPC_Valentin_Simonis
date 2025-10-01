from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to URDF, RVIZ, and apriltag configuration files
    # Paths to URDF, RVIZ, and apriltag configuration files
    default_model_path = '/home/reefranger/Desktop/ReefRanger/robot/robot/src/robotpub/desc/robot.urdf'
    default_rviz_config_path = '/home/reefranger/Desktop/ReefRanger/robot/robot/src/robotpub/desc/urdf.rviz'
    params_file = '/home/reefranger/Desktop/ReefRanger/robot/robot/src/usb_cam/config/params_1.yaml'

    gui_enabled = True  # Directly set whether to use GUI for joint_state_publisher

    return LaunchDescription([

        # #Start the USB camera node
    Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[params_file],
        remappings=[('/image_raw', '/camera/rgb/image_raw')]
    ),

    Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}]
    ),

    Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
        condition=UnlessCondition(str(gui_enabled))
    ),

        #YOLOv8 Bringup Launch File
        ExecuteProcess(
            cmd=['ros2', 'launch', 'yolov8_bringup', 'yolov8.launch.py', 'device:=cpu'],
            output='screen',
            #parameters=[{"device": "cpu"}]
        ), 
   

      
  
        ExecuteProcess(
            cmd=['ros2', 'run', 'mccontroller', 'mccontroller'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'save_detect', 'save_detect'],
            output='screen'
        ),

        # Path Control Node
        ExecuteProcess(
            cmd=['ros2', 'run', 'commander', 'commander'],
            output='screen'#'log' if we want to store logs in a file
        ),

        ExecuteProcess(
            cmd=['ros2', 'launch', 'foxglove_bridge', 'foxglove_bridge_launch.xml'],
            output='screen'
        ),
        
        Node(
            package='moveovertable',  # Replace with your package name
            executable='moveovertable',  # Replace with your node executable name
            name='moveovertable',  # Node name
            output='screen',
        ),

        #Yolo 
        Node(
            package='gototable',  # Replace with your package name
            executable='gototable',  # Replace with your node executable name
            name='gototable',  # Node name
            output='screen',
        ),

    ])

