from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    params_file = '/home/reefranger/Desktop/ReefRanger/robot/robot/src/usb_cam/config/params_1.yaml'

    return LaunchDescription([

        #Foxglove
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
        ),

        #Camera
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[params_file],
            remappings=[('/image_raw', '/camera/rgb/image_raw')]
         ),

        #STM
        Node(
            package='nav_tools',
            executable='STMcom',
            name='STMcom',
            output='screen',
        ),

        # YOLO
        ExecuteProcess(
            cmd=['ros2', 'launch', 'yolov8_bringup', 'yolov8.launch.py', 'device:=cpu', 'model:=/home/reefranger/Desktop/ReefRanger/robot/aki/src/yolov8_ros/best.pt'],
            output='screen',
            #parameters=[{"device": "cpu"}]
        ), 

        Node(
            package='nav_yolo',
            executable='tables',
            name='nav_yolo_tables',
            output='screen',
        ),

        Node(
            package='nav_yolo', 
            executable='pid',  
            name='nav_yolo_pid',  
            output='screen',
        ),     

        #SLAM
        Node(
            package='nav_slam',
            executable='changepath',
            name='nav_slam_changepath',
            output='screen',
        ),

        Node(
            package='nav_slam', 
            executable='path', 
            name='nav_slam_path_navigator',  
            output='screen'
        ),

        Node(
            package='nav_slam',  
            executable='pid',  
            name='nav_slam_pid_controller',  
            output='screen',
        ),

        Node(
            package='nav_slam',  
            executable='grid',  
            name='nav_slam_generate_grid', 
            output='screen',
        ),  

        Node(
            package='nav_slam',  
            executable='feeding',  
            name='nav_slam_feeding', 
            output='screen',
        ),  

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'apriltag_ros', 'apriltag_node',
                '--ros-args',
                '-r', '/image_rect/compressed:=/image_raw/compressed',
                '-r', 'camera_info:=/camera_info',
                '--params-file', '/home/reefranger/Desktop/ReefRanger/robot/aki/src/apriltag_ros/cfg/tags_36h11.yaml'
            ],
            output='screen'
        ),
    ])
    

