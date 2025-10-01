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
            executable='STMcom_mpc',
            name='STMcom_mpc',
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
            package='ros2_orb_slam3',
            executable='mono_driver_node.py',
            name='orb_slam_driver',
            output='screen',
            parameters=[{'settings_name': 'EuRoC', 'image_seq': 'sample_euroc_MH05'}]
        ),

 	    Node(
            package='ros2_orb_slam3',
            executable='mono_node_cpp',
            name='orb_slam_core',
            output='screen',
            parameters=[{'node_name_arg': 'mono_slam_cpp'}]
        ),

        

        
    ])
    

