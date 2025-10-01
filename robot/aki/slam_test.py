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

        #ORBSlam
        # Node(
        #     package='nav_tools',
        #     executable='thruster',
        #     name='nav_tools_thruster',
        #     output='screen',
        # ),

        Node(
            package='nav_tools',
            executable='STMcom',
            name='STMcom',
            output='screen',
        ),
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
        # Node(
        #     package='nav_slam',
        #     executable='reset',
        #     name='nav_slam_reset_cloudpoints',
        #     output='screen'
        # ),
        # Node(
        #     package='nav_slam', 
        #     executable='path', 
        #     name='nav_slam_path_navigator',  
        #     output='screen'
        # ),
        Node(
            package='nav_slam',  
            executable='pid_test',  
            name='nav_slam_pid_controller',  
            output='screen',
        ),
        # Node(
        #     package='nav_slam',  
        #     executable='pid',  
        #     name='nav_slam_pid_controller',  
        #     output='screen',
        # ),
        # Node(
        #     package='nav_slam',  
        #     executable='grid',  
        #     name='nav_slam_generate_grid', 
        #     output='screen',
        # ),  
       
    ])
    
    

