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

    gui_enabled = True  # Directly set whether to use GUI for joint_state_publisher

    return LaunchDescription([

        #Start the USB camera node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
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



        ExecuteProcess(
            cmd=['ros2', 'run', 'mccontroller', 'mccontroller'],
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

        # Start ORB-SLAM3 driver
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
        # Start the occupancy grid node (MapGenerator)
        Node(
            package='occupancy_grid',
            executable='occupancy_grid',
            name='map_generator',
            output='screen'
        ),

        # Start the keyboard reset node (KeyboardReset)
        Node(
            package='occupancy_grid',
            executable='reset_cloudpoints',
            name='reset_cloudpoints',
            output='screen'
        ),
        
        #ORBSlam
        Node(
            package='navigate_path',  # Replace with your package name
            executable='navigate_path',  # Replace with your node executable name
            name='path_navigator',  # Node name
            output='screen'
        ),
        
        Node(
            package='moveovertable',  # Replace with your package name
            executable='moveovertable',  # Replace with your node executable name
            name='moveovertable',  # Node name
            output='screen',
        ),

    ])
    



