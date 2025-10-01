from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the USB camera node
        # Node(
        #     package='usb_cam',
        #     executable='usb_cam_node_exe',
        #     name='usb_cam',
        #     output='screen'
        # ),

        # Start ORB-SLAM3 core
        Node(
            package='ros2_orb_slam3',
            executable='mono_node_cpp',
            name='orb_slam_core',
            output='screen',
            parameters=[{'node_name_arg': 'mono_slam_cpp'}]
        ),

        # Start ORB-SLAM3 driver
        Node(
            package='ros2_orb_slam3',
            executable='mono_driver_node.py',
            name='orb_slam_driver',
            output='screen',
            parameters=[{'settings_name': 'EuRoC', 'image_seq': 'sample_euroc_MH05'}]
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

        # Start RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])

