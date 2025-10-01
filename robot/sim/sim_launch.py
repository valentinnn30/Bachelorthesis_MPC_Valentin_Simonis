from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    
   
    return LaunchDescription([


        # ROS-GZ bridge for camera image
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_image_bridge',
            arguments=['/camera/rgb/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),

        # ROS-GZ bridge for camera info
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_info_bridge',
            arguments=['/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
            output='screen'
        ),

        # ROS-gz bridge for propeller joint command position
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='propeller_joint_bridge',
            arguments=['/model/tethys/joint/propeller_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double'],
            output='screen'
        ),

        # ROS-gz bridge for vertical command
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='vertical_bridge',
            arguments=['/vertical@std_msgs/msg/Float64@gz.msgs.Double'],
            output='screen'
        ),

        # ROS-gz bridge for horizontal command
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='horizontal_bridge',
            arguments=['/horizontal@std_msgs/msg/Float64@gz.msgs.Double'],
            output='screen'
        ),

        # ROS-GZ bridge for IMU data
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='imu_bridge',
            arguments=['/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU'],
            output='screen'
        ),

        # Launch the robot_localization node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                'efk.yaml'
            ]
        ),
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bcu2c_buoyancy_bridge',
            arguments=['/model/bcu2c/buoyancy_engine@std_msgs/msg/Float64@gz.msgs.Double'],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='pose_bridge',
            arguments=['/model/own/pose@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V'],
            output='screen'
        ),

        # ROS-gz bridge for thruster command
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='thruster_cmd_bridge',
            arguments=['/thruster/cmd@std_msgs/msg/Float64@gz.msgs.Double'],
            output='screen'
        ),        
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='umdrehung_1_bridge',
            arguments=['/umdrehung_1/thrust@std_msgs/msg/Float64@gz.msgs.Double'],
            output='screen'
        ),

        # ROS-gz bridge for umdrehung_2 thrust command
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='umdrehung_2_bridge',
            arguments=['/umdrehung_2/thrust@std_msgs/msg/Float64@gz.msgs.Double'],
            output='screen'
        ),
                # ROS-gz bridge for umdrehung_2 thrust command
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='umdrehung_2_bridge',
            arguments=['/navsat@sensor_msgs/NavSatFix@gz.msgs.NavSat'],
            output='screen'
        ),


        # ROS-gz bridge for umdrehung_3 thrust command
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='umdrehung_3_bridge',
            arguments=['/umdrehung_3/thrust@std_msgs/msg/Float64@gz.msgs.Double'],
            output='screen'
        ),

        # ROS-gz bridge for umdrehung_4 thrust command
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='umdrehung_4_bridge',
            arguments=['/umdrehung_4/thrust@std_msgs/msg/Float64@gz.msgs.Double'],
            output='screen'
        ),
                Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='umdrehung_1_cmd_thrust_bridge',
            arguments=['/model/tethys/joint/umdrehung_1/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double'],
            output='screen'
        ),

        # ROS-gz bridge for umdrehung_2 cmd_thrust command
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='umdrehung_2_cmd_thrust_bridge',
            arguments=['/model/tethys/joint/umdrehung_2/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double'],
            output='screen'
        ),

        # ROS-gz bridge for umdrehung_3 cmd_thrust command
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='umdrehung_3_cmd_thrust_bridge',
            arguments=['/model/tethys/joint/umdrehung_3/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double'],
            output='screen'
        ),

        # ROS-gz bridge for umdrehung_4 cmd_thrust command
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='umdrehung_4_cmd_thrust_bridge',
            arguments=['/model/tethys/joint/umdrehung_4/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double'],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='navsatbridge',
            arguments=['/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat'],
            output='screen'
        ),


    ])
