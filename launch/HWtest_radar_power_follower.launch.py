from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('radar_cable_follower'),
        'config',
        'params.yaml'
    )

    tf_drone_to_iwr = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        # arguments=["0", "0", "0.05", "0", "0", "0", "drone", "iwr6843_frame"] # Simulation (x, y, z, yaw, roll, pitch)
        arguments=["0", "0.0", "-0.08", "1.570796", "0.0", "-1.570796", "drone", "iwr6843_frame"] # Simulation (x, y, z, yaw, roll, pitch)
    )

    world_to_drone = Node(
        package="radar_cable_follower",
        executable="drone_frame_broadcaster"
    )

    mmwave = Node(
        package="iwr6843isk_pub",
        executable="pcl_pub",
        parameters=[
            {'cfg_path': '/home/ubuntu/ros2_ws/src/iwr6843isk_ros2/cfg_files/xwr68xx_profile_25Hz_Elev_43m.cfg'},
            {'cli_port': '/dev/ttyUSB0'},
            {'data_port': '/dev/ttyUSB1'}
         ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    radar_pointcloud_filter = Node(
        package="radar_cable_follower",
        executable="radar_pointcloud_filter",
        parameters=[config]
    )

    offboard_control = Node(
        package="radar_cable_follower",
        executable="offboard_control",
        parameters=[config]
    )


    return LaunchDescription([
        tf_drone_to_iwr,
        world_to_drone,
        mmwave,
        radar_pointcloud_filter,
        offboard_control
    ])