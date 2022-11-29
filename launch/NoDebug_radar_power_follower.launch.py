from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    tf_drone_to_iwr = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        # arguments=["0", "0", "0.05", "0", "0", "0", "drone", "iwr6843_frame"] # Simulation (x, y, z, yaw, pitch, roll)
        arguments=["0", "0", "0.05", "1.57", "0", "3.14", "drone", "iwr6843_frame"] # Simulation (x, y, z, yaw, pitch, roll)
        #arguments=["0", "0", "0.05", "3.1415", "-1.57079632679", "0", "drone", "iwr6843_frame"] # Simulation
    )

    world_to_drone = Node(
        package="radar_cable_follower",
        executable="drone_frame_broadcaster"
    )

    lidar_to_mmwave = Node(
        package="radar_cable_follower",
        executable="lidar_to_mmwave_node"
    )

    radar_pointcloud_filter = Node(
        package="radar_cable_follower",
        executable="radar_pointcloud_filter",
        parameters=[
            {'launch_with_debug': 0}
         ]
    )

    offboard_control = Node(
        package="radar_cable_follower",
        executable="offboard_control",
        parameters=[
            {'launch_with_debug': 0}
         ]
    )


    return LaunchDescription([
        tf_drone_to_iwr,
        world_to_drone,
        lidar_to_mmwave,
        radar_pointcloud_filter,
        offboard_control
    ])
