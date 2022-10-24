from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('iii_drone'),
        'config',
        'params.yaml'
    )

    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe"
        # remappings=[
        #     ("/image_raw", "/usb_cam/image_raw")
        # ]
    )

    mmwave_node = Node(
        package="iwr6843aop_pub",
        executable="pcl_pub"
    )

    tf_drone_to_iwr = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        # arguments=["-0.07", "-0.025", "0.05", "3.1415", "0", "1.57079632679", "drone", "iwr6843_frame"]
    	arguments=["0", "0", "-0.05", "-1.5079632679", "0.04", "1.57079632679", "drone", "iwr6843_frame"]
    )

    world_to_drone = Node(
        package="iii_drone",
        executable="drone_frame_broadcaster"
    )

    hough = Node(
        package="iii_drone",
        executable="hough_interfacer"
        # remappings=[
        #     ("/cable_yaw_angle", "/hough_transformer/cable_yaw_angle")
        # ]
    )

    pl_dir_computer = Node(
        package="iii_drone",
        executable="pl_dir_computer"
    )

    pl_mapper = Node(
        package="iii_drone",
        executable="pl_mapper"
    )

    return LaunchDescription([
        camera_node,
        mmwave_node,
        tf_drone_to_iwr,
        world_to_drone,
        hough,
        pl_dir_computer,
        pl_mapper
    ])
