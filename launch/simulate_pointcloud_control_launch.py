from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='iii_drone',
            executable='img_3d_to_2d_proj',
            name='img_3d_to_2d_proj'
        ),
        Node(
            package='iii_drone',
            #namespace='turtlesim2',
            executable='lidar_to_mmwave',
            name='lidar_to_mmwave'
        )
    ])
