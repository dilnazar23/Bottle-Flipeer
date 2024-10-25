from launch import LaunchDescription
##from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
##from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('flip'),
        'config',
        'trajectory_config.yaml'
        )

    return LaunchDescription(
        [
            Node(
                package="flip",
                executable="flip",
                name="flip_pub",
                parameters=[config],
                output="screen",
            )
        ]
    )
