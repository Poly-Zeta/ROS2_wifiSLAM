from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('odom_publisher')
    params = os.path.join(pkg_share, 'config', 'odom_params.yaml')

    return LaunchDescription([
        Node(
            package='odom_publisher',
            executable='odom_publisher_main',
            name='odom_publisher',
            output='screen',
            parameters=[params]
        )
    ])