from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('line_follower')

    param_file = os.path.join(
        pkg_share,
        'config',
        'line_follower.yaml'
    )

    return LaunchDescription([
        Node(
            package='line_follower',
            executable='line_follower_main',
            name='line_follower',
            output='screen',
            parameters=[param_file]
        )
    ])
