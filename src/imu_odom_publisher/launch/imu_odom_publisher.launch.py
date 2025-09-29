from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_odom_publisher',
            executable='imu_odom_publisher_main',
            name='imu_odom_publisher',
            output='screen',
            parameters=[
                {"imu_topic": "i2cReceive/imu"},
                {"odom_topic": "/imu_odom"},
                {"odom_frame": "odom"},
                {"base_frame": "base_link"},
                {"publish_tf": True}
            ]
        )
    ])
