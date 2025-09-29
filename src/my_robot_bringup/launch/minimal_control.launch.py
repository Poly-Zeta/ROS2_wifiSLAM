from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='i2c_sensors_driver',
            executable='i2c_sensors_driver_main',
            name='i2c_sensors_driver',
            output='screen'
        ),
        Node(
            package='ros2serial_arduino',
            executable='serial_send_node',
            name='ros2serial_arduino',
            output='screen'
        )
    ])