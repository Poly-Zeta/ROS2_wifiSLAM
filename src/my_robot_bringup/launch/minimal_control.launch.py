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
            package='serial_cmdvel_bridge',
            executable='serial_cmdvel_bridge_main',
            name='serial_cmdvel_bridge',
            output='screen'
        ),
    ])
