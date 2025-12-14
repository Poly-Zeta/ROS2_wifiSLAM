from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uart_wifi_analyzer',
            executable='uart_wifi_analyzer_main',
            name='uart_wifi_analyzer',
            output='screen',
            parameters=[{
                'device': '/dev/ttyUSB-ESP32C5-WifiAnalyzer',
                'baudrate': 115200,
            }],
        )
    ])