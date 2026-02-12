from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_name = LaunchConfiguration('port_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ldlidar',
            description='Serial device path for LDLiDAR'
        ),

        Node(
            package='ldlidar_ros2',
            executable='ldlidar_ros2_node',
            name='ldlidar_publisher_ld06',
            output='screen',
            parameters=[{
                'product_name': 'LDLiDAR_LD06',
                'laser_scan_topic_name': 'scan',
                'point_cloud_2d_topic_name': 'pointcloud2d',
                'frame_id': 'laser',
                'port_name': port_name,
                'serial_baudrate': 230400,
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
                'angle_crop_min': 0.0,
                'angle_crop_max': 0.0,
                'range_min': 0.02,
                'range_max': 12.0
            }]
        )
    ])
