from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # パッケージのパス取得
    odom_pkg = get_package_share_directory('odom_publisher')
    # ldlidar_pkg = get_package_share_directory('ldlidar')
    # ldlidar_pkg = get_package_share_directory('lidar_test_bringup')
    ldlidar_pkg = get_package_share_directory('scan_reverse')
    slam_pkg = get_package_share_directory('slam_toolbox')
    bringup_pkg = get_package_share_directory('my_robot_bringup')

    return LaunchDescription([
        # i2c_sensors_driver
        Node(
            package='i2c_sensors_driver',
            executable='i2c_sensors_driver_main',
            name='i2c_sensors_driver',
            output='screen'
        ),

        # ldlidar
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ldlidar_pkg, 'launch', 'ldlidar_only.launch.py')#, 'ldlidar.launch.py')
                # os.path.join(ldlidar_pkg, 'launch', 'scan_reverse.launch.py')
            ),
            # launch_arguments={'serial_port': '/dev/ttyAMA1'}.items()
        ),

        # odom_publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(odom_pkg, 'launch', 'odom_publisher.launch.py')
            )
        ),

        # static_tf_from_yaml
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(odom_pkg, 'launch', 'static_tf.launch.py')
            )
        ),

        # ros2serial_arduino
        Node(
            package='ros2serial_arduino',
            executable='serial_send_node',
            name='ros2serial_arduino',
            output='screen'
        ),

        # slam_toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_pkg, 'launch', 'online_sync_launch.py')
            ),
            # 必要なら launch_arguments={'slam_params_file': '...'} を追加
            launch_arguments={'slam_params_file': os.path.join(bringup_pkg,'config','slamToolbox_myParam.yaml')}.items()
        )
    ])