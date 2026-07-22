from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_my = get_package_share_directory('my_robot_bringup')

    # --- Paths inside this package (based on your tree) ---
    urdf_path = os.path.join(pkg_my, 'urdf', 'my_robot.urdf')
    ekf_param_path = os.path.join(pkg_my, 'ekf', 'ekf.yaml')

    # slam params: choose ONE (slam/ or config/)
    slam_param_path = os.path.join(pkg_my, 'slam', 'slamToolbox_myParam.yaml')
    # slam_param_path = os.path.join(pkg_my, 'config', 'slamToolbox_myParam.yaml')

    # --- 1) wheel odom publisher (launch from another package) ---
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('odom_publisher'),
                'launch',
                'odom_publisher.launch.py'
            )
        )
    )

    # --- 2) lidar launch (from another package) ---
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lidar_test_bringup'),
                'launch',
                'ldlidar_only.launch.py'
            )
        )
    )

    # --- 3) robot_state_publisher ---
    # Prefer reading URDF and passing it as robot_description.
    # (ros2 run robot_state_publisher ... my_robot.urdf の代替)
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # --- 4) EKF (robot_localization) ---
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_param_path],
    )

    # --- 5) SLAM toolbox (online_sync_launch.py) ---
    # ros2 launch slam_toolbox online_sync_launch.py slam_params_file:=...
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_sync_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': slam_param_path
        }.items()
    )

    return LaunchDescription([
        odom_launch,
        lidar_launch,
        rsp_node,
        ekf_node,
        slam_launch,
    ])
