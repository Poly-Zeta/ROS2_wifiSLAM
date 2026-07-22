from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_my = get_package_share_directory('my_robot_bringup')

    # --- Paths inside this package (based on your tree) ---
    urdf_path = os.path.join(pkg_my, 'urdf', 'my_robot_v2.urdf')

    rkolio_param_path = os.path.join(pkg_my, 'rkolio', 'rko_lio_indoor.yaml')

    # --- 2) lidar launch (from another package) ---
    rkolio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rko_lio'),
                'launch',
                'odometry.launch.py'
            )
        ),
        launch_arguments={
            'config_file': rkolio_param_path
        }.items()
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

    

    return LaunchDescription([
        rkolio_launch,
        rsp_node
    ])
