from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_my = get_package_share_directory('my_robot_bringup')
    ekf_param_path=os.path.join(pkg_my,'ekf','ekf_vo_plio_wo.yaml')


    # ------------------------------------------------------------
    # EKF
    # ------------------------------------------------------------
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_param_path],
    )

    return LaunchDescription([
        ekf_node,
    ])
