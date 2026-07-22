from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_my = get_package_share_directory('my_robot_bringup')
    urdf_path = os.path.join(pkg_my, 'urdf', 'my_robot.urdf')
    ekf_param_path = os.path.join(pkg_my, 'ekf', 'ekf.yaml')#_rtabmap.yaml')

    #odom publisher / lidar
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('odom_publisher'), 'launch', 'odom_publisher.launch.py')
        )
    )
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lidar_test_bringup'), 'launch', 'ldlidar_only.launch.py')
        )
    )

    # RSP
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # EKF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_param_path],
    )

    # RealSense
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_namespace': 'realsense',
            'camera_name': 'd435i',
            'publish_tf': 'true', #'false',
            'base_frame_id': 'camera_link',
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'unite_imu_method': '1',
        }.items()
    )

    # RTAB-Map
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'publish_tf': True,
            'subscribe_depth': True,
            'subscribe_scan': True,
            'approx_sync': True,
        }],
        remappings=[
            ('odom', '/odometry/filtered'),

            # LiDAR
            ('scan', '/scan'),

            # RealSense
            ('rgb/image',       '/realsense/d435i/color/image_raw'),
            ('rgb/camera_info', '/realsense/d435i/color/camera_info'),
            ('depth/image',     '/realsense/d435i/aligned_depth_to_color/image_raw'),
        ],
    )

    return LaunchDescription([
        odom_launch,
        lidar_launch,
        rsp_node,
#        ekf_node,
        realsense_launch,
#        rtabmap_node,
    ])

