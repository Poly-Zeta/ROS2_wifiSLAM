
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg_my = get_package_share_directory('my_robot_bringup')
    urdf_path = os.path.join(pkg_my, 'urdf', 'my_robot.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    # ------------------------------------------------------------
    # Launch configs
    # ------------------------------------------------------------
    unite_imu_method = LaunchConfiguration('unite_imu_method')
    gyro_fps = LaunchConfiguration('gyro_fps')
    accel_fps = LaunchConfiguration('accel_fps')
    use_scan = LaunchConfiguration('use_scan')

    depth_profile = LaunchConfiguration('depth_profile')
    infra_profile = LaunchConfiguration('infra_profile')
    color_profile = LaunchConfiguration('color_profile')

    # ------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------
    declare_unite_imu_method = DeclareLaunchArgument(
        'unite_imu_method',
        default_value='2',
        description='0-None, 1-copy, 2-linear_interpolation'
    )

    declare_gyro_fps = DeclareLaunchArgument(
        'gyro_fps',
        default_value='200',
        description='RealSense gyro fps'
    )

    declare_accel_fps = DeclareLaunchArgument(
        'accel_fps',
        default_value='100',
        description='RealSense accel fps'
    )

    declare_use_scan = DeclareLaunchArgument(
        'use_scan',
        default_value='true',
        description='Use /scan constraint in RTAB-Map'
    )

    declare_depth_profile = DeclareLaunchArgument(
        'depth_profile',
        default_value='424x240x15',
        description='RealSense raw depth profile for infra VO'
    )

    declare_infra_profile = DeclareLaunchArgument(
        'infra_profile',
        default_value='424x240x15',
        description='RealSense infra1 profile for VO'
    )

    declare_color_profile = DeclareLaunchArgument(
        'color_profile',
        default_value='424x240x15',
        description='RealSense color profile for RTAB-Map RGB-D'
    )

    # ------------------------------------------------------------
    # URDF -> TF
    # ------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    # ------------------------------------------------------------
    # RealSense
    # - infra1 + raw depth for visual odom
    # - color + aligned_depth_to_color for RTAB-Map database/cloud_map
    # ------------------------------------------------------------
    ir_emitter_off = SetParameter(
        name='depth_module.emitter_enabled',
        value=0
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            'camera_namespace': '',
            'camera_name': 'camera',

            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': unite_imu_method,
            'gyro_fps': gyro_fps,
            'accel_fps': accel_fps,

            'enable_infra1': 'true',
            'enable_infra2': 'false',
            'enable_color': 'true',
            'enable_depth': 'true',

            'depth_module.depth_profile': depth_profile,
            'depth_module.infra_profile': infra_profile,
            'rgb_camera.color_profile': color_profile,

            'enable_sync': 'true',
            'align_depth.enable': 'true',

            # RTAB-Map / utility nodes create point clouds, not RealSense driver
            'pointcloud.enable': 'false',
        }.items(),
    )

    # ------------------------------------------------------------
    # IMU filter
    # /camera/imu -> /camera/imu/data
    # ------------------------------------------------------------
    madgwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu',
            'publish_tf': False
        }],
        remappings=[
            ('imu/data_raw', '/camera/imu'),
            ('imu/data', '/camera/imu/data'),
        ]
    )

    # ------------------------------------------------------------
    # Visual odometry (infra1 + raw depth + IMU)
    # ------------------------------------------------------------
    visual_odom = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_odom_info': True,
            'approx_sync': False,
            'wait_imu_to_init': False,
            'publish_tf': True,
            'publish_null_when_lost': False,
            'wait_for_transform': 0.2,
            'topic_queue_size': 10,
            'sync_queue_size': 5,
        }],
        remappings=[
            ('imu', '/camera/imu/data'),
            ('rgb/image', '/camera/infra1/image_rect_raw'),
            ('rgb/camera_info', '/camera/infra1/camera_info'),
            ('depth/image', '/camera/depth/image_rect_raw'),
            ('odom', '/rtabmap/odom'),
            ('odom_info', '/rtabmap/odom_info'),
        ]
    )

    # ------------------------------------------------------------
    # Color RGB-D sync for RTAB-Map core
    # color + aligned depth are only for SLAM DB / cloud_map
    # ------------------------------------------------------------
    rgbd_sync_color = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync_color',
        output='screen',
        parameters=[{
            'approx_sync': True,
            'approx_sync_max_interval': 0.03,
            'topic_queue_size': 30,
            'sync_queue_size': 30,
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('rgbd_image', '/rtabmap/rgbd_image'),
        ]
    )

    # ------------------------------------------------------------
    # RTAB-Map core
    # - odom source = infra VO above
    # - mapping image source = color + aligned depth via rgbd_sync
    # - scan used as planar constraint
    # ------------------------------------------------------------
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        arguments=['-d'],
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'publish_tf': True,

            'subscribe_rgbd': True,
            'subscribe_depth': False,
            'subscribe_scan': use_scan,
            'subscribe_odom_info': False,

            'approx_sync': True,
            'odom_sensor_sync': True,
            'wait_imu_to_init': False,
            'wait_for_transform': 1.0,

            'topic_queue_size': 30,
            'sync_queue_size': 30,

            'qos_image': 2,
            'qos_camera_info': 2,
            'qos_odom': 2,
            'qos_scan': 2,

            # 2D LiDAR constraint, 3DoF robot motion
            'Reg/Strategy': '1',
            'RGBD/NeighborLinkRefining': 'true',
            'Reg/Force3DoF': 'true',

            # 2D occupancy mainly from scan
            'Grid/FromDepth': 'false',
            'Grid/Sensor': '0',

            'RGBD/ProximityPathMaxNeighbors': '10',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/LinearUpdate': '0.03',
            'RGBD/AngularUpdate': '0.03',
            'RGBD/OptimizeFromGraphEnd': 'false',

            'Icp/VoxelSize': '0.05',
            'Icp/MaxCorrespondenceDistance': '0.10',
        }],
        remappings=[
            ('odom', '/rtabmap/odom'),
            ('scan', '/scan'),
            ('rgbd_image', '/rtabmap/rgbd_image'),
        ]
    )

    return LaunchDescription([
        declare_unite_imu_method,
        declare_gyro_fps,
        declare_accel_fps,
        declare_use_scan,
        declare_depth_profile,
        declare_infra_profile,
        declare_color_profile,

        robot_state_publisher,
        ir_emitter_off,
        realsense,

        madgwick,
        visual_odom,
        rgbd_sync_color,
        rtabmap,
    ])

