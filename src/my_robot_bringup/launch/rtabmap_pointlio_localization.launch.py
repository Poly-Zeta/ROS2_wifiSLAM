import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg_my = get_package_share_directory('my_robot_bringup')

    urdf_filepath=os.path.join(pkg_my,'urdf','my_robot_v2.urdf')

    ekf_param_path=os.path.join(pkg_my,'ekf','ekf_vo_plio_wo.yaml')

    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    pointlio_launch = os.path.join(
        get_package_share_directory('point_lio'),
        'launch',
        'correct_odom_unilidar_l2.launch.py'
    )

    database_path=LaunchConfiguration('database_path')
    declare_rtabmap_db=DeclareLaunchArgument(
        'database_path',
        default_value='~/maps/rtabmap/room_vicp4.db',
        description='RTAB-Map database path.'
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
        default_value='1',
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
        #default_value='640x480x15',
        default_value='424x240x15',
        description='RealSense raw depth profile for infra VO'
    )

    declare_infra_profile = DeclareLaunchArgument(
        'infra_profile',
        #default_value='640x480x15',
        default_value='424x240x15',
        description='RealSense infra1 profile for VO'
    )

    declare_color_profile = DeclareLaunchArgument(
        'color_profile',
        #default_value='640x480x15',
        default_value='424x240x15',
        description='RealSense color profile for RTAB-Map RGB-D'
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_filepath]
    )

    ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
        output='screen',
        parameters=[ekf_param_path],
    )

    pointlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pointlio_launch)
    )
    
    body_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_link_to_body',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'body',
        ]
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

            'enable_gyro': 'false',
            'enable_accel': 'false',
            'unite_imu_method': 'false',
            'gyro_fps': gyro_fps,
            'accel_fps': accel_fps,

            'enable_infra1': 'true',
            'enable_infra2': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',

            'depth_module.depth_profile': depth_profile,
            'depth_module.infra_profile': infra_profile,
            'rgb_camera.color_profile': color_profile,

            'enable_sync': 'true',
            'align_depth.enable': 'true',
            #'publish_tf': 'false',
            # RTAB-Map / utility nodes create point clouds, not RealSense driver
            'pointcloud.enable': 'false',
        }.items(),
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
            'approx_sync_max_interval': 0.1,
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
#        arguments=['-d'],
        parameters=[{

            'database_path': database_path,
            # localization mode
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',


            'frame_id': 'base_link',
            'odom_frame_id':'',# 'odom',
            'map_frame_id': 'map',
            'publish_tf': True,

            'subscribe_rgb': False,
            'subscribe_rgbd': True,
            'subscribe_depth': False,
            'subscribe_scan_cloud': True,
            'subscribe_scan': False,
            'subscribe_odom_info': False,

            'Reg/Strategy': '2',
            'Icp/VoxelSize': '0.05',
            'Icp/PointToPlane': 'true',

            'Rtabmap/DetectionRate': '0.5',
            'RGBD/LinearUpdate': '0.3',
            'RGBD/AngularUpdate': '0.3',


#            'publish_tf': False,
#            'Reg/Strategy': '0',
#            'Kp/MaxFeatures': '-1',            


            'approx_sync': True,
            'odom_sensor_sync': True,
            'wait_imu_to_init': False,
            'wait_for_transform': 1.0,

            'topic_queue_size': 50,
            'sync_queue_size': 50,

            'qos_image': 2,
            'qos_camera_info': 2,
            'qos_odom': 2,
            'qos_scan': 2,

            'Grid/Sensor': "2",
            'Grid/3D': "false",

            'Grid/NormalsSegmentation': 'true',
            'Grid/MinGroundHeight': '-0.5',
            'Grid/MaxGroundHeight': '0.15',
            'Grid/MaxObstacleHeight': '0.2',


            'Grid/CellSize': "0.05",
            'Grid/RangeMax': "8",

            'Reg/Force3DoF': 'false',
            'RGBD/OptimizeFromGraphEnd': 'false',
        }],
        remappings=[
            ('odom', '/odom_corrected'),

           # ('rgb/image', '/camera/color/image_raw'),
           # ('rgb/camera_info', '/camera/color/camera_info'),
           # ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('rgbd_image', '/rtabmap/rgbd_image'),
           # ('scan_cloud', '/unilidar/cloud'),
            ('scan_cloud', '/cloud_registered_body'),
        ]
    )

    return LaunchDescription([
        declare_rtabmap_db,
        declare_unite_imu_method,
        declare_gyro_fps,
        declare_accel_fps,
        #declare_use_scan,
        declare_depth_profile,
        declare_infra_profile,
        declare_color_profile,

        robot_state_publisher_node,
        body_to_base_tf,
        ekf_node,
        pointlio,
        ir_emitter_off,
        realsense,
        rgbd_sync_color,
        rtabmap,
    ])

