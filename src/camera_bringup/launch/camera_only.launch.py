from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    WIDTH  = 800
    HEIGHT = 600
    FORMAT = "RGB888"
    CAMERA = 0
    ROLE   = "viewfinder"

    # AF を Continuous で起動したい
    AF_MODE = 2   # 0:Manual, 1:Auto, 2:Continuous

    TARGET_FPS = 30
    frame_us = int(1_000_000 / TARGET_FPS)
    frame_limits = f"[{frame_us},{frame_us}]"

    container = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="camera_ros",
                plugin="camera::CameraNode",
                name="camera",
                parameters=[{
                    "camera": CAMERA,
                    "width": WIDTH,
                    "height": HEIGHT,
                    "format": FORMAT,
                    "role": ROLE,

                    # 起動時に効くならラッキー枠（効かない可能性があるので後段でも設定）
                    "AfMode": AF_MODE,
                    "FrameDurationLimits": [frame_us, frame_us],
                }],
            )
        ],
    )

    # --- workaround：起動後に確実にパラメータ反映 ---

    set_afmode = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            "source /opt/ros/jazzy/setup.bash; "
            "source ~/ros_ws/install/setup.bash; "
            # /camera が生きてる（param getできる）まで待つ
            "until ros2 param get /camera AfMode >/dev/null 2>&1; do sleep 0.2; done; "
            # 連続AF
            "ros2 param set /camera AfMode 2; "
            # 反映確認を出力（ログに証拠が残る）
            "ros2 param get /camera AfMode"
        ],
        output="screen",
    )


    set_fps = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            "source /opt/ros/jazzy/setup.bash; "
            "source ~/ros_ws/install/setup.bash; "
            "until ros2 param get /camera FrameDurationLimits >/dev/null 2>&1; do sleep 0.2; done; "
            "ros2 param set /camera FrameDurationLimits '[33333,33333]'; "
            "ros2 param get /camera FrameDurationLimits"
        ],
        output="screen",
    )

   

# /camera が出来てから叩く（順序も保証）
    post_start = TimerAction(
        period=2.0,
        actions=[set_afmode, set_fps]
    )

    return LaunchDescription([container, post_start])
