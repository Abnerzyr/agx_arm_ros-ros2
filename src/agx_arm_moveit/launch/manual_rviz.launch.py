import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description():
    # 手动 RViz 进 /arm 命名空间的可靠方式（与 demo.launch.py 的 MoveIt RViz 同款机制）：
    # PushRosNamespace + SetRemap 让 rviz2 订阅 /arm/robot_description 与 /arm/tf。
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='arm'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                '/home/s1/tiaozhanbei/agx_arm_ros-ros2',
                'src/config/yolo_config_armns.rviz'),
            description='Path to the RViz config to load.',
        ),
        GroupAction(
            actions=[
                PushRosNamespace(LaunchConfiguration('namespace')),
                SetRemap(src='/robot_description', dst='robot_description'),
                Node(
                    package='rviz2',
                    executable='rviz2',
                    output='screen',
                    arguments=['-d', LaunchConfiguration('rviz_config')],
                ),
            ]
        ),
    ])
