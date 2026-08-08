from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    marker_dict = LaunchConfiguration('marker_dict')
    marker_size = LaunchConfiguration('marker_size')
    target_marker_id = LaunchConfiguration('target_marker_id')

    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'align_depth.enable': True,
            'publish_tf': False,
        }],
    )

    aruco = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        output='screen',
        parameters=[{
            'cam_base_topic': '/camera/camera/color/image_raw',
            'marker_dict': marker_dict,
            'marker_size': marker_size,
        }],
    )

    vision = Node(
        package='agx_arm_vision',
        executable='vision_grasp_node',
        output='screen',
        parameters=[{
            'base_frame': 'camera_color_optical_frame',
            'target_marker_id': ParameterValue(
                target_marker_id, value_type=int),
            'use_depth': True,
        }],
    )

    debug_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/aruco_tracker/debug'],
        condition=IfCondition(LaunchConfiguration('show_debug')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('marker_dict', default_value='4X4_50'),
        DeclareLaunchArgument('marker_size', default_value='0.05'),
        DeclareLaunchArgument('target_marker_id', default_value='0'),
        DeclareLaunchArgument('show_debug', default_value='true'),
        realsense,
        aruco,
        vision,
        debug_view,
    ])
