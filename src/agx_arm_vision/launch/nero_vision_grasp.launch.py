import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_real_arm = LaunchConfiguration('use_real_arm')
    use_real_camera = LaunchConfiguration('use_real_camera')
    execute_grasp = LaunchConfiguration('execute_grasp')
    use_depth = ParameterValue(
        PythonExpression([
            "'", use_real_camera, "' == 'true' and '",
            LaunchConfiguration('use_depth'), "' == 'true'",
        ]),
        value_type=bool,
    )

    moveit_launch = os.path.join(
        get_package_share_directory('agx_arm_moveit'),
        'launch',
        'demo.launch.py',
    )
    real_arm_launch = os.path.join(
        get_package_share_directory('agx_arm_ctrl'),
        'launch',
        'start_single_agx_arm_moveit.launch.py',
    )

    mock_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        condition=UnlessCondition(use_real_arm),
        launch_arguments={
            'arm_type': 'nero',
            'effector_type': 'agx_gripper',
            'follow': 'false',
            'use_rviz': 'true',
        }.items(),
    )

    real_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(real_arm_launch),
        condition=IfCondition(use_real_arm),
        launch_arguments={
            'arm_type': 'nero',
            'effector_type': 'agx_gripper',
            'can_port': LaunchConfiguration('can_port'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'speed_percent': LaunchConfiguration('speed_percent'),
            'follow': 'true',
            'auto_control_gate': 'true',
        }.items(),
    )

    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        condition=IfCondition(use_real_camera),
        parameters=[{
            'align_depth.enable': True,
            'publish_tf': False,
        }],
    )
    aruco = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        output='screen',
        condition=IfCondition(use_real_camera),
        parameters=[{
            'cam_base_topic': '/camera/camera/color/image_raw',
            'marker_dict': LaunchConfiguration('marker_dict'),
            'marker_size': LaunchConfiguration('marker_size'),
        }],
    )

    virtual_aruco = Node(
        package='agx_arm_vision',
        executable='virtual_aruco_pub',
        output='screen',
        condition=UnlessCondition(use_real_camera),
        parameters=[{'marker_id': LaunchConfiguration('target_marker_id')}],
    )
    virtual_depth = Node(
        package='agx_arm_vision',
        executable='virtual_depth_camera',
        output='screen',
        condition=UnlessCondition(use_real_camera),
    )

    vision = Node(
        package='agx_arm_vision',
        executable='vision_grasp_node',
        output='screen',
        parameters=[{
            'base_frame': 'base_link',
            'target_marker_id': LaunchConfiguration('target_marker_id'),
            'use_depth': use_depth,
        }],
    )
    executor = Node(
        package='agx_arm_vision',
        executable='grasp_executor',
        output='screen',
        condition=IfCondition(execute_grasp),
        parameters=[{
            'base_link': 'base_link',
            'end_effector_link': 'tcp_link',
            'arm_group': 'arm',
            'gripper_joint': 'gripper',
            'gripper_open': 0.1,
            'gripper_closed': 0.0,
            'target_z_offset': LaunchConfiguration('target_z_offset'),
            'constrain_orientation': LaunchConfiguration(
                'constrain_orientation'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_real_arm', default_value='false'),
        DeclareLaunchArgument('use_real_camera', default_value='true'),
        DeclareLaunchArgument(
            'execute_grasp', default_value='true',
            description='Execute grasp sequences for detected targets'),
        DeclareLaunchArgument('can_port', default_value='can0'),
        DeclareLaunchArgument('auto_enable', default_value='false'),
        DeclareLaunchArgument('speed_percent', default_value='10'),
        DeclareLaunchArgument('marker_dict', default_value='4X4_50'),
        DeclareLaunchArgument('marker_size', default_value='0.05'),
        DeclareLaunchArgument('target_marker_id', default_value='0'),
        DeclareLaunchArgument('use_depth', default_value='true'),
        DeclareLaunchArgument('target_z_offset', default_value='0.0'),
        DeclareLaunchArgument(
            'constrain_orientation', default_value='true'),
        mock_arm,
        real_arm,
        realsense,
        aruco,
        virtual_aruco,
        virtual_depth,
        TimerAction(period=5.0, actions=[vision]),
        TimerAction(period=8.0, actions=[executor]),
    ])
