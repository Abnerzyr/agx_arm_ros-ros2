import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_path = os.path.join(
        get_package_share_directory('agx_arm_moveit'),
        'config',
        'agx_arm.urdf.xacro',
    )
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_path,
            ' arm_type:=nero effector_type:=agx_gripper',
        ]),
        value_type=str,
    )

    arm_driver = Node(
        package='agx_arm_ctrl',
        executable='agx_arm_ctrl_single',
        name='agx_arm_ctrl_single_node',
        output='screen',
        parameters=[{
            'can_port': LaunchConfiguration('can_port'),
            'arm_type': 'nero',
            'effector_type': 'agx_gripper',
            'auto_enable': False,
            'control_enabled': False,
            'speed_percent': 5,
            'fw_version': 'v111',
            'pub_rate': 200,
            'enable_timeout': 5.0,
            'tcp_offset': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'gripper_default_effort': 1.0,
            'auto_home': False,
        }],
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        remappings=[('joint_states', 'feedback/joint_states')],
    )

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

    pc_grasp = Node(
        package='agx_arm_vision',
        executable='pointcloud_grasp',
        output='screen',
        parameters=[{'publish_static_joints': False}],
    )

    rviz_config = os.path.join(
        get_package_share_directory('agx_arm_moveit'),
        'config',
        'default_config.rviz',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('can_port', default_value='can1'),
        arm_driver,
        robot_state_pub,
        realsense,
        pc_grasp,
        rviz,
    ])
