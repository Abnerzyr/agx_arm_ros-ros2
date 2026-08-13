import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    demo_launch = os.path.join(
        get_package_share_directory('agx_arm_moveit'),
        'launch',
        'demo.launch.py',
    )

    pkg_share = get_package_share_directory('agx_arm_vision')
    sim_rviz = os.path.join(
        pkg_share, 'rviz', 'grasp_executor_sim.rviz')

    mock_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo_launch),
        launch_arguments={
            'arm_type': 'nero',
            'effector_type': 'agx_gripper',
            'follow': 'false',
            'use_rviz': 'false',
        }.items(),
    )

    mock_gripper = Node(
        package='agx_arm_vision',
        executable='mock_gripper',
        output='screen',
        parameters=[{
            'simulate_object': LaunchConfiguration('simulate_object'),
        }],
    )

    executor = Node(
        package='agx_arm_vision',
        executable='grasp_executor',
        output='screen',
        remappings=[
            ('/feedback/joint_states', '/control/joint_states'),
        ],
    )

    target_marker = Node(
        package='agx_arm_vision',
        executable='grasp_target_marker',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', sim_rviz],
        condition=IfCondition(LaunchConfiguration('use_sim_rviz')),
    )

    random_flow = Node(
        package='agx_arm_vision',
        executable='random_grasp_flow',
        output='screen',
        parameters=[{
            'max_step': 0.35,
            'min_z': 0.30,
            'max_z': 0.60,
            'max_radius': 0.50,
        }],
        condition=IfCondition(LaunchConfiguration('random_test')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('simulate_object', default_value='true'),
        DeclareLaunchArgument('use_sim_rviz', default_value='true'),
        DeclareLaunchArgument('random_test', default_value='true'),
        mock_arm,
        mock_gripper,
        executor,
        target_marker,
        rviz,
        random_flow,
    ])
