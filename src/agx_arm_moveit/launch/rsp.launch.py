import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from _moveit_config_builder import build_moveit_config, declare_common_args


def _launch(context):
    follow = LaunchConfiguration("follow").perform(context)
    feedback_topic = LaunchConfiguration("feedback_topic").perform(context)
    control_topic = LaunchConfiguration("control_topic").perform(context)
    joint_states_topic = str(feedback_topic) if follow == "true" else str(control_topic)

    # 命名空间下，link 名前缀已烤进 URDF（见 _moveit_config_builder），
    # 因此这里不再用 frame_prefix（避免双前缀）。
    # robot_state_publisher 会把 TF 发到全局 /tf；实测所有 TF 消费者
    # （RViz / move_group / 视觉节点的 transform_listener）都订阅根 /tf，
    # 所以这里不能把 TF remap 进命名空间，保持发根 /tf 即可。
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    remappings = [("joint_states", joint_states_topic)]

    moveit_config = build_moveit_config(context)

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            respawn=True,
            output="screen",
            parameters=[moveit_config.robot_description],
            remappings=remappings,
        )
    ]


def generate_launch_description():
    return LaunchDescription(declare_common_args() + [OpaqueFunction(function=_launch)])
