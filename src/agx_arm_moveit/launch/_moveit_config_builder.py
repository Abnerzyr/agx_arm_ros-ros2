import ast
import re

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

ALL_ARM_TYPES = ["piper", "piper_x", "piper_l", "piper_h", "nero"]
ALL_EFFECTOR_TYPES = ["none", "agx_gripper", "revo2"]
ALL_REVO2_TYPES = ["left", "right"]


def declare_common_args():
    return [
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="ROS namespace for this arm instance (e.g. arm1).",
        ),
        DeclareLaunchArgument(
            "arm_type", default_value="piper",
            choices=ALL_ARM_TYPES, description="Arm type.",
        ),
        DeclareLaunchArgument(
            "effector_type", default_value="none",
            choices=ALL_EFFECTOR_TYPES, description="Effector type.",
        ),
        DeclareLaunchArgument(
            "revo2_type", default_value="left",
            choices=ALL_REVO2_TYPES,
            description="Revo2 / Revo2 Touch hand side (when effector_type is revo2 or revo2_touch).",
        ),
        DeclareLaunchArgument(
            "tcp_offset",
            default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
            description="TCP offset [x, y, z, rx, ry, rz] in meters/radians.",
        ),
        DeclareLaunchArgument(
            "follow",
            default_value="false",
            choices=["true", "false"],
            description="Follow real arm state. "
            "true: move_group subscribes to feedback_topic; "
            "false: subscribes to control_topic (mock hardware).",
        ),
        DeclareLaunchArgument(
            "feedback_topic",
            default_value="feedback/joint_states",
            description="Joint states feedback topic (used when follow:=true).",
        ),
        DeclareLaunchArgument(
            "control_topic",
            default_value="control/joint_states",
            description="Joint states control topic (used when follow:=false, and for ros2_control_node).",
        ),
    ]


def _select_profile(effector_type: str, revo2_type: str) -> str:
    if effector_type == "agx_gripper":
        return "gripper"
    if effector_type == "revo2":
        return f"revo2_{revo2_type}"
    return "none"


# ─────────────────────────────────────────────────────────────────────
# 帧前缀烘焙（只给 LINK / 帧引用加前缀，JOINT 名保持不带前缀）。
#
# 背景：MoveIt2 humble 不支持 robot_state_publisher 的 frame_prefix 参数
# （实测所有 libmoveit*.so 里都没有 frame_prefix），所以当机械臂进 /arm
# 命名空间时，必须把前缀直接写进 URDF/SRDF 的 link 名里，让 MoveIt 模型
# 的 planning frame 变成 arm/world，与 TF（arm/*）对上。
# 关节状态匹配、ros2_control、moveit_simple_controller_manager 全部用
# JOINT 名（不带前缀），因此 joint 名不能动。
# ─────────────────────────────────────────────────────────────────────
_LINK_NAME_RE = re.compile(r'(<link\s+name=")([^"]+)(")')
_PARENT_LINK_RE = re.compile(r'(parent\s+link=")([^"]+)(")')
_CHILD_LINK_RE = re.compile(r'(child\s+link=")([^"]+)(")')
_FRAME_ID_RE = re.compile(r'(frame_id=")([^"]+)(")')
_SRDF_LINK_RE = re.compile(r'(<link\s+name=")([^"]+)(")')
_SRDF_BASE_LINK_RE = re.compile(r'(base_link=")([^"]+)(")')
_SRDF_TIP_LINK_RE = re.compile(r'(tip_link=")([^"]+)(")')
_SRDF_LINK1_RE = re.compile(r'(link1=")([^"]+)(")')
_SRDF_LINK2_RE = re.compile(r'(link2=")([^"]+)(")')
_SRDF_PARENT_LINK_RE = re.compile(r'(parent_link=")([^"]+)(")')


def _pfx(match, prefix):
    return match.group(1) + prefix + match.group(2) + match.group(3)


def _bake_urdf_links(xml, prefix):
    xml = _LINK_NAME_RE.sub(lambda m: _pfx(m, prefix), xml)
    xml = _PARENT_LINK_RE.sub(lambda m: _pfx(m, prefix), xml)
    xml = _CHILD_LINK_RE.sub(lambda m: _pfx(m, prefix), xml)
    xml = _FRAME_ID_RE.sub(lambda m: _pfx(m, prefix), xml)
    return xml


def _bake_srdf_links(xml, prefix):
    xml = _SRDF_LINK_RE.sub(lambda m: _pfx(m, prefix), xml)
    xml = _SRDF_BASE_LINK_RE.sub(lambda m: _pfx(m, prefix), xml)
    xml = _SRDF_TIP_LINK_RE.sub(lambda m: _pfx(m, prefix), xml)
    xml = _SRDF_LINK1_RE.sub(lambda m: _pfx(m, prefix), xml)
    xml = _SRDF_LINK2_RE.sub(lambda m: _pfx(m, prefix), xml)
    xml = _SRDF_PARENT_LINK_RE.sub(lambda m: _pfx(m, prefix), xml)
    return xml


def build_moveit_config(context):
    arm_type = LaunchConfiguration("arm_type").perform(context)
    effector_type = LaunchConfiguration("effector_type").perform(context)
    revo2_type = LaunchConfiguration("revo2_type").perform(context)
    tcp_offset = ast.literal_eval(
        LaunchConfiguration("tcp_offset").perform(context)
    )

    profile = _select_profile(effector_type, revo2_type)
    urdf_mappings = {
        "arm_type": arm_type,
        "effector_type": effector_type,
        "revo2_type": revo2_type,
        "initial_positions_file": (
            "nero_initial_positions.yaml"
            if arm_type == "nero" else "initial_positions.yaml"
        ),
        "tcp_offset_xyz": f"{tcp_offset[0]} {tcp_offset[1]} {tcp_offset[2]}",
        "tcp_offset_rpy": f"{tcp_offset[3]} {tcp_offset[4]} {tcp_offset[5]}",
    }
    srdf_mappings = {
        "arm_type": arm_type,
        "effector_type": effector_type,
        "revo2_type": revo2_type,
    }

    moveit_config = (
        MoveItConfigsBuilder("agx_arm", package_name="agx_arm_moveit")
        .robot_description(file_path="config/agx_arm.urdf.xacro", mappings=urdf_mappings)
        .robot_description_semantic(
            file_path="config/agx_arm.srdf.xacro", mappings=srdf_mappings
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .trajectory_execution(file_path=f"config/moveit_controllers_{profile}.yaml")
        .to_moveit_configs()
    )

    if arm_type == "nero":
        moveit_config.trajectory_execution[
            "moveit_simple_controller_manager"
        ]["arm_controller"]["joints"] = [
            "joint1", "joint2", "joint3", "joint4",
            "joint5", "joint6", "joint7",
        ]

    moveit_config.trajectory_execution[
        "allowed_start_tolerance"] = 0.05

    moveit_config.planning_pipelines["ompl"].update({
        "arm": {
            "default_planner_config": "RRTstar",
            "planner_configs": {
                "RRTstar": {
                    "type": "geometric::RRTstar",
                    "range": 0.3,
                    "goal_bias": 0.05,
                    "delay_collision_checking": 1,
                },
            },
        },
        "simplify_solutions": True,
        "path_tolerance": 0.05,
    })

    # 命名空间非空时，把前缀烤进 URDF/SRDF 的 link 名（joint 名不动）。
    # 使 MoveIt 模型根 link 变为 arm/world，planning frame 与 TF 一致。
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    if namespace:
        prefix = namespace + "/"
        urdf = moveit_config.robot_description.get("robot_description")
        srdf = moveit_config.robot_description_semantic.get(
            "robot_description_semantic")
        if urdf:
            moveit_config.robot_description["robot_description"] = (
                _bake_urdf_links(urdf, prefix))
        if srdf:
            moveit_config.robot_description_semantic[
                "robot_description_semantic"] = _bake_srdf_links(srdf, prefix)

    return moveit_config
