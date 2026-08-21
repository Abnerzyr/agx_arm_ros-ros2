# 机械臂命名空间改造方案（只改臂，零影响原有功能）

> 状态：**待评审**（未开始改代码）
> 目标：解决底盘（chassis_bridge/Nav2）与机械臂（agx_arm）并行运行时 `/robot_description` 互相覆盖、TF `base_link` 冲突的问题。
> 方案：只改机械臂侧，机械臂进 `/arm` 命名空间；底盘保持根命名空间零改动。

---

## 1. 问题背景

底盘链路（`chassis_bridge` / Nav2）和机械臂（`agx_arm`）同时运行时，两个模型的 `robot_state_publisher` 互相干扰：

- RViz 里模型错乱 / 模型被另一个覆盖
- TF 树报 `base_link` 冲突（multiple authority）错误
- `/robot_description` 被互相覆盖

### 根因

| 冲突点 | 底盘（lapis URDF） | 机械臂（nero URDF） |
|--------|-------------------|---------------------|
| `/robot_description` | 发布自己的模型 | 也发布自己的模型 → 互相覆盖 |
| TF 帧 `base_link` | `base_footprint → base_link → 轮子/IMU/雷达` | `world → base_link → link1…` → 帧名冲突 |

机械臂 `demo.launch.py` 默认 `namespace=""`（根命名空间），与底盘撞在一起。

### 为什么不改底盘（对比）

底盘侧与根命名空间深度绑定（已核实 `ekf_real.yaml` 里 `odom0: /odom_raw`、`odom_frame: odom`、`base_link_frame: base_footprint` 全是硬编码绝对话题/根帧）：

| 改动项 | 机械臂加 namespace | 底盘加 namespace |
|--------|------------------|-----------------|
| launch 命令 | 集中在臂侧工作区 | 4 个 launch + C++ 节点话题（`/odom` `/imu/data` `/wheel_odometry`）+ 脚本全要包命名空间 |
| Nav2 参数 | 无关 | nav2_params.yaml 几十处帧/话题名要改 |
| EKF | 无关 | ekf_real.yaml 的 odom0/imu0/帧名要改 |
| SLAM | 无关 | slam.yaml + slam_toolbox 段要改 |
| 雷达链 | 无关 | velodyne frame_id、pcl_to_scan 要改 |
| 网页遥控 | 无关 | /cmd_vel 话题要改 |
| 出错风险 | 低 | 高（漏改一处导航就静默失效） |

**结论：机械臂侧集中改动；底盘零接触。**

---

## 2. 设计原则：默认行为严格不变

所有改动以 **`namespace` 为空 / 节点不带 `__ns` 时，行为与现状一致** 为基准。利用 ROS2 两条机制保证：

1. **相对话题在根命名空间 = 全局**：`'task_command'` 不带 `__ns` 时解析为 `/task_command`，与现状完全相同；
2. **`frame_prefix` 为空 = 无前缀**：launch 里 `frame_prefix = namespace.strip("/") + "/"`，namespace 为空时为空串，TF 帧、`octomap_frame` 全部回退现状。

只有显式传 `namespace:=arm` + `__ns:=/arm` + 帧参数，才启用分组。现有 `start_shelf.sh` / `start_shelf_sim.sh` / `start_shelf_realcam.sh` / `start_pc.sh` 等**一行不改**，照常运行。

---

## 3. 改动总览（3 层，全部在臂侧工作区 `agx_arm_ros-ros2`）

| 层 | 包 | 内容 | 文件数 |
|----|----|------|--------|
| 1 | `agx_arm_moveit` | frame_prefix 贯彻（static TF / move_group）+ 点云话题相对化 | 4-5 |
| 2 | `agx_arm_vision` | 视觉/工作流节点话题相对化（跨组话题保持绝对） | 7 |
| 3 | 启动脚本 | 新增 `start_shelf_realcam_armns.sh`，不动现有脚本 | 1（新增） |

---

## 4. 详细改动清单

### 第 1 层：launch 层（agx_arm_moveit）

| 文件 | 改动 | 默认行为（namespace 空） |
|------|------|--------------------------|
| `launch/rsp.launch.py` | ✅ 已加 `frame_prefix`（当前未提交改动），**不用再动** | — |
| `launch/static_virtual_joint_tfs.launch.py` | 手写 `static_transform_publisher`（**不能用** moveit_configs_utils 的 `generate_static_virtual_joint_tfs_launch`，它不读 frame_prefix）：读 namespace，`world→base_link` 变 `arm/world→arm/base_link` | 空 ns 时与现状一致 |
| `launch/move_group.launch.py` | ① parameters 加 `{"frame_prefix": fp}`（MoveIt 原生支持，planning frame 自动加前缀）；② `octomap_frame` 改 `fp + "base_link"` | 空 fp 回退 `base_link`、无前缀键 |
| `config/sensors_3d.yaml` | 两个 `point_cloud_topic` 从绝对 `/yolo/points_filtered`、`/place/points_filtered` 改相对（`filtered_cloud_topic` 已是相对，不动） | 根空间下解析结果相同 |
| `launch/_moveit_config_builder.py` | （可选）加共享 helper `_frame_prefix(namespace)`，避免三处重复计算 | 纯重构，无行为变化 |

**RViz 不用改**：`PushRosNamespace` + `SetRemap(/robot_description → robot_description)` 已覆盖——RViz 在 `/arm` 下时 RobotModel 自动读 `/arm/robot_description`；Move Group Namespace 已由 `_build_namespaced_moveit_rviz_config` 生成。

### 第 2 层：视觉/工作流层（agx_arm_vision）

**改法**：下列绝对话题改为相对（去掉前导 `/`）；`/camera/camera/...` 和 `/aruco_detections` **保持绝对**（跨组访问外部节点）。

| 文件 | 相对化话题 |
|------|-----------|
| `moveit2_local.py` | `move_action`、`apply_planning_scene`、`compute_cartesian_path`、`arm_controller/follow_joint_trajectory`（4 个服务/动作） |
| `shelf_workflow_node.py` | `task_command`、`release_command`、`manual_grasp_start`、`manual_release`、`grasp_executor_state`、`yolo/target_box`、`grasp_pose`、`shelf/align_target`（`aruco_detections` 保持绝对） |
| `grasp_executor.py` | 全部 16 个：`clear_octomap`(服务)、`control/move_j`、`feedback/gripper_status`、`feedback/joint_states`、`filtered_cloud`、`grasp_executor_state`、`grasp_pose`、`manual_grasp_start`、`manual_release`、`map_update_enable`、`place_filtered_cloud`、`place/points_filtered`、`place_pose`、`place_update_enable`、`yolo/points_filtered`、`yolo/target_box` |
| `yolo_grasp_node.py` | `grasp_pose`、`map_update_enable`、`yolo/crop_rgb`、`yolo/detections`、`yolo/points`、`yolo/points_filtered`、`yolo/quality_map`、`yolo/target_box`（相机 3 个保持绝对） |
| `place_planner.py` | `place_pose`、`place_update_enable`、`place/points_filtered`、`place_target_marker`（相机 2 个保持绝对） |
| `grasp_target_marker.py` | `grasp_pose`、`grasp_target_marker` |
| `mock_gripper.py` | `control/gripper_target`、`control/joint_states`、`feedback/gripper_status`、`gripper_controller/follow_joint_trajectory` |

**TF 帧参数化**（不写死）：`base_frame` / `end_effector_link` / `camera_frame`（shelf_workflow）、`base_link` / `end_effector_link`（grasp_executor）、`base_frame`（yolo_grasp / place_planner）均为已 `declare_parameter` 的参数，带 namespace 启动时由脚本 `-p` 传 `arm/` 前缀即可，**节点代码零改动**。

### 第 3 层：启动脚本（新增，不碰现有）

新增 `start_shelf_realcam_armns.sh`（复制 realcam 版改造）：

- MoveIt launch 加 `namespace:=arm`；
- 每个视觉节点加 `--ros-args -r __ns:=/arm` + `-p base_frame:=arm/base_link -p end_effector_link:=arm/tcp_link`（帧名按实际 TF 树填；`camera_frame` 视相机是否在臂树上定）；
- 现有 `start_shelf.sh` / `start_shelf_sim.sh` / `start_shelf_realcam.sh` / `start_pc.sh` 等**全部保持原样**。

---

## 5. 话题分组决策表

| 话题 | 归属 | 原因 |
|------|------|------|
| `task_command`、`grasp_pose`、`yolo/*`、`place_pose`、`manual_*`、`grasp_executor_state`、`clear_octomap`、`control/*`、`feedback/*` 等 | **组内相对化** | 臂栈内部闭环 |
| `/camera/camera/...`（3 个） | **保持绝对** | RealSense 在全局启动，进组会断链 |
| `/aruco_detections` | **保持绝对** | aruco_tracker 外部包在全局 |
| 未来车臂通信（如 `/arm/task_command` 触发、状态回报） | **绝对桥接** | 跨组唯一通道 |

---

## 6. 兼容性论证

1. **默认启动**（无 namespace / 无 `__ns`）：所有相对话题在根空间解析为现状的绝对话题；`frame_prefix` 为空；`octomap_frame` 回退 `base_link`；点云 topic 解析不变 → **行为与现状一致**；
2. **现有脚本零修改**；车侧工作区零接触；
3. 唯一"静默变化"风险点：`sensors_3d.yaml` 的 `point_cloud_topic` 从绝对改相对——已核对根空间解析结果相同（`filtered_cloud_topic` 本来就是相对写法），可放心。

---

## 7. 验证清单

1. 编译：`colcon build --packages-select agx_arm_moveit agx_arm_vision`（编译验证由 AI 负责）；
2. **默认回归**（用户执行）：跑一遍现有 `start_shelf_realcam.sh`，确认与改动前行为一致；
3. **namespace 验证**（用户执行）：`start_shelf_realcam_armns.sh` 启动后——
   - `ros2 topic list | grep /arm/` 话题均带 `/arm/` 前缀；
   - `ros2 topic info /arm/robot_description` 存在且唯一；
   - `ros2 run tf2_tools view_frames` 无 `base_link` 冲突；
4. **车臂并行**（用户执行）：车（chassis_bridge）照常启动，两边互不干扰，RViz 可同时显示两套。

---

## 8. 实施顺序与提交边界

1. **提交 1（launch 层）**：第 1 层 4-5 个文件 → 编译 → 用户回归验证默认行为；
2. **提交 2（视觉层）**：第 2 层 7 个文件 → 编译 → 用户回归验证；
3. **提交 3（脚本）**：新增 `start_shelf_realcam_armns.sh` → 用户验证 namespace 场景。

每步验收后再进下一步，随时可停。

---

## 9. 待确认事项

- [ ] 命名空间名用 `arm`（即 `/arm/...`）是否 OK？
- [ ] realcam 场景相机是装在臂上（帧带 `arm/` 前缀）还是独立固定（全局帧）？——决定 `camera_frame` 参数怎么传
- [ ] 纯仿真的 `virtual_*` 节点是否进组？（建议**不动**，反正不与车并行）
- [ ] 三个提交的边界和顺序是否 OK？

---

## 附录 A：已核实的现状证据（2026-08）

- 机械臂 `demo.launch.py` 已内置 namespace 框架：`namespace` 参数（默认空）、`PushRosNamespace`、`SetRemap(/robot_description→robot_description)`、控制器 YAML 命名空间化、RViz Move Group Namespace 生成；
- `agx_arm_ctrl/start_single_agx_arm_moveit.launch.py` 已透传 namespace；
- **frame_prefix 目前只加在 `rsp.launch.py` 一处**（未提交改动），`move_group.launch.py`（`octomap_frame: "base_link"`）与 `static_virtual_joint_tfs.launch.py`（utils 生成，不读 frame_prefix）未配套 → 直接 `namespace:=arm` 会 TF 断链；
- 视觉节点话题全部为绝对路径（grasp_executor 16 个、shelf_workflow 9 个等），**光加 `__ns` 无效**，必须"话题改相对 + `__ns`"成组改；
- `moveit2_local.py` 的 `/move_action`、`/apply_planning_scene`、`/compute_cartesian_path`、`/arm_controller/follow_joint_trajectory` 为绝对路径，MoveIt 进 `/arm` 后需同步相对化；
- `sensors_3d.yaml`：`point_cloud_topic` 绝对、`filtered_cloud_topic` 相对；
- 底盘工作区 `/home/s1/ros2_ws_1/ros2_ws`：`chassis_bridge` 发布 `/odom`、`/imu/data`、`/wheel_odometry`（C++ 节点，绝对话题），EKF 配置硬编码根帧；
- 视觉节点 TF 帧参数（`base_frame` / `end_effector_link` / `camera_frame`）均已 declare_parameter，可启动时 `-p` 覆盖。
