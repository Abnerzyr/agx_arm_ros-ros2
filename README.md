# AGX NERO 机械臂 YOLO 视觉抓取系统（ROS2）

本仓库是 AgileX NERO 七轴机械臂 + AGX 夹爪 + RealSense 深度相机的 **YOLO 视觉抓取**完整实现，基于 ROS2（Humble / Jazzy）与 MoveIt2。

|ROS |STATE|
|---|---|
|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|
|![jazzy](https://img.shields.io/badge/ros-jazzy-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

## 功能概述

系统实现"**视觉识别 → 抓取规划 → 运动执行 → 力反馈夹取**"的闭环抓取流程：

1. **目标检测**：YOLOv8s-worldv2 检测目标物体（默认：魔方），支持自定义类别（需 CLIP）
2. **抓取点估计**：GR-ConvNet 在 RGB-D 裁剪块上预测抓取位置、角度与开口宽度
3. **桌面剔除**：RANSAC 拟合桌平面并排除桌面点，避免在桌面上抓取
4. **3D 包围盒**：由深度点云拟合目标包围盒，作为规划场景中的碰撞物体
5. **运动规划**：MoveIt2 先 `plan_only` 验证可达性，再执行运动（速度 10%）
6. **夹取与回位**：力反馈夹爪闭合（力阈值 1.5 N），完成后回 home 等待人工放物

> 抓取动作需人工通过话题触发（见 [手动控制](#手动控制)），确保安全。

## 系统架构

```
RealSense 相机（深度对齐彩色）
  ├─ /camera/camera/color/image_raw               （彩色图）
  ├─ /camera/camera/aligned_depth_to_color/image_raw （深度图）
  └─ /camera/camera/color/camera_info             （相机内参）
                    │
                    ▼
┌──────────────────────────────────────┐
│ yolo_grasp（YOLO + GR-ConvNet）       │
│  检测 → 裁剪 → 抓取点估计 → 3D 包围盒   │
└───────┬──────────────┬───────────────┘
        │              │
        ▼              ▼
  /grasp_pose     /yolo/target_box
  (PoseStamped)   (Marker 碰撞盒)
        │              │
        └──────┬───────┘
               ▼
┌──────────────────────────────────────┐
│ grasp_executor（抓取执行状态机）        │
│  IDLE→验证→执行→夹取→回home→等待放物   │
└───────────────┬──────────────────────┘
                │ MoveGroup action（/move_action）
                ▼
      MoveIt2（move_group） + agx_arm_ctrl（CAN 驱动） + AGX 夹爪
```

点云链路：`yolo_grasp` 发布 `/yolo/points_filtered`（剔除目标盒区域、重建桌面平面）→ move_group 的 octomap 更新器订阅并输出 `/move_group/filtered_cloud` → `grasp_executor` 据此判断规划场景是否已重建完成。

## 目录结构

```
src/
├── agx_arm_ctrl        # 机械臂 ROS2 驱动（CAN 通信）+ launch
├── agx_arm_description # URDF 模型
├── agx_arm_moveit      # MoveIt2 配置（含 octomap 传感器、demo launch）
├── agx_arm_msgs        # 自定义消息（GripperStatus 等）
└── agx_arm_vision      # 视觉抓取节点（yolo_grasp / grasp_executor 等）
start_arm_yolo.sh       # YOLO 抓取一键启动脚本
start_arm_yolo_tilt.sh  # 增加相机倾斜标定的启动脚本
docs/                   # 文档（CAN 使用、TCP 偏移、Q&A）
```

## 快速开始

### 1. 安装 Python SDK

```bash
git clone https://github.com/agilexrobotics/pyAgxArm.git
cd pyAgxArm
# Jazzy
pip3 install . --break-system-packages
# Humble
pip3 install .
```

### 2. 安装依赖

```bash
cd <工作空间>/scripts
bash ./agx_arm_install_deps.sh
```

脚本会安装 Python 依赖（python-can、scipy、numpy）、CAN 工具（can-utils、ethtool）以及 ROS2 依赖（ros2-control、moveit 等）。若系统区域设置非英文，需设置：

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

### 3. 编译

```bash
cd <工作空间>
colcon build
source install/setup.bash
```

### 4. 启动 YOLO 抓取

```bash
cd <工作空间>
bash start_arm_yolo.sh
```

脚本自动完成：

1. 激活 can0/can1（波特率 1 Mbps），用 pyAgxArm 自动探测机械臂所在 CAN 口
2. 清理旧进程，启动 MoveIt + 机械臂（`arm_type:=nero`、`effector_type:=agx_gripper`、速度 10%、自动回 home）
3. 启动 RViz（`src/config/yolo_config.rviz`）与 RealSense 相机
4. 启动 `yolo_grasp`、`grasp_target_marker`、`grasp_executor` 三个节点

若相机相对桌面存在倾斜，使用带标定的版本（启动时先测量约 15 秒，需保证桌面在视野内）：

```bash
bash start_arm_yolo_tilt.sh
```

> 注意：启动脚本内含有硬编码的绝对路径（`/home/s1/tiaozhanbei/agx_arm_ros-ros2`），迁移到其他路径时需同步修改。

### 手动控制

```bash
# 触发抓取（对当前存储的最新目标位姿执行抓取）
ros2 topic pub --once -w 1 /manual_grasp_start std_msgs/msg/Empty '{}'

# 完成夹取并回 home 后，确认放物（夹爪张开）
ros2 topic pub --once -w 1 /manual_release std_msgs/msg/Empty '{}'

# 手动回 home
ros2 service call /move_home std_srvs/srv/Empty
```

## 节点说明

### yolo_grasp（视觉抓取点估计）

入口：`ros2 run agx_arm_vision yolo_grasp`，默认每 0.5 s 处理一帧。

**处理流程：**

1. YOLO 检测；无目标或置信度低于阈值时仅发布全景点云
2. 选取"最近"检测框（框内深度中值最小者）作为抓取目标
3. 以检测框为中心裁剪方形 RGB-D patch（复制边界填充，供 GR-ConvNet 使用）
4. RANSAC 拟合桌平面并剔除桌面点；对剩余有效区域做距离变换过滤
5. 归一化后送入 GR-ConvNet，得到抓取质量图、角度与开口宽度
6. 抓取点分级回退：严格掩码 → 放宽掩码 → 全局最大质量 → 检测框中心
7. 由框内点云拟合目标 3D 包围盒（尺寸 + 朝向，含 `box_padding` 膨胀）
8. 将抓取位姿变换到 `base_link` 后发布

**参数：**

| 参数 | 默认值 | 说明 |
|---|---|---|
| `base_frame` | `base_link` | 抓取位姿输出坐标系 |
| `rgb_topic` | `/camera/camera/color/image_raw` | 彩色图像话题 |
| `depth_topic` | `/camera/camera/aligned_depth_to_color/image_raw` | 对齐深度话题 |
| `info_topic` | `/camera/camera/color/camera_info` | 相机内参话题 |
| `confidence_threshold` | `0.25` | YOLO 置信度阈值 |
| `grasp_quality_threshold` | `0.3` | 抓取质量阈值 |
| `input_size` | `224` | GR-ConvNet 输入尺寸 |
| `box_padding` | `0.005` | 目标包围盒各向膨胀量 (m) |
| `target_classes` | `["rubik's cube"]` | 目标类别；自定义类别需安装 CLIP 并调用 `set_classes` |

**话题：**

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/camera/camera/color/image_raw` | `Image` | 彩色图 |
| 订阅 | `/camera/camera/aligned_depth_to_color/image_raw` | `Image` | 深度图（mm） |
| 订阅 | `/camera/camera/color/camera_info` | `CameraInfo` | 相机内参 |
| 订阅 | `/map_update_enable` | `Bool` | 是否允许发布点云（由 executor 控制） |
| 发布 | `/grasp_pose` | `PoseStamped` | 抓取位姿（base_link 系） |
| 发布 | `/yolo/target_box` | `Marker` | 目标 3D 包围盒（CUBE） |
| 发布 | `/yolo/detections` | `Image` | 检测结果可视化 |
| 发布 | `/yolo/crop_rgb` | `Image` | 输入网络的裁剪块 |
| 发布 | `/yolo/quality_map` | `Image` | 抓取质量图（JET 伪彩） |
| 发布 | `/yolo/points` | `PointCloud2` | 全景点云 |
| 发布 | `/yolo/points_filtered` | `PointCloud2` | 剔除目标盒区域、重建桌面的点云（供 octomap） |

### grasp_executor（抓取执行状态机）

入口：`ros2 run agx_arm_vision grasp_executor`，每 0.1 s 执行一次状态机 tick。

**状态机流程：**

```
IDLE ── 收到 /grasp_pose（2s 内与目标盒配对）──► 存储位姿，等待 /manual_grasp_start
  │
  ├─ 触发后：目标盒加入规划场景 → 清空 octomap（/clear_octomap）
  │           → 等待点云重建 → 关节稳定 → plan_only 验证可达性
  ├─ 可达 → MOVE_TO_TARGET（MoveIt 执行，速度 0.1）
  ├─ WAIT_REACH：TCP 误差 ≤ 0.03 m 连续 3 次 → CLOSE_GRIPPER
  ├─ CLOSE_GRIPPER：夹爪闭合（力 > 1.5 N 视为夹住）
  ├─ MOVE_HOME：移除目标盒 → 回 home → 进入 WAIT_RELEASE
  └─ WAIT_RELEASE：收到 /manual_release → 夹爪张开 → IDLE
```

**关键机制：**

- **位姿与目标盒配对**：`/grasp_pose` 需在 2 s 窗口内与 `/yolo/target_box` 配对，否则拒绝该次抓取（`require_target_box:=true` 时）
- **octomap 重建确认**：清空后等待 `filtered_cloud` 新数据（`insert_settle` 0.3 s），超时则继续规划
- **点云门控**：空闲时发布 `/map_update_enable=true`，运动期间置 `false`，避免运动中的点云干扰规划
- **碰撞盒管理**：目标盒在规划阶段加入、回 home 前移除，防止与目标物碰撞
- **TIMING 日志**：记录 clear / rebuild / validate / plan / exec / gripper / home 各阶段耗时，便于性能分析
- 空闲时每 10 s 清一次 octomap，防止残留点云导致规划失败

**参数：**

| 参数 | 默认值 | 说明 |
|---|---|---|
| `end_effector_link` | `tcp_link` | 末端执行器坐标系 |
| `arm_group` | `arm` | MoveIt 规划组 |
| `gripper_joint` | `gripper` | 夹爪关节名 |
| `gripper_open` / `gripper_closed` | `0.1` / `0.0` | 夹爪开 / 合宽度 (m) |
| `force_threshold` | `1.5` | 夹持力阈值 (N) |
| `reach_tolerance` | `0.03` | TCP 到位误差 (m) |
| `reach_timeout` | `45.0` | 到位等待超时 (s) |
| `velocity_scaling` | `0.1` | MoveIt 执行速度比例 |
| `target_z_offset` | `0.0` | 目标位姿 Z 方向偏移 |
| `require_target_box` | `true` | 是否必须与目标盒配对 |
| `box_wait_timeout` | `1.0` | 等待目标盒配对超时 (s) |
| `idle_clear_interval` | `10.0` | 空闲清 octomap 间隔 (s) |
| `insert_settle` | `0.3` | octomap 重建确认时间 (s) |
| `rebuild_timeout` | `2.5` | 重建等待超时 (s) |
| `filtered_cloud_topic` | `/move_group/filtered_cloud` | octomap 输出话题（重建判定） |
| `constrain_orientation` | `true` | 是否约束末端姿态 |
| `home_joints` | `[-1.751, -0.342, 1.656, 1.036, 0.360, 0.074, 1.570]` | home 关节角 (rad) |

**话题 / 服务 / 动作：**

| 方向 | 名称 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/grasp_pose` | `PoseStamped` | 抓取目标位姿 |
| 订阅 | `/yolo/target_box` | `Marker` | 目标盒 |
| 订阅 | `/yolo/points_filtered` | `PointCloud2` | 点云时间戳（重建判定） |
| 订阅 | `/move_group/filtered_cloud` | `PointCloud2` | octomap 输出 |
| 订阅 | `/manual_grasp_start` / `/manual_release` | `Empty` | 手动触发抓取 / 放物 |
| 订阅 | `/feedback/gripper_status` | `GripperStatus` | 夹爪宽度 / 力反馈 |
| 订阅 | `/feedback/joint_states` | `JointState` | 关节反馈（稳定性判定） |
| 发布 | `/map_update_enable` | `Bool` | 点云发布门控 |
| 发布 | `/control/gripper_target` | `JointState` | 夹爪目标宽度 |
| 调用 | `/clear_octomap` | `Empty` | 清空 octomap |
| 调用 | `/apply_planning_scene` | `ApplyPlanningScene` | 规划场景增删目标盒 |
| 动作 | `/move_action` | `MoveGroup` | MoveIt 规划 / 执行 |

### grasp_target_marker（可视化）

订阅 `/grasp_pose`，在 RViz 中发布抓取点球体与朝向箭头（话题 `/grasp_target_marker`），用于确认抓取目标位置。

### tilt_measure（相机倾斜标定）

`start_arm_yolo_tilt.sh` 专用：采集 N 帧（默认 30）深度图，RANSAC 拟合桌平面，计算相机相对 base 水平面的倾斜角，并输出修正建议日志。

## MoveIt 集成

- 传感器配置（`src/agx_arm_moveit/config/sensors_3d.yaml`）：octomap 更新器订阅 `/yolo/points_filtered`，输出 `/move_group/filtered_cloud`，最大范围 5 m、最大更新率 1 Hz
- 规划场景：目标盒通过 `/apply_planning_scene` 服务加入 / 移除
- 控制门控：`auto_control_gate:=true` 时，仅在 MoveIt 执行阶段开放 `/control/*` 控制门

## 调试

```bash
# 查看各节点日志（启动脚本重定向位置）
tail -f /tmp/yolo.log /tmp/grasp.log /tmp/marker.log

# RViz 中可订阅的可视化话题
#   /yolo/detections、/yolo/crop_rgb、/yolo/quality_map、
#   /yolo/points_filtered、/yolo/target_box、/grasp_target_marker

# 查看话题与抓取位姿
ros2 topic list
ros2 topic echo /grasp_pose
```

抓取执行过程中，`grasp_executor` 会输出 `[TIMING]` 日志（如 `clear_srv=0.50s rebuild=1.20s exec_plan=8.30s`），可据此定位各阶段耗时。

## 常见问题

| 现象 | 原因 / 解决 |
|---|---|
| 找不到机械臂 CAN 口 | 检查 CAN 接线；`ip -details link show canX` 查看是否 BUS-OFF / NO-CARRIER |
| `TF lookup failed` | 检查 `/tf` 是否正常发布 `camera_color_optical_frame → base_link` 变换 |
| 抓取被拒绝（无配对目标盒） | `/yolo/target_box` 未在 2 s 窗口内发布；检查 yolo_grasp 日志与话题 |
| 自定义目标类别不生效 | 需安装 CLIP：`pip3 install git+https://github.com/ultralytics/CLIP.git` |
| 抓取位姿偏斜 | 相机安装倾斜导致，改用 `start_arm_yolo_tilt.sh` 标定 |
| MoveIt 规划失败 | 查看 `/tmp/grasp.log` 中的错误码与 TIMING 日志；确认点云、目标盒、octomap 正常 |

## 相关文档

| 说明 | 文档 |
|---|---|
| CAN 模块使用 | [docs/CAN_USER.md](./docs/CAN_USER.md) |
| TCP 偏移设置 | [docs/tcp_offset/TCP_OFFSET.md](./docs/tcp_offset/TCP_OFFSET.md) |
| 常见问题 | [docs/Q&A.md](./docs/Q&A.md) |
| 官方 SDK | [pyAgxArm](https://github.com/agilexrobotics/pyAgxArm) |

## 安全注意事项

- **保持安全距离**：机械臂运动时，请勿进入其工作空间
- 抓取为手动触发，触发前请确认目标与环境安全（`/manual_grasp_start`）
- 速度已限制为 10%（`velocity_scaling=0.1`），调整时务必谨慎
- 靠近运动学奇异点时，关节可能发生突然大幅运动
