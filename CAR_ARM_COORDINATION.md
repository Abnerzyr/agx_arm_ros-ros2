# 车臂协同：两套常驻 + 相机按需开关的内存优化方案

## 关键新信息

这是 **Jetson Orin**（6 核 ARM + 集成 GPU + 统一内存，7.4G RAM + 11G swap），yolo 用 ultralytics（很可能走 CUDA）、grconv 显式 `map_location='cpu'`。统一内存意味着 GPU 显存和系统内存是一个池子，torch 上 GPU 也直接吃同一块内存。

下面是"两套常驻、只开关相机"的优化方案。

## 0. 先提一个绕不开的前提

两套同时常驻 = 回到 `base_link`/`/robot_description` 冲突。车树 `base_footprint→base_link`、臂树 `world→base_link` 都常驻发布，不隔离会直接错乱。所以这个方案**必须以实施 `ARM_NAMESPACE_PLAN.md`（臂进 `/arm`）为前提**——这不是可选，是"两套常驻"的必要条件。2 分钟启动成本只付一次，换来的是后续只做秒级切换。

## 1. 核心思路：把"常驻集"和"可切换集"分开

内存大头不在 MoveIt/Nav2，而在**相机 + torch 模型 + RViz**。所以常驻保留便宜的，只切换贵的：

| 层级 | 内容 | 内存量级 | 何时 |
|------|------|----------|------|
| **常驻** | 车栈（chassis/IMU/EKF/Nav2/livox） | 几百 MB | 全程 |
| **常驻** | 臂栈 MoveIt + 控制器 + CAN（臂在 home 空闲） | 几百 MB | 全程 |
| **常驻** | grasp_executor / shelf_workflow / aruco / place_planner / coordinator | 空闲时几十 MB | 全程（无相机帧=纯 idle） |
| **开/关** | `realsense2_camera` | ~200–500 MB（流缓冲） | **停稳才开，抓完就关** |
| **开/关** | `yolo_grasp`（yolo+grconv torch 模型） | CPU 模型数百 MB / GPU 统一内存更多 | 抓取/放置段 |
| **不开** | RViz | 点云显示最吃 | 仅调试 |

切换只动 `start_vision.sh` / `stop_vision.sh`（相机 + yolo，秒级~几十秒），**不碰** MoveIt/Nav2/控制器 → 避免每次 2 分钟全量重启。相机一关，yolo/aruco/place/executor 因没有图像输入自动进入空闲。

## 2. 内存优化手段清单（保证流畅）

1. **相机参数收敛**（现在的 launch 是全默认全流 1280×720@30）：
   - 降 `640x480x15`（yolo 输入才 224×224）；
   - 关 `enable_imu/accel/gyro` 等无用流；保留 `publish_tf:=false`。
2. **Jetson 线程控制**：torch 默认抢占 6 核，推理会把 CPU 打满导致画面卡顿。设 `OMP_NUM_THREADS=2`、`torch.set_num_threads(2)`；YOLO 明确 `device=0`(CUDA)、grconv 留 1–2 线程。抓取间隙 `torch.cuda.empty_cache()` 防统一内存累积。
3. **空闲门控**：开车段由 coordinator 发 `/map_update_enable=False`（yolo/grasp_executor 已有该门控），配合相机关闭，确保开车段**零推理、零点云**。
4. **octomap 清理**：MoveIt 常驻但 octomap 无点云输入时为空；每次抓/放后调 `/clear_octomap` 兜底。
5. **QoS/缓冲防积压**：相机 best_effort + keep_last(5)，点云话题降频——避免慢订阅者把相机 DDS 缓冲填满导致"画面不更新"。
6. **看门狗**：监控 `image_raw` 帧时间戳新鲜度（>1.5s 判冻结 → 自动重启相机节点）+ `free` 空闲内存阈值预警。
7. **RViz 不开**（这是最容易吃内存的显示端）。
8. **`/dev/shm/fastrtps*` 清理** + 极低优先级的"每日/每 N 次任务软重启"兜底。

## 3. 协调器职责（轻量常驻）

- 状态机：开车 → 停稳(`reached`) → 发 `camera_on` + `cmd{layer}` → 抓完(`task_done`) → 发 `camera_off` → 开车 → … → 停稳 → `camera_on` + `place` → 放完(`task_done`) → `camera_off`；
- 只调用 `start_vision.sh`/`stop_vision.sh`（秒级），**不整组重启**；
- 兼做内存监控 + 相机看门狗。

## 4. 需要你确认

1. **yolo 现在走 GPU 还是 CPU？**（`ros2 run ... yolo_grasp` 启动日志里看 device）——决定线程/显存建议怎么写；
2. **开车段的可用内存目标**：预计相机+yolo 关掉后能回到可用 >3GB，这个水位够吗？
3. **yolo 关闭后的重启 20–40s 能否接受？**如果嫌慢，可退一步：**相机照关，yolo 保持常驻但空闲**（靠 `map_update_enable` 门控零推理）——省去重载模型时间，但模型内存释放不了，两种取舍你选哪种？
4. 确认后可以整理出 `start_vision.sh`/`stop_vision.sh`/coordinator/相机参数的具体改动清单。
