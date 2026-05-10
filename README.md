# DiffPhysDrone ROS2 仿真

基于 EGO-Planner 的 ROS2 四旋翼仿真环境，集成了 DiffPhysDrone 深度学习本地规划器，支持 Foxglove 实时可视化和闭环轨迹验证。

## 项目结构

```
simulation/
├── Dockerfile                 # ROS2 Jazzy + CUDA 12.8 + CycloneDDS
├── docker-compose.yaml        # GPU 透传、网络配置
├── launch.sh                  # 一键启动脚本
├── goal_and_validate.sh       # 目标发送 + 轨迹验证
├── ego-planner-swarm/         # 仿真器核心（修改版）
│   └── src/
│       ├── uav_simulator/     # 四旋翼动力学、深度渲染、SO3控制
│       └── planner/           # 地图生成、轨迹管理
└── diffphys_ws/               # ROS2 工作空间
    └── src/
        └── diffphys_local_planner/  # 自定义本地规划器
            ├── local_planner_node.py
            ├── model.py
            └── validator.py
```

## 快速开始

### 1. 环境要求

- NVIDIA GPU（CUDA 12.8+，计算能力 8.6+）
- Docker + NVIDIA Container Toolkit
- 本地已训练的 DiffPhysDrone checkpoint

### 2. 准备 checkpoint

将训练好的 checkpoint 放到 `DiffPhysDrone/output/checkpoint0004.pth`，或修改 `single_run_diffphys.launch.py` 中的路径：

```python
checkpoint = LaunchConfiguration('checkpoint', default='/workspace/DiffPhysDrone/output/checkpoint0004.pth')
```

### 3. 启动仿真

```bash
# 首次启动（构建镜像）
./launch.sh --rebuild

# 后续启动（跳过构建）
./launch.sh
```

启动后会自动打开 4 个 tmux 窗格：
- **Pane 0**: Foxglove Bridge（ws://localhost:8765）
- **Pane 1**: 仿真器 + 本地规划器
- **Pane 2**: 目标发送 + 轨迹验证

### 4. 查看可视化

在浏览器中打开 [Foxglove](https://app.foxglove.dev/)，连接 `ws://localhost:8765`，添加以下 topic：

| Topic | 用途 |
|-------|------|
| `/drone_0_visual_slam/odom` | 无人机位姿 |
| `/drone_0_pcl_render_node/depth` | 深度图 |
| `/colordepth` | 彩色深度图（近黑远白） |
| `/drone_0_global_path` | 规划路径 |
| `/map_generator/global_cloud` | 全局点云 |
| `/tf` | 坐标变换 |

### 5. 查看结果

验证完成后，结果保存在 `validation_result.json`：

```json
{
  "goal_reached": true,
  "cross_track_m": 0.83,
  "final_progress": 0.98,
  "avg_speed": 0.06,
  "trajectory_samples": [...]
}
```

## 如何接入自定义 Local Planner

### 核心接口

本地规划器需要实现以下 ROS2 接口：

**订阅：**
- `drone_0_visual_slam/odom` (`nav_msgs/Odometry`) — 无人机位姿
- `drone_0_pcl_render_node/depth` (`sensor_msgs/Image`) — 深度图（32FC1，640×480）
- `/move_base_simple/goal` (`geometry_msgs/PoseStamped`) — 目标点

**发布：**
- `drone_0_planning/pos_cmd` (`quadrotor_msgs/PositionCommand`) — 控制命令

### PositionCommand 格式

```python
cmd.position     # 目标位置 (x, y, z) — so3_control 会跟踪这个位置
cmd.velocity     # 目标速度 (vx, vy, vz)
cmd.acceleration # 加速度前馈 (ax, ay, az) — 无人机姿态由此决定
cmd.yaw          # 目标偏航角
cmd.yaw_dot      # 偏航角速度
```

**重要：** `cmd.position` 应设置为航点位置（不是当前 odometry 位置），否则 so3_control 的位置控制器不会干预，导致无人机无法跟踪路径。

### 深度图处理

深度图以 32FC1 格式发布，单位为米，分辨率为 640×480。模型输入需要：

```python
# 1. 调整尺寸到 48x64
d = F.interpolate(d, size=(48, 64), mode='bilinear', align_corners=False)

# 2. 归一化
d = 3.0 / d.clamp(0.3, 24) - 0.6

# 3. 最大池化 → 12x16
d = F.max_pool2d(d, 4, 4)
```

### 状态向量

模型输入的状态向量（10维）：

```python
local_v = R.T @ odom_vel          # 机体坐标系下的速度 (3)
local_target_v = R.T @ target_v   # 机体坐标系下的目标方向 (3)
r_z = R[:, 2]                     # 机体 Z 轴方向 (3)
margin = 0.2                      # 安全边距 (1)
```

其中 `R` 是从 odometry 四元数转换的旋转矩阵。

## 关键技术细节

### 坐标系

```
Body Frame:  X=前, Y=左, Z=上
Camera Frame (ROS): X=右, Y=下, Z=前
```

`cam02body` 设为 Identity（camera = body），渲染器内部使用 `body_to_optical` 转换到 ROS 光学坐标系。

### SO3 控制器

so3_control 根据 `PositionCommand` 计算期望力和姿态：

```
force = mg + kx*(pos_err) + kv*(vel_err) + mass*acc_feedforward
```

### 仿真参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 地图大小 | 26×20×3 m | |
| 障碍物 | 125 个圆环 | `obs_num=0, circle_num=125` |
| 无人机质量 | 0.98 kg | |
| 最大速度 | 2.0 m/s | |
| 感知范围 | 5.0 m | |
| 深度图分辨率 | 640×480 | |

## 测试过程中解决的问题

### 1. 深度图不发布

**问题：** `random_forest_sensing.cpp` 中有 `return` 提前退出，导致局部点云不发布。

**解决：** 移除提前 return，添加 TRANSIENT_LOCAL QoS。

### 2. TF 缺失导致 so3_control 不工作

**问题：** `odom_visualization.cpp` 不发布 `world→base0` TF。

**解决：** 强制始终发布 `world→base0` TF，启用 `tf45=True` 发布完整 TF 树。

### 3. 深度图使用 CUDA 但未启用

**问题：** `local_sensing/CMakeLists.txt` 未设置 `ENABLE_CUDA=true`。

**解决：** 添加 `ENABLE_CUDA=true` 和 `sm_86` 编译选项。

### 4. 无人机到达目标后剧烈震荡

**问题：** 目标附近 `goal_dir` 反复翻转 180°，yaw 限速（1.8°/步）导致绕圈。

**解决：**
- 增大到达阈值：0.5m → 1.5m
- 到达后锁定 yaw（不再追踪 goal_dir）
- yaw 限速放宽：1.8° → 9°/步

### 5. roll 运动过于剧烈

**问题：** 模型输出的侧向加速度导致大幅 roll，影响深度图质量。

**解决：** 在 SO3Control 中投影 body Y 轴到水平面，强制 roll=0，只保留 yaw 和 pitch。

## 常用命令

```bash
# 启动
./launch.sh

# 停止
tmux kill-session -t sim

# 重建单个包
docker exec diffphys_ros2 bash -c 'source /opt/ros/jazzy/setup.bash && \
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
  cd /workspace/simulation/diffphys_ws && \
  colcon build --symlink-install --packages-select diffphys_local_planner'

# 查看 topic
docker exec diffphys_ros2 bash -c 'source /opt/ros/jazzy/setup.bash && \
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && ros2 topic list'

# 查看日志
tmux capture-pane -t sim:launch.1 -p | tail -20
```
