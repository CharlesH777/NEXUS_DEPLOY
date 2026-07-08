# nexus_fastlio

> 专有授权 (All Rights Reserved), Copyright © 2026 Charles. 见根目录 [LICENSE](../../LICENSE).

FAST-LIO2 真机部署适配节点。本包**不包含** FAST-LIO2 本体（`lio_node` 二进制在外部
`third_party/FASTLIO2_ROS2` 或用户指定路径中），只提供 Livox 输入适配和 FAST-LIO2
里程计桥接。

## 节点

| 可执行文件 | 语言 | 作用 |
|---|---|---|
| `fastlio_lidar_adapter` | Python | Livox CustomMsg → FAST-LIO2 雷达输入（旋转 pitch、改 frame_id） |
| `fastlio_imu_adapter` | Python | IMU → FAST-LIO2 IMU 输入（加速度缩放、旋转 pitch、改 frame_id） |
| `fastlio_odom_bridge` | Python | `/fastlio2/lio_odom` → `/odom`、`/pose` 和 TF |

## Topics

### fastlio_lidar_adapter

| Topic | 方向 | 类型 | 说明 |
|---|---|---|---|
| `/livox/lidar` | 输入 | `livox_ros_driver2/CustomMsg` | 真机 Livox 原始数据 |
| `/lidar_fastlio` | 输出 | `livox_ros_driver2/CustomMsg` | FAST-LIO2 雷达输入 |

### fastlio_imu_adapter

| Topic | 方向 | 类型 | 说明 |
|---|---|---|---|
| `/livox/imu` | 输入 | `sensor_msgs/Imu` | 真机 Livox IMU |
| `/imu_fastlio` | 输出 | `sensor_msgs/Imu` | FAST-LIO2 IMU 输入 |

### fastlio_odom_bridge

| Topic | 方向 | 类型 | 说明 |
|---|---|---|---|
| `/fastlio2/lio_odom` | 输入 | `nav_msgs/Odometry` | FAST-LIO2 里程计输出 |
| `/odom` | 输出 | `nav_msgs/Odometry` | 部署栈统一里程计输入 |
| `/pose` | 输出 | `geometry_msgs/PoseStamped` | MPPI/探索使用的世界位姿 |

## 配置

| 文件 | 说明 |
|---|---|
| `config/fastlio2_real.yaml` | FAST-LIO2 `lio_node` 的真机运行参数（雷达范围、体素、IESKF、IMU 噪声、外参） |

## 构建

```bash
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1

colcon build --packages-select nexus_fastlio --symlink-install
```

依赖 `livox_ros_driver2` 的 `CustomMsg` 消息定义。

## 运行

适配节点由根目录 `scripts/run_real_robot.sh` 在 `NEXUS_ENABLE_FASTLIO2=true` 时自动拉起。
FAST-LIO2 `lio_node` 二进制路径由 `NEXUS_FASTLIO2_BIN` 指定。

```bash
NEXUS_FASTLIO2_BIN=/path/to/lio_node bash scripts/run_real_robot.sh
```

单独启动适配节点：

```bash
source install/setup.bash
ros2 run nexus_fastlio fastlio_lidar_adapter
ros2 run nexus_fastlio fastlio_imu_adapter
ros2 run nexus_fastlio fastlio_odom_bridge
```

## 关键参数

| 参数 | 默认 | 说明 |
|---|---:|---|
| `input_topic` | `/livox/lidar` / `/livox/imu` | 输入 topic |
| `output_topic` | `/lidar_fastlio` / `/imu_fastlio` | 输出 topic |
| `rotation_pitch_deg` | `0.0` | 雷达 / IMU 俯仰旋转角 |
| `linear_accel_scale` | `1.0` | IMU 加速度缩放 |
| `target_frame_id` | `base_link` | 输出 frame_id |
