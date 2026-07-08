# nexus_teleop

> 专有授权 (All Rights Reserved), Copyright © 2026 Charles. 见根目录 [LICENSE](../../LICENSE).

遥控和位姿/TF 桥接工具包。提供手柄/键盘遥控 GUI（带实时参数调节）和 PoseStamped → TF 广播桥。

## 节点

| 可执行文件 | 语言 | 作用 |
|---|---|---|
| `teleop_gui` | Python | tkinter 遥控 GUI：发 `/cmd_vel`，并实时调节通行图/MPPI/探索参数 |
| `pose_to_tf_bridge` | Python | 订阅 PoseStamped，广播 TF（world → base_link 等） |

## Topics

### teleop_gui

| Topic | 方向 | 类型 | 说明 |
|---|---|---|---|
| `/cmd_vel` | 输出 | `geometry_msgs/Twist` | 遥控速度指令 |

GUI 还会通过 `GetParameters` / `SetParameters` 服务动态读写以下节点的参数：
`/traversability_to_map`、`/mppi_navigator`、`/novelty_explorer`。

### pose_to_tf_bridge

| Topic | 方向 | 类型 | 说明 |
|---|---|---|---|
| `/cube_robot/world_pose` | 输入 | `geometry_msgs/PoseStamped` | Gazebo 真值位姿 |
| TF: `world` → `base_link` | 输出 | `tf2_msgs/TFMessage` | 广播的 TF |

## 构建

```bash
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1

colcon build --packages-select nexus_teleop --symlink-install
```

GUI 依赖 `python3-tk`（系统包）和 `PyYAML`。

## 运行

```bash
source install/setup.bash

# 遥控 GUI（需要 DISPLAY）
ros2 run nexus_teleop teleop_gui

# 位姿 → TF 桥
ros2 run nexus_teleop pose_to_tf_bridge
```

便捷入口（自动设 DISPLAY 和参数）：

```bash
bash scripts/open_teleop.sh
```

## 关键参数

### teleop_gui

| 参数 | 默认 | 说明 |
|---|---:|---|
| 线速度 | — | 滑块调节 |
| 横移速度 | — | 滑块调节 |
| 角速度 | — | 滑块调节 |
| 发布频率 | — | cmd_vel 发布频率 |

GUI 内的参数调节面板覆盖通行图 kernel/threshold、MPPI 采样/代价权重、探索 frontier/radar 等常用项。

### pose_to_tf_bridge

| 参数 | 默认 | 说明 |
|---|---|---|
| `pose_topic` | `/cube_robot/world_pose` | 输入位姿 topic |
| `parent_frame` | `world` | TF 父坐标系 |
| `child_frame` | `base_link` | TF 子坐标系 |
| `offset_x/y/z` | `0.0` | 位姿偏移补偿 |

## 相关文档

- [README.md](../../README.md) — 系统概述、手动发目标、CLI 调试
- [SCRIPTS.md](../../SCRIPTS.md) — `open_teleop.sh` 详解
