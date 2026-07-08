# nexus_elevation_mppi

> 专有授权 (All Rights Reserved), Copyright © 2026 Charles. 见根目录 [LICENSE](../../LICENSE).

NEXUS 仿真主线的感知 + 探索 + 局部规划包。从 CuPy 高程图出发，计算高度差通行性地图，
驱动 frontier 自动探索，并用 C++ MPPI 做局部轨迹优化。

## 节点

| 可执行文件 | 语言 | 作用 |
|---|---|---|
| `mppi_navigator` | C++ | MPPI 局部轨迹优化，采样 batch 轨迹选最优，发布速度指令和最优路径 |
| `traversability_to_map` | Python | 订阅 GridMap elevation 层，按局部高度差 max-min 生成 OccupancyGrid 通行图 |
| `elevation_map_exporter` | Python | 订阅 GridMap，解码 circular buffer，落盘 npz/pgm/json 用于离线分析 |
| `novelty_explorer` | Python | 高度差通行图 → 模拟 radar raycast → novelty/frontier → Dijkstra → goal/path |

## Topics

### mppi_navigator

| Topic | 方向 | 类型 |
|---|---|---|
| `/elevation_mapping_node/elevation_map` | 输入 | `grid_map_msgs/GridMap` |
| `/traversability_map` | 输入 | `nav_msgs/OccupancyGrid` |
| `/nav_odom` | 输入 | `nav_msgs/Odometry` |
| `/cube_robot/world_pose` | 输入 | `geometry_msgs/PoseStamped` |
| `/goal_pose` | 输入 | `geometry_msgs/PoseStamped` |
| `/mppi/reference_path` | 输入 | `nav_msgs/Path` |
| `/mppi/cmd_vel_raw` | 输出 | `geometry_msgs/Twist` |
| `/mppi/optimal_path` | 输出 | `nav_msgs/Path` |
| `/mppi/reference_path_debug` | 输出 | `nav_msgs/Path` |
| `/mppi/terrain_cost_map` | 输出 | `nav_msgs/OccupancyGrid` |

### traversability_to_map

| Topic | 方向 | 类型 |
|---|---|---|
| `/elevation_mapping_node/elevation_map` | 输入 | `grid_map_msgs/GridMap` |
| `/traversability_map` | 输出 | `nav_msgs/OccupancyGrid` |

### novelty_explorer

| Topic | 方向 | 类型 |
|---|---|---|
| `/traversability_map` | 输入 | `nav_msgs/OccupancyGrid` |
| `/nav_odom` | 输入 | `nav_msgs/Odometry` |
| `/cube_robot/world_pose` | 输入 | `geometry_msgs/PoseStamped` |
| `/goal_pose` | 输出 | `geometry_msgs/PoseStamped` |
| `/mppi/reference_path` | 输出 | `nav_msgs/Path` |
| `/novelty_explorer/radar_known` | 输出 | `nav_msgs/OccupancyGrid` |
| `/novelty_explorer/novelty_map` | 输出 | `nav_msgs/OccupancyGrid` |

## 配置

| 文件 | 说明 |
|---|---|
| `config/nexus_mppi_controller.yaml` | MPPI 节点参数（采样数、运动约束、代价权重、足迹） |
| `config/nexus_elevation_mapping.rviz` | RViz 预设（通行图 / novelty / MPPI 路径 / 地形代价） |

全栈默认参数在仓库根 `config/nexus_navigation_stack.yaml`，各节点也可单独指定配置文件。

## 构建

```bash
source /opt/ros/humble/setup.bash
source ~/NEXUS/tools/elevation_mapping_cupy_ros2_ws/install/setup.bash
export PYTHONNOUSERSITE=1

colcon build --packages-select nexus_elevation_mppi --symlink-install
```

依赖 `grid_map_msgs`（来自外部 CuPy 高程图工作区），构建前需先 source 该工作区。

## 运行

```bash
source install/setup.bash

# 单独启动各节点
ros2 run nexus_elevation_mppi mppi_navigator
ros2 run nexus_elevation_mppi traversability_to_map
ros2 run nexus_elevation_mppi elevation_map_exporter
ros2 run nexus_elevation_mppi novelty_explorer
```

完整链路启动（含 Gazebo + CuPy 高程图 + MPPI + 探索 + sand MPC）：

```bash
bash scripts/run_mppi.sh
```

## 关键参数

MPPI 默认 20 Hz 控制，batch=1000，time_steps=56，model_dt=0.05。
`fail_on_all_collision: false` 在所有采样轨迹都碰障碍时不硬停车，而是用最低代价轨迹软 fallback。
`unknown_is_obstacle: false` 未知区域不直接当障碍。

完整参数见 `config/nexus_mppi_controller.yaml` 和根目录 [README.md](../../README.md) 的「算法 3」一节。

## 相关文档

- [README.md](../../README.md) — 全链路说明、算法细节、调参入口、排障
- [SCRIPTS.md](../../SCRIPTS.md) — `run_mppi.sh` / `run_elevation_mppi.sh` 脚本详解
- [docs/ARCHITECTURE.md](../../docs/ARCHITECTURE.md) — 系统架构和数据流
