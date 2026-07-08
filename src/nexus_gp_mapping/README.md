# nexus_gp_mapping

> 专有授权 (All Rights Reserved), Copyright © 2026 Charles. 见根目录 [LICENSE](../../LICENSE).

基于高斯过程 (Gaussian Process) 的地形建图节点。用 gpytorch 稀疏 GP（inducing point kernel）
从点云拟合地形高度场，作为 CuPy 高程图之外的备选建图链路。

当前属于**可选旧链路**，不是默认主线。默认探索链路用的是 CuPy 高程图 + 高度差通行图。

## 节点

| 可执行文件 | 语言 | 作用 |
|---|---|---|
| `gp_mapping_node` | Python | 从累积点云拟合稀疏 GP 地形图，发布 GridMap |

## Topics

| Topic | 方向 | 类型 | 说明 |
|---|---|---|---|
| `/cloud_registered_accum` | 输入 | `sensor_msgs/PointCloud2` | 累积世界坐标系点云 |
| `/nav_odom` | 输入 | `nav_msgs/Odometry` | 机器人位姿 |
| GP 输出 | 输出 | `grid_map_msgs/GridMap` | GP 拟合的地形高度图 |

## 配置

| 文件 | 说明 |
|---|---|
| `config/nexus_gp_navigation.rviz` | GP 建图链路的 RViz 预设 |

## 构建

```bash
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1

colcon build --packages-select nexus_gp_mapping --symlink-install
```

Python 依赖（gpytorch + torch）：

```bash
/usr/bin/python3 -m pip install --user torch gpytorch
```

## 运行

GP 建图链路由以下脚本启动：

```bash
# FAST-LIO2 + GP 建图
bash scripts/run_gp_fastlio.sh

# 仅 GP 建图（无 FAST-LIO2，输入累积点云）
bash scripts/run_gp_nav.sh
```

通过 `MAP_SIM_ENABLE_DEFAULT_STACK=1` 在 `scripts/run_sim.sh` 中分发到 GP 链路。

## 模型

`SparseGPModel`（gpytorch `ExactGP`）：

- 均值：`LinearMean(input_size=3)` — 吸收大尺度坡度，核只学残差粗糙度
- 核：`ScaleKernel(RQKernel)` + `InducingPointKernel` — 稀疏 inducing point 加速
- 初始 lengthscale 较大，避免首轮迭代过度平滑陡峭地形

关键 GP 参数（通过 `MAP_SIM_GP_*` 环境变量传入）包括：分辨率、inducing points 数量、
鲁棒拟合迭代次数、地面种子格大小、浮点拒绝余量等。

## 相关文档

- [README.md](../../README.md) — 系统概述、GP 链路和默认链路的关系
- [SCRIPTS.md](../../SCRIPTS.md) — `run_gp_fastlio.sh` / `run_gp_nav.sh` 详解、`MAP_SIM_GP_*` 参数
