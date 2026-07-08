# nexus_sand_mpc

> 专有授权 (All Rights Reserved), Copyright © 2026 Charles. 见根目录 [LICENSE](../../LICENSE).

ROS 2 版沙地滑移 (sand-slip) MPC 指令补偿器。从 `sand_sim` 迁移而来，放在 MPPI 后级
对速度指令做 slip 估计和延迟补偿，输出最终底盘 `/cmd_vel`。

注意：这是一个**库驱动的 do-mpc/CasADi MIMO MPC**，不是手写求解器。ROS 节点只负责
建模、延迟队列、观测回放和指令封装；优化求解委托给 `do-mpc`。

## 节点

| 可执行文件 | 语言 | 作用 |
|---|---|---|
| `sand_mpc_compensator` | Python | 订阅 MPPI 原始指令和 odom，估计 slip，求解 MIMO MPC，输出补偿后 `/cmd_vel` |

## Topics

| Topic | 方向 | 类型 | 说明 |
|---|---|---|---|
| `/mppi/cmd_vel_raw` | 输入 | `geometry_msgs/Twist` | MPPI 原始速度指令 |
| `/nav_odom` | 输入 | `nav_msgs/Odometry` | 平面 odom，用于估计实际 v/w 和 slip |
| `/cmd_vel` | 输出 | `geometry_msgs/Twist` | 补偿后最终底盘指令 |

## 配置

| 文件 | 说明 |
|---|---|
| `config/sand_mpc.yaml` | 节点参数（控制频率、horizon、延迟模型、slip 估计、代价权重） |
| `launch/sand_mpc.launch.py` | 单独启动 sand MPC 节点的 launch 文件 |

全栈默认参数在仓库根 `config/nexus_navigation_stack.yaml`。

## 构建

```bash
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1

colcon build --packages-select nexus_sand_mpc --symlink-install
```

Python 依赖（需装到系统 Python，不要装到 conda）：

```bash
/usr/bin/python3 -m pip install --user numpy do-mpc casadi
```

## 运行

```bash
source install/setup.bash

# 单独启动
ros2 launch nexus_sand_mpc sand_mpc.launch.py
# 或
ros2 run nexus_sand_mpc sand_mpc_compensator
```

完整链路中由 `scripts/run_mppi.sh` 自动拉起（`MAP_SIM_ENABLE_SAND_MPC=1` 时）。

## 关键参数

| 参数 | 默认 | 说明 |
|---|---:|---|
| `control_rate` | `20.0` | MPC 输出频率 |
| `horizon` | `10` | MPC 预测步数 |
| `dt_nominal` | `0.05` | 离散时间步长 |
| `cmd_delay` | `0.05` | 指令延迟模型 |
| `drive_tau` / `turn_tau` | `0.08` | 平动 / 转向一阶响应 |
| `slip_alpha` | `0.20` | slip 估计更新率 |
| `slip_init` | `0.20` | 初始 slip |
| `correction_gain` | `0.30` | odom 观测校正增益 |
| `v_max` / `w_max` | `1.50` / `1.40` | 速度限幅 |
| `passthrough_on_missing_odom` | `true` | odom 缺失时透传原始指令 |
| `publish_zero_on_timeout` | `true` | 指令超时时发布 0 |

### 容错行为

- odom 暂时缺失时透传 `/mppi/cmd_vel_raw`，避免链路断掉。
- 指令超时（`command_timeout_sec`）时发布零速。
- `preserve_lateral_direction: true` 保留原始横向方向，只补偿模长和角速度。

## 源码

| 文件 | 说明 |
|---|---|
| `nexus_sand_mpc/sand_mpc_node.py` | ROS 节点封装：订阅、定时回调、指令发布 |
| `nexus_sand_mpc/sand_mpc_controller.py` | do-mpc/CasADi MIMO MPC： plant 模型、延迟队列、观测回放 |

## 相关文档

- [README.md](../../README.md) — 「算法 4：Sand MPC 指令补偿」一节
- [SCRIPTS.md](../../SCRIPTS.md) — `run_mppi.sh` 中 sand MPC 的启动和 cmd_vel 路由
