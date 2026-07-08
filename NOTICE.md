# 第三方代码声明

本项目（NEXUS Gazebo Sim）整体采用**专有授权（All Rights Reserved）**，版权所有 © 2026 Charles。
本项目内包含的以下第三方代码保留其原始许可证，仅用于内部参考和编译依赖。

---

## 1. livox_ros_driver2

- **来源**: https://github.com/Livox-SDK/livox_ros_driver2
- **许可证**: MIT License
- **版权**: Copyright (c) Livox
- **路径**: `src/livox_ros_driver2/`
- **用途**: Livox 系列激光雷达 ROS 2 驱动及自定义消息接口

---

## 2. ros2_livox_simulation

- **来源**: https://github.com/Livox-SDK/livox_laser_simulation (ROS 2 移植)
- **许可证**: MIT License
- **版权**: Copyright (c) 2023 Ricardo Casimiro
- **路径**: `src/ros2_livox_simulation/`
- **用途**: Gazebo Classic 中 Livox 雷达仿真插件、URDF、世界模型

---

## 3. voxblox

- **来源**: https://github.com/ethz-asl/voxblox
- **许可证**: BSD License
- **版权**: Copyright (c) 2016, ETHZ ASL
- **路径**: `src/third_party/voxblox/`
- **用途**: 体素地图库

---

## 4. minkindr

- **来源**: https://github.com/ethz-asl/minkindr
- **许可证**: BSD License
- **版权**: Copyright (c) 2015, Autonomous Systems Lab, ETH Zurich
- **路径**: `src/third_party/minkindr/`
- **用途**: 刚体变换工具库

---

## 5. m_explore_ros2

- **来源**: https://github.com/rohbotics/m-explore_ros2
- **许可证**: BSD License
- **版权**: Copyright (c) 2015-2016, Carlos Alvarez
- **路径**: `src/third_party/m_explore_ros2/`
- **用途**: ROS 2 前沿探索节点

---

## 6. lrae_planner / gbplanner / planner_common / local_planner

- **来源**: GBPlanner / LRAE 探索规划器
- **许可证**: MIT / BSD（各子包见 `package.xml` 声明）
- **路径**: `src/third_party/lrae_planner/`, `src/third_party/gbplanner/`, 等
- **用途**: 自主探索路径规划

---

## 7. 其他第三方包

以下包位于 `src/third_party/` 下，各自的许可证在 `package.xml` 中声明：

| 包名 | 许可证 |
|------|--------|
| `fitplane` | MIT |
| `gen_local_goal` | MIT |
| `kdtree` | TBD |
| `local_planner` | BSD |
| `sensor_conversion` | MIT |
| `adaptive_obb_ros` | 未声明 |
| `planner_common` | TODO |
| `planner_msgs` | TODO |
| `planner_semantic_msgs` | TODO |

---

## 兼容性说明

本项目采用专有授权（All Rights Reserved）。第三方代码（MIT / BSD / Apache-2.0）作为依赖引入，其原始许可证仅覆盖第三方代码本身，不影响本项目的整体授权。
