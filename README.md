# NEXUS Real-Robot Deploy

This folder is the real-robot deployment cut of `NEXUS_GAZEBO_SIM`.
Gazebo Classic, world/model assets, robot spawn, simulation clock, and simulated ros2_control paths are intentionally not included.

## Runtime Boundary

- LiDAR input: real `livox_ros_driver2`, default `CustomMsg` topic `/livox/lidar` for FAST-LIO2.
- Odometry source: FAST-LIO2 publishes `/fastlio2/lio_odom`; `nexus_fastlio/fastlio_odom_bridge` republishes it as `/odom` and `/pose`.
- Elevation input: `/fastlio2/world_cloud`, not the raw Livox point cloud.
- Control output: `/cmd_vel` is the hardware-driver boundary. With sand MPC enabled, Nav2 publishes `/mppi/cmd_vel_raw` and sand MPC publishes `/cmd_vel`.
- Frames: default global frame is `map`, robot frame is `base_footprint`, base link is `base_link`, LiDAR frame is `livox_frame`.
- Time: `use_sim_time=false` everywhere.

FAST-LIO2 is enabled by default. The launch path is:

```text
/livox/lidar + /livox/imu
  -> fastlio_lidar_adapter + fastlio_imu_adapter
  -> FAST-LIO2 lio_node
  -> /fastlio2/lio_odom + /fastlio2/world_cloud
  -> fastlio_odom_bridge
  -> /odom + /pose + map->base_footprint TF
```

`scripts/run_real_robot.sh` checks that the FAST-LIO2 executable exists. Set it explicitly if your install path is different:

```bash
NEXUS_FASTLIO2_BIN=/path/to/lio_node bash scripts/run_real_robot.sh
```

If you disable FAST-LIO2, you must provide your own `/odom` and TF source:

```bash
NEXUS_ENABLE_FASTLIO2=false NEXUS_LIVOX_XFER_FORMAT=0 bash scripts/run_real_robot.sh
```

## Build

```bash
cd /home/charles/NEXUS/NEXUS/NEXUS_LIDAR_SIM/DEPLOY
bash scripts/build_deploy.sh
```

The build script first builds `tools/elevation_mapping_cupy_ros2_ws` if needed, then builds packages under `src/`.

## Run

```bash
bash scripts/run_real_robot.sh
```

Common options:

```bash
NEXUS_FASTLIO2_BIN=/path/to/lio_node bash scripts/run_real_robot.sh
NEXUS_ENABLE_RVIZ=true bash scripts/run_real_robot.sh
NEXUS_ENABLE_SAND_MPC=false bash scripts/run_real_robot.sh nav2_cmd_vel_topic:=/cmd_vel
NEXUS_LIVOX_CONFIG=/path/to/MID360_config.json bash scripts/run_real_robot.sh
```

Stop:

```bash
bash scripts/stop.sh
```

## Gazebo Data Integration Test

Gazebo is not part of the production runtime. It is only used by
`scripts/test_with_sim_data.sh` as a sensor-data source for DEPLOY integration
testing.

Latest status and validation notes are documented in
[`docs/GAZEBO_DEPLOY_INTEGRATION.md`](docs/GAZEBO_DEPLOY_INTEGRATION.md).

## Migrated Packages

- `livox_ros_driver2`
- `nexus_elevation_mppi`
- `nexus_fastlio`
- `nexus_gp_mapping`
- `nexus_sand_mpc`
- `nexus_teleop`
- Selected LRAE local planning packages: `fitplane`, `gen_local_goal`, `local_planner`, `lrae_planner`, `sensor_conversion`
- Elevation mapping dependencies under `tools/elevation_mapping_cupy_ros2_ws/src`

## Removed From Deployment

- `.fuel_models`
- `.external_worlds`
- `ros2_livox_simulation`
- Gazebo world/model/URDF/spawn/controller launch paths
- Simulation truth TF publisher
- Existing `build/`, `install/`, and `log/` products from the source project
