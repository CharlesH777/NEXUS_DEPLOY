# Gazebo Data Integration Test For DEPLOY

Date: 2026-07-08

## Conclusion

The DEPLOY data path is flowing correctly when Gazebo is used only as a sensor
data source.

The production DEPLOY runtime does not launch Gazebo. Gazebo appears only in
`scripts/test_with_sim_data.sh`, where it replaces real Livox/IMU hardware for
integration testing.

## Runtime Boundary

Production entry point:

```bash
bash scripts/run_real_robot.sh
```

Production data source:

```text
real Livox LiDAR + real Livox IMU
```

Integration-test entry point:

```bash
MAP_SIM_GZCLIENT=0 timeout 240 bash scripts/test_with_sim_data.sh
```

Integration-test data source:

```text
Gazebo Livox/IMU simulation
```

The test intentionally starts Gazebo, but the DEPLOY launch under test is still
`launch/real_robot_bringup.launch.py`.

## Verified Data Flow

The following chain was verified:

```text
Gazebo Livox/IMU
  -> fastlio_lidar_adapter + fastlio_imu_adapter
  -> FAST-LIO2 lio_node
  -> /fastlio2/lio_odom
  -> fastlio_odom_bridge
  -> /odom + /pose + map->base_footprint TF
  -> /fastlio2/world_cloud
  -> elevation_mapping_node
  -> /elevation_mapping_node/elevation_map
  -> traversability_to_map
  -> /traversability_map
  -> Nav2 MPPI + sand MPC
```

## Passed Checks

These outputs were observed during the latest clean run:

- `/odom` published `nav_msgs/msg/Odometry` in frame `map`, child frame `base_footprint`.
- `/fastlio2/world_cloud` published point cloud headers in frame `map`.
- `/traversability_map` published occupancy-grid headers in frame `map`.
- `map -> base_footprint` TF was available and changing.
- `elevation_mapping_node` initialized its map: length `20.0`, resolution `0.1`, cells `202`.
- Nav2 lifecycle manager reported managed nodes active.
- `sand_mpc_compensator` started and subscribed to `/odom`.
- After cleanup, no Gazebo, FAST-LIO2, DEPLOY launch, or integration-test ROS processes remained.
- After restarting the ROS daemon, `ros2 node list` was empty.

## Important Test Environment

The test script sanitizes Conda variables but keeps the Python user packages
needed by elevation mapping. It also exports CUDA and torch wheel library paths:

```text
/usr/local/lib/python3.10/dist-packages/nvidia/*/lib
/home/charles/.local/lib/python3.10/site-packages/nvidia/*/lib
/usr/local/cuda/targets/x86_64-linux/lib
/usr/local/cuda/lib64
```

This is required because `elevation_mapping_cupy` imports `torch` and `cupy`.
Without these library paths, torch can fail to load CUDA libraries such as
`libcusparseLt.so.0` or `libnvshmem_host.so.3`.

## Known Warnings

The integration test passes the data-flow objective, but the following warnings
remain relevant for tuning:

- FAST-LIO2 can report `IMU Message is out of order`.
- Nav2 can report action aborts such as empty path or missed control loop rates.
- The old `NEXUS_GAZEBO_SIM` install may print `not found` lines for stale
  elevation overlay paths while sourcing. These did not block the current test.
- Python ROS nodes can print `ExternalShutdownException` during intentional
  process-group shutdown. In this test context that is shutdown noise, not a
  startup/runtime data-flow failure.

## No Gazebo Coupling In Production

Gazebo coupling is not present in the production entry points:

- `scripts/run_real_robot.sh`
- `launch/real_robot_bringup.launch.py`
- `launch/nav2_mppi_real.launch.py`
- `config/nexus_real_navigation_stack.yaml`
- `config/nav2_mppi_real_params.yaml`

`scripts/test_with_sim_data.sh` is the only Gazebo-dependent path and is used
only to feed simulated sensor data into DEPLOY for validation.

