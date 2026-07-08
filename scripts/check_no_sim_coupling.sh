#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

patterns=(
  "gazebo"
  "gzserver"
  "gzclient"
  "gazebo_ros"
  "ros_gz"
  "ros_ign"
  "spawn_entity"
  "/clock"
  "use_sim_time:[[:space:]]*true"
  "cube_robot"
  "/nav_odom"
  "lidar_PointCloud2"
)

scan_paths=(
  launch
  config
  scripts
  runlocal
  src/nexus_fastlio
  src/nexus_elevation_mppi
  src/nexus_sand_mpc
  src/nexus_teleop
  src/nexus_gp_mapping
  tools/elevation_ros2
)

status=0
for pattern in "${patterns[@]}"; do
  if rg -n -i \
    --glob '!*.md' \
    --glob '!scripts/check_no_sim_coupling.sh' \
    --glob '!scripts/test_with_sim_data.sh' \
    "$pattern" "${scan_paths[@]}" 2>/dev/null; then
    status=1
  fi
done

if [ "$status" -ne 0 ]; then
  echo "[ERR] Simulation-coupled runtime references found." >&2
  exit 1
fi

if [ -d src/ros2_livox_simulation ] || [ -d .fuel_models ] || [ -d .external_worlds ]; then
  echo "[ERR] Simulation package/assets are present in deployment workspace." >&2
  exit 1
fi

echo "[OK] Runtime launch/config/scripts are free of Gazebo simulation coupling."
