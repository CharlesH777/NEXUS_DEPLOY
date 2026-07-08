#!/usr/bin/env bash
set -euo pipefail

# ──────────────────────────────────────────────────────────────────
# Integration test: feed Gazebo-simulated LiDAR+IMU into the DEPLOY
# stack to verify the full pipeline end-to-end without real hardware.
# ──────────────────────────────────────────────────────────────────

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_DIR="${NEXUS_SIM_DIR:-$ROOT_DIR/../NEXUS_GAZEBO_SIM}"
DEPLOY_DIR="$ROOT_DIR"
SIM_PID=""
DEPLOY_PID=""
STATIC_TF_PID=""

kill_process_group() {
  local pid="${1:-}"
  [ -n "$pid" ] || return 0
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  sleep 1
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
}

cleanup() {
  set +e
  kill_process_group "$DEPLOY_PID"
  kill_process_group "$STATIC_TF_PID"
  kill_process_group "$SIM_PID"
  pkill -TERM -f "gzserver" 2>/dev/null || true
  pkill -TERM -f "gzclient" 2>/dev/null || true
  pkill -TERM -f "robot_state_publisher" 2>/dev/null || true
  pkill -TERM -f "livox_laser_simulation" 2>/dev/null || true
  pkill -TERM -f "real_robot_bringup" 2>/dev/null || true
  pkill -TERM -f "lio_node" 2>/dev/null || true
  pkill -TERM -f "$DEPLOY_DIR/install/nexus_fastlio/lib/nexus_fastlio/fastlio_" 2>/dev/null || true
  pkill -TERM -f "$DEPLOY_DIR/install/nexus_elevation_mppi/lib/nexus_elevation_mppi/traversability_to_map" 2>/dev/null || true
  pkill -TERM -f "$DEPLOY_DIR/install/nexus_elevation_mppi/lib/nexus_elevation_mppi/novelty_explorer" 2>/dev/null || true
  pkill -TERM -f "$DEPLOY_DIR/install/nexus_sand_mpc/lib/nexus_sand_mpc/sand_mpc_compensator" 2>/dev/null || true
  pkill -TERM -f "$DEPLOY_DIR/tools/elevation_mapping_cupy_ros2_ws/install/elevation_mapping_cupy/lib/elevation_mapping_cupy/elevation_mapping_node.py" 2>/dev/null || true
  pkill -TERM -f "$DEPLOY_DIR/scripts/continuous_navigator.py" 2>/dev/null || true
  pkill -TERM -f "$SIM_DIR/install/ros2_livox_simulation/lib/ros2_livox_simulation/cmd_vel_to_swerve" 2>/dev/null || true
  pkill -TERM -f "$SIM_DIR/install/ros2_livox_simulation/lib/ros2_livox_simulation/fix_imu_time" 2>/dev/null || true
  pkill -TERM -f "nav2_lifecycle_manager/lifecycle_manager" 2>/dev/null || true
}
trap cleanup EXIT

echo "================================================================"
echo "[1/5] Starting Gazebo bare-sensor sim (no nav stack)..."
echo "================================================================"

# --- Sanitize conda but keep CUDA/torch/user packages ---
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER \
      CONDA_SHLVL CONDA_EXE CONDA_PYTHON_EXE _CE_CONDA _CE_M || true
export PATH="/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/cuda/bin:${PATH}"
# Keep LD_LIBRARY_PATH but strip conda entries
if [ -n "${LD_LIBRARY_PATH:-}" ]; then
  export LD_LIBRARY_PATH="$(printf '%s' "$LD_LIBRARY_PATH" | tr ':' '\n' | awk 'NF && $0 !~ /(mini)?conda/ && !seen[$0]++' | paste -sd: -)"
fi
CUDA_TARGET_LIB="${CUDA_TARGET_LIB:-/usr/local/cuda/targets/x86_64-linux/lib}"
TORCH_NVIDIA_LIB_DIRS="${NEXUS_TORCH_NVIDIA_LIB_DIRS:-$(find /usr/local/lib/python3.10/dist-packages/nvidia "$HOME/.local/lib/python3.10/site-packages/nvidia" -maxdepth 3 -type d -name lib 2>/dev/null | paste -sd: -)}"
export LD_LIBRARY_PATH="$TORCH_NVIDIA_LIB_DIRS:$CUDA_TARGET_LIB:/usr/local/cuda/lib64:/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
# Restore Python user site-packages for torch/tf_transformations
export PYTHONPATH="$HOME/.local/lib/python3.10/site-packages:${PYTHONPATH:-}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/nexus_ros_log_simtest}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/nexus_mpl_config}"
mkdir -p "$ROS_LOG_DIR" "$MPLCONFIGDIR"
unset PYTHONHOME
hash -r || true

set +u
source /opt/ros/humble/setup.bash
source "$SIM_DIR/install/setup.bash"
source "$SIM_DIR/tools/elevation_mapping_cupy_ros2_ws/install/setup.bash"
set -u

# Start Gazebo: sensors only, no nav/FAST-LIO2/elevation/RViz/TF-pub
setsid env \
MAP_SIM_ENABLE_ELEVATION_MAPPING=0 \
MAP_SIM_ENABLE_MPPI_NAVIGATION=0 \
MAP_SIM_ENABLE_DEFAULT_STACK=0 \
MAP_SIM_ENABLE_FASTLIO2=0 \
MAP_SIM_ENABLE_POINTCLOUD_PIPELINE=0 \
MAP_SIM_ENABLE_RVIZ=0 \
MAP_SIM_ENABLE_TF_PUB=0 \
MAP_SIM_ENABLE_ROS2_CONTROL=1 \
MAP_SIM_GZCLIENT="${MAP_SIM_GZCLIENT:-1}" \
MAP_SIM_FORCE_CLEAN_START=1 \
MAP_SIM_WORLD="${MAP_SIM_WORLD:-rm_2026_slam_world.world}" \
ros2 launch ros2_livox_simulation sim_launch_omni.py \
  world_name:="${MAP_SIM_WORLD:-rm_2026_slam_world.world}" \
  use_gui:="${MAP_SIM_GZCLIENT:-1}" \
  enable_headless_rendering:=0 \
  spawn_robot:=true \
  spawn_x:=0.0 spawn_y:=0.0 spawn_z:=0.19 \
  enable_livox:=true \
  enable_depth_camera:=0 \
  enable_imu:=true \
  enable_tf_pub:=false \
  enable_rviz:=0 \
  enable_pointcloud_pipeline:=0 \
  enable_fastlio2:=0 \
  enable_ros2_control:=true \
  livox_samples:=20000 \
  livox_downsample:=1 \
  livox_max_range:=70.0 \
  > /tmp/nexus_sim_bare.log 2>&1 &

SIM_PID=$!
echo "[INFO] Gazebo sim started (PID $SIM_PID, headless)"
echo "[INFO] Waiting 20s for sim to initialize..."
sleep 20

echo ""
echo "================================================================"
echo "[2/5] Checking sim topics..."
echo "================================================================"
set +u
source /opt/ros/humble/setup.bash
source "$SIM_DIR/install/setup.bash"
set -u

echo "--- Sim topics ---"
ros2 topic list 2>/dev/null | grep -E "livox|imu|clock|cmd_vel|tf" | sort || true
echo ""
echo "--- Sim TF frames ---"
timeout 3 ros2 run tf2_ros tf2_echo world base_footprint 2>/dev/null || true
echo ""

echo "================================================================"
echo "[3/5] Starting DEPLOY stack with sim data..."
echo "================================================================"

# Source DEPLOY workspace
set +u
source "$DEPLOY_DIR/install/setup.bash"
set -u

# Publish static world→map TF so Gazebo's world frame connects to DEPLOY's map frame
setsid ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id world --child-frame-id map \
  > /dev/null 2>&1 &
STATIC_TF_PID=$!
echo "[INFO] Static TF world→map published (PID $STATIC_TF_PID)"

# Launch DEPLOY stack: consume sim-provided sensor data
setsid ros2 launch "$DEPLOY_DIR/launch/real_robot_bringup.launch.py" \
  use_sim_time:=true \
  launch_livox:=false \
  enable_fastlio2:=true \
  enable_elevation:=true \
  enable_traversability:=true \
  enable_nav2:=true \
  enable_sand_mpc:=true \
  enable_exploration:=true \
  enable_rviz:=false \
  publish_map_to_odom:=false \
  publish_base_footprint_to_base_link:=false \
  publish_livox_static_tf:=false \
  global_frame:=map \
  robot_frame:=base_footprint \
  base_frame:=base_link \
  livox_frame:=livox_frame \
  fastlio2_lidar_input_topic:=/livox/lidar \
  fastlio2_lidar_output_topic:=/lidar_fastlio \
  fastlio2_lidar_rotation_pitch_deg:=30.0 \
  fastlio2_imu_input_topic:=/imu_fixed \
  fastlio2_imu_output_topic:=/imu_fastlio \
  fastlio2_imu_linear_accel_scale:=0.1 \
  fastlio2_imu_rotation_pitch_deg:=30.0 \
  fastlio2_target_frame_id:=base_link \
  fastlio2_publish_odom_tf:=true \
  > /tmp/nexus_deploy_simtest.log 2>&1 &

DEPLOY_PID=$!
echo "[INFO] DEPLOY stack started (PID $DEPLOY_PID)"

echo ""
echo "================================================================"
echo "[4/5] Monitoring for 30s..."
echo "================================================================"
sleep 5
echo "--- Active nodes (5s) ---"
ros2 node list 2>/dev/null | sort

sleep 10
echo ""
echo "--- /odom check (15s) ---"
timeout 3 ros2 topic echo /odom --once 2>/dev/null | head -15 || echo "  [NO /odom DATA YET]"

echo ""
echo "--- /fastlio2/world_cloud check ---"
timeout 3 ros2 topic echo /fastlio2/world_cloud --once --field header 2>/dev/null || echo "  [NO world_cloud YET]"

echo ""
echo "--- /traversability_map check ---"
timeout 3 ros2 topic echo /traversability_map --once --field header 2>/dev/null || echo "  [NO traversability_map YET]"

sleep 10
echo ""
echo "--- Active nodes (25s) ---"
ros2 node list 2>/dev/null | sort

echo ""
echo "--- /odom check (25s) ---"
timeout 3 ros2 topic echo /odom --once 2>/dev/null | head -15 || echo "  [STILL NO /odom]"

echo ""
echo "--- Nav2 lifecycle status ---"
ros2 lifecycle list /controller_server 2>/dev/null | head -5 || echo "  [controller_server not found]"
ros2 lifecycle list /planner_server 2>/dev/null | head -5 || echo "  [planner_server not found]"

echo ""
echo "--- TF tree (map → base_footprint) ---"
timeout 3 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null || true

echo ""
echo "================================================================"
echo "[5/5] Done. Shutting down..."
echo "================================================================"

cleanup
trap - EXIT

echo ""
echo "=== Deploy log tail ==="
tail -30 /tmp/nexus_deploy_simtest.log 2>/dev/null || true
echo ""
echo "=== Sim log tail ==="
tail -10 /tmp/nexus_sim_bare.log 2>/dev/null || true

echo ""
echo "[DONE] Integration test complete."
