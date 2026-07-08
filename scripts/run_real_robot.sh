#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELEV_WS_DIR="${NEXUS_ELEV_WS_DIR:-$ROOT_DIR/tools/elevation_mapping_cupy_ros2_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
FASTLIO2_BIN="${NEXUS_FASTLIO2_BIN:-$ROOT_DIR/third_party/FASTLIO2_ROS2/install_nexus/fastlio2/lib/fastlio2/lio_node}"
FASTLIO2_CONFIG="${NEXUS_FASTLIO2_CONFIG:-$ROOT_DIR/src/nexus_fastlio/config/fastlio2_real.yaml}"

sanitize_python_env() {
  unset PYTHONHOME PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER \
        CONDA_SHLVL CONDA_EXE CONDA_PYTHON_EXE _CE_CONDA _CE_M || true
  export PATH="/usr/bin:/bin:/usr/sbin:/sbin:${PATH}"
  if [ -n "${LD_LIBRARY_PATH:-}" ]; then
    local sanitized_ld
    sanitized_ld="$(printf '%s' "$LD_LIBRARY_PATH" | tr ':' '\n' | awk 'NF && $0 !~ /(mini)?conda/ && !seen[$0]++' | paste -sd: -)"
    if [ -n "$sanitized_ld" ]; then
      export LD_LIBRARY_PATH="$sanitized_ld"
    else
      unset LD_LIBRARY_PATH || true
    fi
  fi
  hash -r || true
}

is_enabled() {
  case "${1,,}" in
    1|true|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

[ -f "$ROS_SETUP" ] || {
  echo "[ERR] Missing ROS setup: $ROS_SETUP" >&2
  exit 1
}
[ -f "$ROOT_DIR/install/setup.bash" ] || {
  echo "[ERR] Missing workspace install: $ROOT_DIR/install/setup.bash" >&2
  echo "[HINT] Build first: bash scripts/build_deploy.sh" >&2
  exit 1
}
[ -f "$ELEV_WS_DIR/install/setup.bash" ] || {
  echo "[ERR] Missing elevation workspace install: $ELEV_WS_DIR/install/setup.bash" >&2
  echo "[HINT] Build first: bash scripts/build_deploy.sh" >&2
  exit 1
}

FASTLIO2_ENABLED="${NEXUS_ENABLE_FASTLIO2:-true}"
if is_enabled "$FASTLIO2_ENABLED"; then
  [ -x "$FASTLIO2_BIN" ] || {
    echo "[ERR] FAST-LIO2 is enabled, but lio_node is not executable: $FASTLIO2_BIN" >&2
    echo "[HINT] Build/install FAST-LIO2, or set NEXUS_FASTLIO2_BIN=/path/to/lio_node." >&2
    echo "[HINT] To run without FAST-LIO2, set NEXUS_ENABLE_FASTLIO2=false and provide your own /odom." >&2
    exit 1
  }
  [ -f "$FASTLIO2_CONFIG" ] || {
    echo "[ERR] FAST-LIO2 config not found: $FASTLIO2_CONFIG" >&2
    exit 1
  }
fi

sanitize_python_env
set +u
source "$ROS_SETUP"
source "$ELEV_WS_DIR/install/setup.bash"
source "$ROOT_DIR/install/setup.bash"
set -u

if [ "${NEXUS_ENABLE_SAND_MPC:-1}" = "1" ]; then
  NAV2_CMD_VEL_TOPIC="${NEXUS_NAV2_CMD_VEL_TOPIC:-/mppi/cmd_vel_raw}"
else
  NAV2_CMD_VEL_TOPIC="${NEXUS_NAV2_CMD_VEL_TOPIC:-/cmd_vel}"
fi

exec ros2 launch "$ROOT_DIR/launch/real_robot_bringup.launch.py" \
  launch_livox:="${NEXUS_LAUNCH_LIVOX:-true}" \
  enable_fastlio2:="$FASTLIO2_ENABLED" \
  enable_elevation:="${NEXUS_ENABLE_ELEVATION:-true}" \
  enable_traversability:="${NEXUS_ENABLE_TRAVERSABILITY:-true}" \
  enable_nav2:="${NEXUS_ENABLE_NAV2:-true}" \
  enable_sand_mpc:="${NEXUS_ENABLE_SAND_MPC:-true}" \
  enable_exploration:="${NEXUS_ENABLE_EXPLORATION:-true}" \
  enable_exporter:="${NEXUS_ENABLE_EXPORTER:-false}" \
  enable_rviz:="${NEXUS_ENABLE_RVIZ:-false}" \
  publish_map_to_odom:="${NEXUS_PUBLISH_MAP_TO_ODOM:-false}" \
  publish_base_footprint_to_base_link:="${NEXUS_PUBLISH_BASE_FOOTPRINT_TO_BASE_LINK:-true}" \
  publish_livox_static_tf:="${NEXUS_PUBLISH_LIVOX_STATIC_TF:-true}" \
  nav2_cmd_vel_topic:="$NAV2_CMD_VEL_TOPIC" \
  global_frame:="${NEXUS_GLOBAL_FRAME:-map}" \
  robot_frame:="${NEXUS_ROBOT_FRAME:-base_footprint}" \
  base_frame:="${NEXUS_BASE_FRAME:-base_link}" \
  livox_frame:="${NEXUS_LIVOX_FRAME:-livox_frame}" \
  livox_config:="${NEXUS_LIVOX_CONFIG:-$ROOT_DIR/src/livox_ros_driver2/config/MID360_config.json}" \
  livox_xfer_format:="${NEXUS_LIVOX_XFER_FORMAT:-1}" \
  livox_multi_topic:="${NEXUS_LIVOX_MULTI_TOPIC:-0}" \
  livox_publish_freq:="${NEXUS_LIVOX_PUBLISH_FREQ:-10.0}" \
  livox_bd_code:="${NEXUS_LIVOX_BD_CODE:-livox0000000001}" \
  base_link_x:="${NEXUS_BASE_LINK_X:-0.0}" \
  base_link_y:="${NEXUS_BASE_LINK_Y:-0.0}" \
  base_link_z:="${NEXUS_BASE_LINK_Z:-0.0}" \
  base_link_roll:="${NEXUS_BASE_LINK_ROLL:-0.0}" \
  base_link_pitch:="${NEXUS_BASE_LINK_PITCH:-0.0}" \
  base_link_yaw:="${NEXUS_BASE_LINK_YAW:-0.0}" \
  livox_x:="${NEXUS_LIVOX_X:-0.0}" \
  livox_y:="${NEXUS_LIVOX_Y:-0.0}" \
  livox_z:="${NEXUS_LIVOX_Z:-0.4}" \
  livox_roll:="${NEXUS_LIVOX_ROLL:-0.0}" \
  livox_pitch:="${NEXUS_LIVOX_PITCH:-0.0}" \
  livox_yaw:="${NEXUS_LIVOX_YAW:-0.0}" \
  fastlio2_bin:="$FASTLIO2_BIN" \
  fastlio2_config:="$FASTLIO2_CONFIG" \
  fastlio2_namespace:="${NEXUS_FASTLIO2_NAMESPACE:-/fastlio2}" \
  fastlio2_tf_topic:="${NEXUS_FASTLIO2_TF_TOPIC:-/fastlio2/tf}" \
  fastlio2_lidar_input_topic:="${NEXUS_FASTLIO2_LIDAR_INPUT_TOPIC:-/livox/lidar}" \
  fastlio2_lidar_output_topic:="${NEXUS_FASTLIO2_LIDAR_OUTPUT_TOPIC:-/lidar_fastlio}" \
  fastlio2_lidar_rotation_pitch_deg:="${NEXUS_FASTLIO2_LIDAR_ROTATION_PITCH_DEG:-0.0}" \
  fastlio2_imu_input_topic:="${NEXUS_FASTLIO2_IMU_INPUT_TOPIC:-/livox/imu}" \
  fastlio2_imu_output_topic:="${NEXUS_FASTLIO2_IMU_OUTPUT_TOPIC:-/imu_fastlio}" \
  fastlio2_imu_linear_accel_scale:="${NEXUS_FASTLIO2_IMU_LINEAR_ACCEL_SCALE:-1.0}" \
  fastlio2_imu_rotation_pitch_deg:="${NEXUS_FASTLIO2_IMU_ROTATION_PITCH_DEG:-0.0}" \
  fastlio2_target_frame_id:="${NEXUS_FASTLIO2_TARGET_FRAME_ID:-base_link}" \
  fastlio2_odom_topic:="${NEXUS_FASTLIO2_ODOM_TOPIC:-/fastlio2/lio_odom}" \
  fastlio2_publish_odom_tf:="${NEXUS_FASTLIO2_PUBLISH_ODOM_TF:-true}" \
  fastlio2_publish_pose:="${NEXUS_FASTLIO2_PUBLISH_POSE:-true}" \
  odom_topic:="${NEXUS_ODOM_TOPIC:-/odom}" \
  pose_topic:="${NEXUS_POSE_TOPIC:-/pose}" \
  "$@"
