#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${NEXUS_ELEV_WS_DIR:-$ROOT_DIR/elevation_mapping_cupy_ros2_ws}"
SRC_DIR="$WS_DIR/src"
REPO_URL="${NEXUS_ELEV_REPO_URL:-https://github.com/leggedrobotics/elevation_mapping_cupy.git}"
REPO_BRANCH="${NEXUS_ELEV_REPO_BRANCH:-ros2}"
GRID_MAP_REPO_URL="${NEXUS_GRID_MAP_REPO_URL:-https://github.com/ANYbotics/grid_map.git}"
GRID_MAP_BRANCH="${NEXUS_GRID_MAP_BRANCH:-humble}"

sanitize_python_env() {
  unset PYTHONHOME PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL CONDA_EXE CONDA_PYTHON_EXE _CE_CONDA _CE_M || true
  export PATH="/usr/bin:/bin:/usr/sbin:/sbin:${PATH}"
  if [ -n "${LD_LIBRARY_PATH:-}" ]; then
    local sanitized_ld=""
    sanitized_ld="$(printf '%s' "$LD_LIBRARY_PATH" \
      | tr ':' '\n' \
      | awk 'NF && $0 !~ /(mini)?conda/ && !seen[$0]++' \
      | paste -sd: -)"
    if [ -n "$sanitized_ld" ]; then
      export LD_LIBRARY_PATH="$sanitized_ld"
    else
      unset LD_LIBRARY_PATH || true
    fi
  fi
  hash -r || true
}

[ -f /opt/ros/humble/setup.bash ] || {
  echo "[ERR] Missing /opt/ros/humble/setup.bash"
  exit 1
}

mkdir -p "$SRC_DIR"

if [ ! -d "$SRC_DIR/elevation_mapping_cupy" ]; then
  git clone --depth 1 --branch "$REPO_BRANCH" "$REPO_URL" "$SRC_DIR/elevation_mapping_cupy"
fi

if [ ! -d "$SRC_DIR/grid_map" ]; then
  git clone --depth 1 --branch "$GRID_MAP_BRANCH" "$GRID_MAP_REPO_URL" "$SRC_DIR/grid_map"
fi

sanitize_python_env
set +u
source /opt/ros/humble/setup.bash
set -u

/usr/bin/python3 -m pip install --user ros2-numpy transforms3d

cd "$WS_DIR"
/usr/bin/colcon build \
  --packages-up-to elevation_mapping_cupy \
  --symlink-install \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3

echo "[OK] elevation_mapping_cupy ROS2 workspace built at $WS_DIR"
echo "[NEXT] Launch with: bash ./scripts/run_real_robot.sh"
