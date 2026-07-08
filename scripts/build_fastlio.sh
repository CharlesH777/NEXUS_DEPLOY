#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FASTLIO_WS_DIR="${NEXUS_FASTLIO2_WS_DIR:-$ROOT_DIR/third_party/FASTLIO2_ROS2}"
FASTLIO_INSTALL_BASE="${NEXUS_FASTLIO2_INSTALL_BASE:-install_nexus}"
FASTLIO_BUILD_BASE="${NEXUS_FASTLIO2_BUILD_BASE:-build_nexus}"
FASTLIO_LOG_BASE="${NEXUS_FASTLIO2_LOG_BASE:-log_nexus}"

sanitize_python_env() {
  unset PYTHONHOME PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER \
        CONDA_SHLVL CONDA_EXE CONDA_PYTHON_EXE _CE_CONDA _CE_M || true
  export PATH="/usr/bin:/bin:/usr/sbin:/sbin:${PATH}"
  if [ -n "${LD_LIBRARY_PATH:-}" ]; then
    local sanitized_ld
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
[ -f "$ROOT_DIR/install/setup.bash" ] || {
  echo "[ERR] Missing $ROOT_DIR/install/setup.bash"
  echo "[HINT] Build the deploy workspace first: bash scripts/build_deploy.sh"
  exit 1
}
[ -d "$FASTLIO_WS_DIR/fastlio2" ] || {
  echo "[ERR] Missing FAST-LIO2 source dir: $FASTLIO_WS_DIR/fastlio2"
  echo "[HINT] Symlink or clone FASTLIO2_ROS2 into $ROOT_DIR/third_party/"
  exit 1
}

sanitize_python_env
set +u
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u

cd "$FASTLIO_WS_DIR"
colcon --log-base "$FASTLIO_LOG_BASE" build \
  --packages-select fastlio2 \
  --symlink-install \
  --build-base "$FASTLIO_BUILD_BASE" \
  --install-base "$FASTLIO_INSTALL_BASE"

echo "[OK] FAST-LIO2 built at $FASTLIO_WS_DIR/$FASTLIO_INSTALL_BASE"

echo "[NEXT] Launch with: bash scripts/run_real_robot.sh"