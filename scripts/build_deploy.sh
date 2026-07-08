#!/usr/bin/env bash
set -euo pipefail

# ╔══════════════════════════════════════════════════════════════╗
# ║  NEXUS DEPLOY 一键构建脚本                                       ║
# ║  1. 检查环境依赖 (check_env.sh)                                  ║
# ║  2. 构建 elevation_mapping_cupy workspace (CUDA 高程图)         ║
# ║  3. colcon build src/ (nexus 包 + livox 驱动)                   ║
# ║  4. 构建 FAST-LIO2 (里程计 + 点云)                              ║
# ╚══════════════════════════════════════════════════════════════╝
#
# 用法:
#   bash scripts/build_deploy.sh              # 全量构建
#   bash scripts/build_deploy.sh --skip-env   # 跳过环境检查
#   NEXUS_BUILD_ELEVATION=0 bash scripts/build_deploy.sh  # 跳过高程图
#   NEXUS_BUILD_FASTLIO2=0  bash scripts/build_deploy.sh  # 跳过 FAST-LIO2

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELEV_WS_DIR="${NEXUS_ELEV_WS_DIR:-$ROOT_DIR/tools/elevation_mapping_cupy_ros2_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
SKIP_ENV=0

for arg in "$@"; do
  case "$arg" in
    --skip-env) SKIP_ENV=1; shift ;;
  esac
done

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

# ── 0. 环境检查 ─────────────────────────────────────────────────
if [ "$SKIP_ENV" -eq 0 ]; then
  echo "[0/4] 环境检查..."
  bash "$ROOT_DIR/scripts/check_env.sh" || {
    echo "[ERR] 环境检查未通过。安装缺失依赖后重试，或 --skip-env 跳过。" >&2
    exit 1
  }
  echo
fi

# ── 1. FAST-LIO2 符号链接检测 ──────────────────────────────────
FASTLIO_DIR="$ROOT_DIR/third_party/FASTLIO2_ROS2"
if [ -L "$FASTLIO_DIR" ]; then
  echo "[WARN] $FASTLIO_DIR 是符号链接 → $(readlink "$FASTLIO_DIR")"
  echo "       拷贝到其他机器会断裂。建议替换为真实源码:"
  echo "       rm $FASTLIO_DIR && cp -r <源码路径> $FASTLIO_DIR"
  echo
fi

# ── 2. source ROS ───────────────────────────────────────────────
[ -f "$ROS_SETUP" ] || {
  echo "[ERR] Missing ROS setup: $ROS_SETUP" >&2
  exit 1
}
sanitize_python_env
set +u
source "$ROS_SETUP"
set -u

# ── 3. 构建 elevation_mapping_cupy workspace ────────────────────
if [ "${NEXUS_BUILD_ELEVATION:-1}" = "1" ] && [ ! -f "$ELEV_WS_DIR/install/setup.bash" ]; then
  echo "[1/4] 构建 elevation_mapping_cupy workspace..."
  NEXUS_ELEV_WS_DIR="$ELEV_WS_DIR" \
    bash "$ROOT_DIR/tools/elevation_ros2/build_elevation_mapping_ros2.sh"
else
  echo "[1/4] elevation_mapping_cupy workspace 已构建 (跳过)"
fi

if [ -f "$ELEV_WS_DIR/install/setup.bash" ]; then
  set +u
  source "$ELEV_WS_DIR/install/setup.bash"
  set -u
else
  echo "[ERR] Missing elevation workspace install: $ELEV_WS_DIR/install/setup.bash" >&2
  echo "[HINT] Build it with: NEXUS_BUILD_ELEVATION=1 bash scripts/build_deploy.sh" >&2
  exit 1
fi

# ── 4. colcon build src/ ────────────────────────────────────────
echo "[2/4] 构建 DEPLOY 工作空间 (src/)..."
cd "$ROOT_DIR"
colcon build \
  --base-paths src \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 \
  "$@"
echo "[OK] DEPLOY 工作空间构建完成。"

# ── 5. FAST-LIO2 (里程计 + 点云) ────────────────────────────────
FASTLIO_WS_DIR="${NEXUS_FASTLIO2_WS_DIR:-$FASTLIO_DIR}"
FASTLIO_BIN="$FASTLIO_WS_DIR/install_nexus/fastlio2/lib/fastlio2/lio_node"

if [ "${NEXUS_BUILD_FASTLIO2:-1}" = "1" ] && [ -d "$FASTLIO_WS_DIR/fastlio2" ]; then
  if [ ! -x "$FASTLIO_BIN" ]; then
    echo "[3/4] 构建 FAST-LIO2..."
    NEXUS_FASTLIO2_WS_DIR="$FASTLIO_WS_DIR" \
      bash "$ROOT_DIR/scripts/build_fastlio.sh"
  else
    echo "[3/4] FAST-LIO2 lio_node 已构建 (跳过)"
  fi
elif [ "${NEXUS_BUILD_FASTLIO2:-1}" = "1" ]; then
  echo "[3/4] [WARN] FAST-LIO2 源码未找到: $FASTLIO_WS_DIR/fastlio2 — 跳过"
  echo "       [HINT] 克隆 FAST-LIO2 到 $ROOT_DIR/third_party/FASTLIO2_ROS2/"
else
  echo "[3/4] FAST-LIO2 构建已跳过 (NEXUS_BUILD_FASTLIO2=0)"
fi

# ── 6. 仿真耦合检查 ────────────────────────────────────────────
echo "[4/4] 仿真耦合检查..."
bash "$ROOT_DIR/scripts/check_no_sim_coupling.sh" || true

echo
echo "============================================================"
echo "[DONE] NEXUS DEPLOY 构建完成！"
echo "  启动: bash scripts/run_real_robot.sh"
echo "  停止: bash scripts/stop.sh"
echo "============================================================"
