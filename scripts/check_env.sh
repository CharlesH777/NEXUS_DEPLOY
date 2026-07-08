#!/usr/bin/env bash
set -euo pipefail

# NEXUS DEPLOY environment checker — verifies all build/run dependencies.
# Usage: bash scripts/check_env.sh

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
PASS=0
FAIL=0
WARN=0

ok()   { echo "  [OK]   $1"; PASS=$((PASS+1)); }
miss() { echo "  [MISS] $1"; FAIL=$((FAIL+1)); }
warn() { echo "  [WARN] $1"; WARN=$((WARN+1)); }

echo "==================== NEXUS DEPLOY 环境检查 ===================="
echo

# ── 1. ROS2 Humble ──────────────────────────────────────────────
echo "── 1. ROS2 Humble"
if [ -f "$ROS_SETUP" ]; then ok "$ROS_SETUP"; else miss "$ROS_SETUP — 请安装 ROS2 Humble"; fi
if command -v ros2 &>/dev/null; then ok "ros2 命令可用"; else miss "ros2 命令 — source $ROS_SETUP"; fi

# ── 2. Python 3.10+ ────────────────────────────────────────────
echo "── 2. Python >= 3.10"
if /usr/bin/python3 -c 'import sys; exit(0 if sys.version_info >= (3,10) else 1)' 2>/dev/null; then
  ok "/usr/bin/python3 $(/usr/bin/python3 --version 2>&1)"
else
  miss "/usr/bin/python3 版本过低 (需要 >=3.10，代码用了 PEP 604 语法)"
fi

# ── 3. Colcon / build tools ────────────────────────────────────
echo "── 3. 构建工具"
for cmd in colcon cmake make; do
  if command -v "$cmd" &>/dev/null; then ok "$cmd"; else miss "$cmd"; fi
done

# ── 4. Livox SDK ───────────────────────────────────────────────
echo "── 4. Livox SDK (livox_ros_driver2 依赖)"
if [ -f /usr/local/lib/liblivox_lidar_sdk_shared.so ]; then
  ok "/usr/local/lib/liblivox_lidar_sdk_shared.so"
elif [ -f /usr/local/lib/liblivox_lidar_sdk_static.a ]; then
  ok "/usr/local/lib/liblivox_lidar_sdk_static.a"
else
  miss "Livox-SDK2 — 请编译安装到 /usr/local/lib"
fi
if [ -f /usr/local/include/livox_lidar_api.h ]; then
  ok "/usr/local/include/livox_lidar_api.h"
else
  miss "livox_lidar_api.h 头文件"
fi

# ── 5. CUDA (elevation_mapping_cupy 依赖) ──────────────────────
echo "── 5. CUDA (elevation_mapping_cupy 依赖)"
if command -v nvcc &>/dev/null; then
  ok "nvcc $(nvcc --version 2>&1 | grep release || echo)"
else
  warn "nvcc 未找到 — 没有 CUDA 则无法运行高程图管线"
fi

# ── 6. ROS 包 (apt 安装) ───────────────────────────────────────
echo "── 6. ROS2 系统包"
if [ -f "$ROS_SETUP" ]; then
  set +u; source "$ROS_SETUP" 2>/dev/null; set -u
fi
for pkg in nav2_bt_navigator nav2_controller nav2_planner pcl_ros pcl_conversions tf2_ros; do
  if ros2 pkg list 2>/dev/null | grep -q "^${pkg}$"; then ok "ros2 pkg: $pkg"; else miss "ros2 pkg: $pkg — sudo apt install ros-humble-$(echo $pkg | tr _ -)"; fi
done

# ── 7. Python 第三方包 ─────────────────────────────────────────
echo "── 7. Python 第三方包 (运行时依赖)"
PY_CHECK="/usr/bin/python3 -c"
check_py() {
  local mod="$1" pip_name="$2"
  if $PY_CHECK "import $mod" 2>/dev/null; then ok "$mod ($pip_name)"; else miss "$mod — pip3 install $pip_name"; fi
}
check_py numpy numpy
check_py scipy scipy
check_py do_mpc "do-mpc"
check_py casadi casadi
check_py yaml PyYAML
if /usr/bin/python3 -c "import tkinter" 2>/dev/null; then ok "tkinter (python3-tk)"; else miss "tkinter — sudo apt install python3-tk"; fi
# torch/gpytorch 只在 GP 建图时需要
if /usr/bin/python3 -c "import torch" 2>/dev/null; then ok "torch (GP 建图)"; else warn "torch — GP 建图需要: pip3 install torch"; fi
if /usr/bin/python3 -c "import gpytorch" 2>/dev/null; then ok "gpytorch (GP 建图)"; else warn "gpytorch — GP 建图需要: pip3 install gpytorch"; fi

# ── 8. FAST-LIO2 源码 ──────────────────────────────────────────
echo "── 8. FAST-LIO2 源码"
FASTLIO_DIR="$ROOT_DIR/third_party/FASTLIO2_ROS2"
if [ -L "$FASTLIO_DIR" ]; then
  warn "$FASTLIO_DIR 是符号链接 → $(readlink "$FASTLIO_DIR")"
  warn "  拷贝到其他机器会断裂。建议: rm $FASTLIO_DIR && cp -r <源码> $FASTLIO_DIR"
elif [ -d "$FASTLIO_DIR/fastlio2" ]; then
  ok "$FASTLIO_DIR/fastlio2 源码存在"
else
  miss "$FASTLIO_DIR/fastlio2 — 请克隆 FAST-LIO2 源码到此目录"
fi

# ── 9. elevation_mapping_cupy workspace ────────────────────────
echo "── 9. elevation_mapping_cupy workspace"
ELEV_WS="$ROOT_DIR/tools/elevation_mapping_cupy_ros2_ws"
if [ -d "$ELEV_WS/src/elevation_mapping_cupy" ]; then ok "$ELEV_WS/src/elevation_mapping_cupy 源码存在"; else miss "elevation_mapping_cupy 源码 — 运行 build_deploy.sh 会自动克隆"; fi

# ── 10. 构建产物残留检查 ───────────────────────────────────────
echo "── 10. 构建产物残留 (应清理后再拷贝)"
for d in build install log; do
  if [ -d "$ROOT_DIR/$d" ]; then warn "$ROOT_DIR/$d 存在 — 拷贝前应清理"; fi
done
PYC_COUNT=$(find "$ROOT_DIR/src" -name "*.pyc" 2>/dev/null | wc -l)
if [ "$PYC_COUNT" -gt 0 ]; then warn "src/ 下有 $PYC_COUNT 个 .pyc 文件 — 拷贝前应清理"; else ok "src/ 无 .pyc 残留"; fi

echo
echo "==================== 检查结果 ===================="
echo "  通过: $PASS   缺失: $FAIL   警告: $WARN"
echo
if [ "$FAIL" -gt 0 ]; then
  echo "[ERR] 有 $FAIL 项依赖缺失，请先安装再构建。"
  exit 1
fi
echo "[OK] 环境检查通过，可以构建。"
