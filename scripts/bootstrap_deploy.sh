#!/usr/bin/env bash
# ============================================================================
# NEXUS DEPLOY 一键从零部署脚本 (bootstrap_deploy.sh)
# ----------------------------------------------------------------------------
# 在一台全新 Ubuntu 22.04 机器上，从零扫描环境、安装依赖、克隆源码、
# 构建，直到可以 `bash scripts/run_real_robot.sh` 运行。
#
# 用法:
#   bash scripts/bootstrap_deploy.sh                  # 全量部署
#   bash scripts/bootstrap_deploy.sh --dry-run        # 只打印不执行
#   bash scripts/bootstrap_deploy.sh --skip-system    # 跳过所有 sudo apt 步骤
#   bash scripts/bootstrap_deploy.sh --skip-cuda      # 跳过 CUDA (无 N 卡)
#   bash scripts/bootstrap_deploy.sh --from 09        # 从阶段 09 开始
#   bash scripts/bootstrap_deploy.sh --only 12        # 只跑阶段 12
#   bash scripts/bootstrap_deploy.sh -y               # apt 自动 yes
#
# 关于"虚拟环境":
#   ROS2 Humble 的 rclpy 针对系统 Python 3.10 编译, **不能放进 venv/conda**,
#   否则 `import rclpy` 会失败。本脚本采用 ROS2 生态标准做法:
#     1. Python 包用 `pip install --user` 装到 ~/.local/ (用户隔离)
#     2. 生成 ~/.nexus_deploy_env.sh 集中管理所有环境变量 (PATH / LD_LIBRARY_PATH /
#        ROS source / workspace source), 等价于 `source activate`
#     3. 运行前 `source ~/.nexus_deploy_env.sh` 即可激活完整环境
#
# 阶段一览:
#   01 preflight     预检 (OS / 架构 / conda 警告 / sudo)
#   02 system        apt 基础工具 + ROS2 apt 源
#   03 ros2          ROS2 Humble Desktop + colcon + rosdep
#   04 cuda          CUDA Toolkit 12.x (仅当检测到 N 卡)
#   05 livox_sdk     Livox-SDK2 源码编译
#   06 sophus        Sophus 1.22.10 源码编译
#   07 gtsam         GTSAM 4.2 源码编译 (可选, 耗时长)
#   08 ros_pkgs      Nav2 / PCL / tf2 / cv_bridge 等 apt 包
#   09 python        pip --user 安装 torch / cupy / do-mpc / casadi 等
#   10 clone_deploy  克隆 DEPLOY 仓库 (若当前不在仓库内)
#   11 clone_fastlio 克隆 FAST-LIO2_ROS2 到 third_party/
#   12 build_deploy  构建 DEPLOY 主工作空间 (含 elevation_mapping_cupy ws)
#   13 build_fastlio 构建 FAST-LIO2 (lio_node)
#   14 finalize      生成 ~/.nexus_deploy_env.sh + 打印运行指引
# ============================================================================
set -euo pipefail

# ── 颜色 ───────────────────────────────────────────────────────────────────
if [ -t 1 ] && command -v tput &>/dev/null && [ "$(tput colors 2>/dev/null || echo 0)" -ge 8 ]; then
  C_RED=$(tput setaf 1); C_GREEN=$(tput setaf 2); C_YELLOW=$(tput setaf 3)
  C_BLUE=$(tput setaf 4); C_CYAN=$(tput setaf 6); C_BOLD=$(tput bold); C_RST=$(tput sgr0)
else
  C_RED=""; C_GREEN=""; C_YELLOW=""; C_BLUE=""; C_CYAN=""; C_BOLD=""; C_RST=""
fi

# ── 日志 ───────────────────────────────────────────────────────────────────
LOG_DIR="${NEXUS_BOOTSTRAP_LOG_DIR:-/tmp/nexus_bootstrap_logs}"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/bootstrap_$(date +%Y%m%d_%H%M%S).log"

log()   { echo -e "${C_BOLD}[$(date +%H:%M:%S)]${C_RST} $*" | tee -a "$LOG_FILE"; }
info()  { echo -e "${C_CYAN}[INFO ]${C_RST} $*" | tee -a "$LOG_FILE"; }
ok()    { echo -e "${C_GREEN}[  OK ]${C_RST} $*" | tee -a "$LOG_FILE"; }
warn()  { echo -e "${C_YELLOW}[WARN ]${C_RST} $*" | tee -a "$LOG_FILE"; }
fail()  { echo -e "${C_RED}[FAIL ]${C_RST} $*" | tee -a "$LOG_FILE" >&2; }
step()  { echo -e "\n${C_BOLD}${C_BLUE}════════════════════════════════════════════════════════════════${C_RST}" | tee -a "$LOG_FILE"
          echo -e "${C_BOLD}${C_BLUE}  Phase $*${C_RST}" | tee -a "$LOG_FILE"
          echo -e "${C_BOLD}${C_BLUE}════════════════════════════════════════════════════════════════${C_RST}" | tee -a "$LOG_FILE"; }

# ── 全局状态 ───────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_DIR=""                # phase_10 设置
SRC_CACHE_DIR="${NEXUS_BOOTSTRAP_SRC_DIR:-$HOME/nexus_bootstrap_src}"
SKIP_SYSTEM=0
SKIP_CUDA=0
DRY_RUN=0
AUTO_YES=0
ONLY_PHASE=""
FROM_PHASE=""
TO_PHASE=""
DEPLOY_URL="${NEXUS_DEPLOY_URL:-git@github.com:CharlesH777/NEXUS_DEPLOY.git}"
DEPLOY_URL_HTTPS="https://github.com/CharlesH777/NEXUS_DEPLOY.git"
FASTLIO_URL="${NEXUS_FASTLIO_URL:-https://github.com/yanliang-wang/FASTLIO2_ROS2.git}"
HAS_NVIDIA=0
HAS_CUDA_TOOLKIT=0
PHASES_RAN=""

# ── 工具函数 ───────────────────────────────────────────────────────────────
run() {
  # run <cmd...>  — 受 DRY_RUN 控制的执行器
  if [ "$DRY_RUN" -eq 1 ]; then
    echo -e "${C_YELLOW}[DRY ]${C_RST} $*" | tee -a "$LOG_FILE"
  else
    "$@" 2>&1 | tee -a "$LOG_FILE"
    return ${PIPESTATUS[0]}
  fi
}

sudo_apt() {
  # sudo_apt <args...>  — 自动 -y 的 apt 调用
  local args=(-y "$@")
  if [ "$AUTO_YES" -eq 1 ]; then
    sudo apt-get "${args[@]}" DEBIAN_FRONTEND=noninteractive
  else
    sudo apt-get "${args[@]}"
  fi
}

should_run_phase() {
  # should_run_phase <num>  — 根据 ONLY/FROM/TO 决定是否执行
  local n="$1"
  if [ -n "$ONLY_PHASE" ] && [ "$n" != "$ONLY_PHASE" ]; then return 1; fi
  if [ -n "$FROM_PHASE" ] && [ "$n" -lt "$FROM_PHASE" ]; then return 1; fi
  if [ -n "$TO_PHASE" ] && [ "$n" -gt "$TO_PHASE" ]; then return 1; fi
  return 0
}

phase_done() {
  # phase_done <num> <name>
  ok "Phase $1 ($2) 完成"
  PHASES_RAN="$PHASES_RAN $1"
}

sanitize_conda() {
  # 清掉 conda 污染 (类似 build_deploy.sh 的 sanitize_python_env, 但更强)
  unset PYTHONHOME PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER \
        CONDA_SHLVL CONDA_EXE CONDA_PYTHON_EXE _CE_CONDA _CE_M || true
  # 从 PATH 移除 conda 路径
  local cleaned=""
  local IFS=':'
  for p in $PATH; do
    case "$p" in
      *miniconda*|*anaconda*|*conda*) ;;
      *) cleaned="${cleaned:+$cleaned:}$p" ;;
    esac
  done
  export PATH="/usr/local/cuda/bin:/usr/local/sbin:/usr/sbin:/usr/bin:/sbin:/bin:${cleaned}"
  hash -r || true
}

detect_environment() {
  # 轻量级环境检测 — 无论是否运行 phase 01 都先做, 供后续阶段使用
  if command -v nvidia-smi &>/dev/null; then
    HAS_NVIDIA=1
  fi
  if [ -x /usr/local/cuda/bin/nvcc ]; then
    HAS_CUDA_TOOLKIT=1
  elif command -v nvcc &>/dev/null; then
    HAS_CUDA_TOOLKIT=1
  fi
}

check_sudo() {
  if ! sudo -n true 2>/dev/null && [ "$AUTO_YES" -eq 0 ]; then
    warn "后续步骤需要 sudo 权限。首次会提示密码。用 -y 可让 apt 自动 yes。"
    sudo -v || { fail "无法获取 sudo 权限"; exit 1; }
  fi
}

# ── Phase 01: 预检 ─────────────────────────────────────────────────────────
phase_01_preflight() {
  step "01 / preflight — 环境预检"

  # OS 版本
  if ! grep -q "22.04" /etc/os-release 2>/dev/null; then
    fail "本脚本只支持 Ubuntu 22.04 LTS (Jammy)。检测到: $(grep PRETTY_NAME /etc/os-release 2>/dev/null || echo unknown)"
    exit 1
  fi
  ok "Ubuntu 22.04 LTS"

  # 架构
  local arch; arch="$(dpkg --print-architecture)"
  if [ "$arch" != "amd64" ]; then
    warn "架构 $arch 未经验证, 官方只测过 amd64。继续但可能失败。"
  else
    ok "架构 amd64"
  fi

  # Python
  if ! /usr/bin/python3 -c 'import sys; exit(0 if sys.version_info >= (3,10) else 1)' 2>/dev/null; then
    fail "系统 Python 版本过低, 需要 >= 3.10。请先安装 python3.10。"
    exit 1
  fi
  ok "/usr/bin/python3 $(/usr/bin/python3 --version 2>&1)"

  # conda 检测
  if [ -n "${CONDA_PREFIX:-}" ] || command -v conda &>/dev/null; then
    warn "检测到 conda 环境 (CONDA_PREFIX=${CONDA_PREFIX:-<set>})。"
    warn "ROS2 Humble 的 rclpy 与 conda Python 不兼容, 必须清理 conda 变量。"
    warn "本脚本会自动 unset conda 变量并从 PATH 移除 conda 路径。"
    warn "建议长期方案: 不要在 conda base 里跑 ROS2, 或从 ~/.bashrc 移除 conda init。"
    sanitize_conda
    ok "conda 环境变量已清理"
  else
    ok "未检测到 conda"
  fi

  # NVIDIA GPU
  if command -v nvidia-smi &>/dev/null; then
    HAS_NVIDIA=1
    local drv; drv="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1)"
    ok "检测到 NVIDIA GPU, driver=$drv"
  else
    HAS_NVIDIA=0
    warn "未检测到 nvidia-smi。CUDA / elevation_mapping_cupy 无法运行。"
    warn "用 --skip-cuda 跳过 CUDA 安装 (但高程图管线将不可用)。"
  fi

  # CUDA toolkit
  if [ -x /usr/local/cuda/bin/nvcc ]; then
    HAS_CUDA_TOOLKIT=1
    ok "CUDA toolkit 已装: $(/usr/local/cuda/bin/nvcc --version 2>&1 | grep release)"
  elif command -v nvcc &>/dev/null; then
    HAS_CUDA_TOOLKIT=1
    ok "CUDA toolkit 已装: $(nvcc --version 2>&1 | grep release)"
  else
    HAS_CUDA_TOOLKIT=0
    if [ "$HAS_NVIDIA" -eq 1 ] && [ "$SKIP_CUDA" -eq 0 ]; then
      info "有 N 卡但无 CUDA toolkit, 将在 Phase 04 安装。"
    fi
  fi

  # sudo
  if [ "$SKIP_SYSTEM" -eq 0 ]; then
    check_sudo
  fi

  # 源码缓存目录
  mkdir -p "$SRC_CACHE_DIR"
  ok "源码缓存目录: $SRC_CACHE_DIR"

  phase_done 01 "preflight"
}

# ── Phase 02: 系统基础工具 ──────────────────────────────────────────────────
phase_02_system() {
  step "02 / system — apt 基础工具 + ROS2 apt 源"

  if [ "$SKIP_SYSTEM" -eq 1 ]; then
    warn "SKIP_SYSTEM=1, 跳过 apt 步骤。"
    phase_done 02 "system"; return
  fi

  info "apt update..."
  sudo_apt update

  info "安装基础工具..."
  sudo_apt install -y \
    curl wget git vim build-essential cmake cmake-curses-gui \
    pkg-config lsb-release gnupg2 ca-certificates \
    software-properties-common apt-transport-https \
    python3 python3-dev python3-pip python3-venv python3-tk \
    libeigen3-dev libpcl-dev libfmt-dev libyaml-cpp-dev \
    libboost-all-dev libtbb-dev libglew-dev libgl1-mesa-dev \
    libsqlite3-dev libpng-dev libjpeg-dev \
    gdb htop tmux rsync zip unzip

  # ROS2 apt 源
  if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
    info "添加 ROS2 apt 源..."
    sudo locale-gen en_US en_US.UTF-8 || true
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 || true
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo_apt update
    ok "ROS2 apt 源已添加"
  else
    ok "ROS2 apt 源已存在"
  fi

  phase_done 02 "system"
}

# ── Phase 03: ROS2 Humble ──────────────────────────────────────────────────
phase_03_ros2() {
  step "03 / ros2 — ROS2 Humble Desktop + colcon + rosdep"

  if [ "$SKIP_SYSTEM" -eq 1 ]; then
    warn "SKIP_SYSTEM=1, 跳过。"; phase_done 03 "ros2"; return
  fi

  if [ -f /opt/ros/humble/setup.bash ]; then
    ok "ROS2 Humble 已装: /opt/ros/humble/setup.bash"
  else
    info "安装 ros-humble-desktop + colcon + rosdev..."
    sudo_apt install -y \
      ros-humble-desktop \
      ros-dev-tools \
      python3-colcon-common-extensions \
      python3-rosdep python3-vcstool
  fi

  # rosdep 初始化
  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    info "初始化 rosdep..."
    sudo rosdep init || true
  fi
  if [ ! -f "$HOME/.ros/rosdep/sources.cache/index" ]; then
    info "rosdep update..."
    rosdep update || warn "rosdep update 失败, 不阻塞"
  else
    ok "rosdep 已初始化"
  fi

  # 写入 ~/.bashrc (幂等)
  if ! grep -q "source /opt/ros/humble/setup.bash" "$HOME/.bashrc" 2>/dev/null; then
    echo 'source /opt/ros/humble/setup.bash' >> "$HOME/.bashrc"
    ok "已写入 ~/.bashrc: source ROS2"
  else
    ok "~/.bashrc 已有 ROS2 source"
  fi

  phase_done 03 "ros2"
}

# ── Phase 04: CUDA Toolkit ─────────────────────────────────────────────────
phase_04_cuda() {
  step "04 / cuda — CUDA Toolkit 12.x"

  if [ "$SKIP_CUDA" -eq 1 ]; then
    warn "SKIP_CUDA=1, 跳过 CUDA。elevation_mapping_cupy 将不可用。"
    phase_done 04 "cuda"; return
  fi

  if [ "$HAS_NVIDIA" -eq 0 ]; then
    warn "无 N 卡, 跳过 CUDA。"
    phase_done 04 "cuda"; return
  fi

  if [ "$HAS_CUDA_TOOLKIT" -eq 1 ]; then
    ok "CUDA toolkit 已装, 跳过"
    phase_done 04 "cuda"; return
  fi

  if [ "$SKIP_SYSTEM" -eq 1 ]; then
    warn "SKIP_SYSTEM=1, 跳过 CUDA toolkit 安装 (需要 sudo apt)。"
    warn "但后续 torch/cupy 仍会装。确保 nvcc 在 PATH 里。"
    phase_done 04 "cuda"; return
  fi

  info "添加 CUDA 仓库并安装 cuda-toolkit-12-6..."
  local keyring="/tmp/cuda-keyring_1.1-1_all.deb"
  if [ ! -f /var/cache/apt/archives/cuda-keyring_1.1-1_all.deb ] && \
     ! dpkg -l cuda-keyring &>/dev/null; then
    wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb -O "$keyring"
    sudo dpkg -i "$keyring"
    sudo_apt update
  fi
  sudo_apt install -y cuda-toolkit-12-6

  # 环境变量写入 ~/.bashrc (幂等)
  local added=0
  if ! grep -q "CUDA_HOME=/usr/local/cuda" "$HOME/.bashrc" 2>/dev/null; then
    {
      echo ''
      echo '# ── CUDA ──'
      echo 'export CUDA_HOME=/usr/local/cuda'
      echo 'export PATH=$CUDA_HOME/bin:$PATH'
      echo 'export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$CUDA_HOME/extras/CUPTI/lib64:${LD_LIBRARY_PATH:-}'
    } >> "$HOME/.bashrc"
    added=1
  fi
  [ "$added" -eq 1 ] && ok "CUDA 环境变量已写入 ~/.bashrc" || ok "CUDA 环境变量已存在"

  # 立即生效
  export CUDA_HOME=/usr/local/cuda
  export PATH="$CUDA_HOME/bin:$PATH"
  export LD_LIBRARY_PATH="$CUDA_HOME/lib64:$CUDA_HOME/extras/CUPTI/lib64:${LD_LIBRARY_PATH:-}"

  # 建符号链接 (如果不存在)
  if [ ! -L /usr/local/cuda ] && [ -d /usr/local/cuda-12.6 ]; then
    sudo ln -sf /usr/local/cuda-12.6 /usr/local/cuda
  fi

  ok "CUDA toolkit 安装完成: $(nvcc --version 2>&1 | grep release)"
  HAS_CUDA_TOOLKIT=1
  phase_done 04 "cuda"
}

# ── Phase 05: Livox-SDK2 ───────────────────────────────────────────────────
phase_05_livox_sdk() {
  step "05 / livox_sdk — Livox-SDK2 源码编译"

  if [ -f /usr/local/lib/liblivox_lidar_sdk_shared.so ]; then
    ok "Livox-SDK2 已装: /usr/local/lib/liblivox_lidar_sdk_shared.so"
    phase_done 05 "livox_sdk"; return
  fi

  if [ "$SKIP_SYSTEM" -eq 1 ]; then
    warn "SKIP_SYSTEM=1, 跳过 (需要 sudo make install)。"
    phase_done 05 "livox_sdk"; return
  fi

  local src="$SRC_CACHE_DIR/Livox-SDK2"
  info "克隆 Livox-SDK2 → $src"
  if [ ! -d "$src" ]; then
    run git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git "$src"
  fi

  info "编译 + 安装..."
  mkdir -p "$src/build"
  (cd "$src/build" && run cmake .. -DCMAKE_BUILD_TYPE=Release && run make -j"$(nproc)" && sudo make install) \
    || { fail "Livox-SDK2 编译失败"; exit 1; }
  sudo ldconfig

  [ -f /usr/local/lib/liblivox_lidar_sdk_shared.so ] \
    && ok "Livox-SDK2 安装成功" \
    || { fail "Livox-SDK2 安装后未找到 .so"; exit 1; }
  phase_done 05 "livox_sdk"
}

# ── Phase 06: Sophus ───────────────────────────────────────────────────────
phase_06_sophus() {
  step "06 / sophus — Sophus 1.22.10 源码编译"

  if [ -d /usr/local/include/sophus ] && [ -f /usr/local/lib/cmake/Sophus/SophusConfig.cmake ]; then
    ok "Sophus 已装"
    phase_done 06 "sophus"; return
  fi

  if [ "$SKIP_SYSTEM" -eq 1 ]; then
    warn "SKIP_SYSTEM=1, 跳过 (需要 sudo make install)。"
    phase_done 06 "sophus"; return
  fi

  local src="$SRC_CACHE_DIR/Sophus"
  info "克隆 Sophus 1.22.10 → $src"
  if [ ! -d "$src" ]; then
    run git clone --depth 1 --branch 1.22.10 https://github.com/strasdat/Sophus.git "$src"
  fi

  info "编译 + 安装 (SOPHUS_USE_BASIC_LOGGING=ON)..."
  mkdir -p "$src/build"
  (cd "$src/build" \
    && run cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF \
    && run make -j"$(nproc)" && sudo make install) \
    || { fail "Sophus 编译失败"; exit 1; }

  ok "Sophus 安装成功"
  phase_done 06 "sophus"
}

# ── Phase 07: GTSAM ────────────────────────────────────────────────────────
phase_07_gtsam() {
  step "07 / gtsam — GTSAM 4.2 源码编译 (可选, 耗时长)"

  if [ -f /usr/local/lib/libgtsam.so ]; then
    ok "GTSAM 已装: /usr/local/lib/libgtsam.so"
    phase_done 07 "gtsam"; return
  fi

  if [ "$SKIP_SYSTEM" -eq 1 ]; then
    warn "SKIP_SYSTEM=1, 跳过 (需要 sudo make install)。"
    phase_done 07 "gtsam"; return
  fi

  warn "GTSAM 编译耗时较长 (10-30 分钟, 视 CPU)。FAST-LIO2 主里程计不需要, 但 pgo/hba 需要。"

  local src="$SRC_CACHE_DIR/gtsam"
  info "克隆 GTSAM 4.2 → $src"
  if [ ! -d "$src" ]; then
    run git clone --depth 1 --branch 4.2.0 https://github.com/borglab/gtsam.git "$src"
  fi

  info "编译 + 安装..."
  mkdir -p "$src/build"
  (cd "$src/build" \
    && run cmake .. \
         -DGTSAM_USE_SYSTEM_EIGEN=ON \
         -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_BUILD_UNSTABLE=ON \
         -DCMAKE_BUILD_TYPE=Release \
    && run make -j"$(nproc)" && sudo make install) \
    || { fail "GTSAM 编译失败"; exit 1; }
  sudo ldconfig

  ok "GTSAM 安装成功"
  phase_done 07 "gtsam"
}

# ── Phase 08: ROS2 系统包 ──────────────────────────────────────────────────
phase_08_ros_pkgs() {
  step "08 / ros_pkgs — Nav2 / PCL / tf2 / cv_bridge 等"

  if [ "$SKIP_SYSTEM" -eq 1 ]; then
    warn "SKIP_SYSTEM=1, 跳过。"; phase_done 08 "ros_pkgs"; return
  fi

  # 幂等: 检查关键包
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  local need_install=0
  for pkg in nav2_bt_navigator nav2_controller nav2_planner pcl_ros pcl_conversions tf2_ros; do
    if ! ros2 pkg list 2>/dev/null | grep -q "^${pkg}$"; then
      need_install=1; break
    fi
  done

  if [ "$need_install" -eq 0 ]; then
    ok "ROS2 系统包已装"
  else
    info "安装 ROS2 系统包..."
    sudo_apt install -y \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-nav2-mppi-controller \
      ros-humble-nav2-common \
      ros-humble-nav2-msgs \
      ros-humble-pcl-ros \
      ros-humble-pcl-conversions \
      ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-ros-py \
      ros-humble-tf2-geometry-msgs ros-humble-tf2-sensor-msgs \
      ros-humble-tf-transformations \
      ros-humble-message-filters \
      ros-humble-cv-bridge \
      ros-humble-rosbag2 ros-humble-rosbag2-py \
      ros-humble-rviz2 ros-humble-rviz-common \
      ros-humble-visualization-msgs \
      ros-humble-image-transport ros-humble-image-transport-plugins \
      ros-humble-composition ros-humble-rclcpp-components \
      ros-humble-launch ros-humble-launch-ros \
      ros-humble-ament-cmake ros-humble-ament-cmake-auto \
      ros-humble-ament-cmake-python ros-humble-ament-lint-auto \
      ros-humble-ament-lint-common ros-humble-rosidl-default-generators \
      ros-humble-rosidl-default-runtime \
      ros-humble-pluginlib ros-humble-filters
    # 注意: grid_map 不在这里装 — 由 elevation_mapping_cupy_ws 源码构建
  fi

  phase_done 08 "ros_pkgs"
}

# ── Phase 09: Python 依赖 ──────────────────────────────────────────────────
phase_09_python() {
  step "09 / python — pip --user 安装 (torch / cupy / do-mpc / casadi / ...)"

  # 清 conda 污染, 确保用系统 python3
  sanitize_conda
  local PY="/usr/bin/python3"

  info "升级 pip..."
  run $PY -m pip install --user --upgrade pip setuptools wheel

  # apt 里的 Python 包 (更快, 更稳)
  if [ "$SKIP_SYSTEM" -eq 0 ]; then
    info "安装 apt Python 包..."
    sudo_apt install -y \
      python3-numpy python3-scipy python3-yaml \
      python3-opencv python3-shapely \
      python3-ruamel.yaml python3-pytest \
      python3-matplotlib
  fi

  # torch — 按 CUDA 版本选 wheel
  info "安装 torch (CUDA wheel)..."
  local torch_url="https://download.pytorch.org/whl/cu124"
  if [ "$HAS_NVIDIA" -eq 1 ]; then
    local drv; drv="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1 | cut -d. -f1)"
    if [ -n "$drv" ] && [ "$drv" -ge 550 ]; then
      torch_url="https://download.pytorch.org/whl/cu128"
    elif [ -n "$drv" ] && [ "$drv" -ge 545 ]; then
      torch_url="https://download.pytorch.org/whl/cu126"
    fi
    info "driver=$drv → torch wheel: $torch_url"
  else
    warn "无 N 卡, 装 CPU 版 torch (elevation_mapping_cupy 将无法运行)"
    torch_url=""
  fi
  if [ -n "$torch_url" ]; then
    run $PY -m pip install --user torch --index-url "$torch_url"
  else
    run $PY -m pip install --user torch
  fi

  info "安装 cupy-cuda12x / do-mpc / casadi / gpytorch / 其他..."
  if [ "$HAS_NVIDIA" -eq 1 ]; then
    run $PY -m pip install --user cupy-cuda12x
  fi
  run $PY -m pip install --user do-mpc casadi gpytorch transforms3d simple-parsing ros2-numpy PyYAML

  # 验证 (dry-run 下跳过, 因为没真装)
  if [ "$DRY_RUN" -eq 1 ]; then
    warn "DRY_RUN=1, 跳过 Python 模块验证"
    phase_done 09 "python"; return
  fi
  info "验证 Python 依赖..."
  local missing=0
  for mod in numpy scipy do_mpc casadi yaml tkinter transforms3d simple_parsing ros2_numpy; do
    if ! $PY -c "import $mod" 2>/dev/null; then
      fail "Python 模块缺失: $mod"; missing=$((missing+1))
    fi
  done
  if [ "$HAS_NVIDIA" -eq 1 ]; then
    for mod in torch cupy gpytorch; do
      if ! $PY -c "import $mod" 2>/dev/null; then
        fail "Python 模块缺失: $mod"; missing=$((missing+1))
      fi
    done
    if ! $PY -c "import torch; assert torch.cuda.is_available()" 2>/dev/null; then
      warn "torch.cuda.is_available() 返回 False — 检查驱动 / CUDA 版本"
    fi
  fi
  [ "$missing" -eq 0 ] && ok "所有 Python 依赖验证通过" || { fail "有 $missing 个 Python 模块缺失"; exit 1; }

  phase_done 09 "python"
}

# ── Phase 10: 克隆 DEPLOY 仓库 ─────────────────────────────────────────────
phase_10_clone_deploy() {
  step "10 / clone_deploy — 定位/克隆 DEPLOY 仓库"

  # 检测当前是否已在 DEPLOY 仓库内
  if [ -f "$SCRIPT_DIR/build_deploy.sh" ] && [ -d "$SCRIPT_DIR/../src" ]; then
    DEPLOY_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
    ok "已在 DEPLOY 仓库内: $DEPLOY_DIR"
    phase_done 10 "clone_deploy"; return
  fi

  DEPLOY_DIR="${NEXUS_DEPLOY_DIR:-$HOME/NEXUS_DEPLOY}"
  if [ -d "$DEPLOY_DIR/.git" ] && [ -f "$DEPLOY_DIR/scripts/build_deploy.sh" ]; then
    ok "DEPLOY 仓库已存在: $DEPLOY_DIR"
  else
    info "克隆 DEPLOY → $DEPLOY_DIR"
    # 优先 SSH, 失败回退 HTTPS
    if ! run git clone "$DEPLOY_URL" "$DEPLOY_DIR"; then
      warn "SSH 克隆失败, 尝试 HTTPS..."
      run git clone "$DEPLOY_URL_HTTPS" "$DEPLOY_DIR"
    fi
  fi

  SCRIPT_DIR="$DEPLOY_DIR/scripts"
  ok "DEPLOY_DIR=$DEPLOY_DIR"
  phase_done 10 "clone_deploy"
}

# ── Phase 11: 克隆 FAST-LIO2 ───────────────────────────────────────────────
phase_11_clone_fastlio() {
  step "11 / clone_fastlio — 克隆 FAST-LIO2_ROS2 到 third_party/"

  local fastlio_dir="$DEPLOY_DIR/third_party/FASTLIO2_ROS2"

  # 检查是否已有源码 (不是空 gitlink)
  if [ -d "$fastlio_dir/fastlio2" ] && [ -f "$fastlio_dir/fastlio2/CMakeLists.txt" ]; then
    ok "FAST-LIO2 源码已存在: $fastlio_dir"
    phase_done 11 "clone_fastlio"; return
  fi

  info "FAST-LIO2 目录为空 (submodule gitlink 无 .gitmodules), 克隆源码..."
  run rm -rf "$fastlio_dir"
  if ! run git clone "$FASTLIO_URL" "$fastlio_dir"; then
    fail "克隆 FAST-LIO2 失败: $FASTLIO_URL"
    warn "请手动指定 fork:  bash scripts/bootstrap_deploy.sh --fastlio-url <url>"
    exit 1
  fi

  [ -f "$fastlio_dir/fastlio2/CMakeLists.txt" ] \
    && ok "FAST-LIO2 克隆成功" \
    || { fail "克隆后未找到 fastlio2/CMakeLists.txt"; exit 1; }
  phase_done 11 "clone_fastlio"
}

# ── Phase 12: 构建 DEPLOY ──────────────────────────────────────────────────
phase_12_build_deploy() {
  step "12 / build_deploy — 构建 DEPLOY 主工作空间 (含 elevation ws)"

  [ -f "$DEPLOY_DIR/scripts/build_deploy.sh" ] \
    || { fail "未找到 $DEPLOY_DIR/scripts/build_deploy.sh"; exit 1; }

  # 确保 CUDA 在 PATH (build 需要 nvcc)
  export CUDA_HOME=/usr/local/cuda
  export PATH="$CUDA_HOME/bin:$PATH"
  export LD_LIBRARY_PATH="$CUDA_HOME/lib64:${LD_LIBRARY_PATH:-}"

  info "运行 build_deploy.sh (会自动构建 elevation_mapping_cupy ws + colcon build src/)"
  run bash "$DEPLOY_DIR/scripts/build_deploy.sh" --skip-env \
    || { fail "build_deploy.sh 失败, 见日志: $LOG_FILE"; exit 1; }

  [ -f "$DEPLOY_DIR/install/setup.bash" ] \
    && ok "DEPLOY 工作空间构建成功" \
    || { fail "构建后未找到 install/setup.bash"; exit 1; }
  phase_done 12 "build_deploy"
}

# ── Phase 13: 构建 FAST-LIO2 ───────────────────────────────────────────────
phase_13_build_fastlio() {
  step "13 / build_fastlio — 构建 FAST-LIO2 (lio_node)"

  [ -f "$DEPLOY_DIR/scripts/build_fastlio.sh" ] \
    || { fail "未找到 build_fastlio.sh"; exit 1; }

  local fastlio_bin="$DEPLOY_DIR/third_party/FASTLIO2_ROS2/install_nexus/fastlio2/lib/fastlio2/lio_node"
  if [ -x "$fastlio_bin" ]; then
    ok "FAST-LIO2 已构建: $fastlio_bin"
    phase_done 13 "build_fastlio"; return
  fi

  export CUDA_HOME=/usr/local/cuda
  export PATH="$CUDA_HOME/bin:$PATH"

  info "运行 build_fastlio.sh..."
  run bash "$DEPLOY_DIR/scripts/build_fastlio.sh" \
    || { fail "build_fastlio.sh 失败"; exit 1; }

  [ -x "$fastlio_bin" ] \
    && ok "FAST-LIO2 构建成功: $fastlio_bin" \
    || { fail "构建后未找到 lio_node"; exit 1; }
  phase_done 13 "build_fastlio"
}

# ── Phase 14: 收尾 ─────────────────────────────────────────────────────────
phase_14_finalize() {
  step "14 / finalize — 生成环境激活脚本 + 运行指引"

  # nvidia wheel lib 路径 (torch/cupy 自带 CUDA 库)
  local nvidia_lib_paths=""
  for d in /usr/local/lib/python3.10/dist-packages/nvidia/*/lib \
           "$HOME/.local/lib/python3.10/site-packages/nvidia"/*/lib; do
    [ -d "$d" ] && nvidia_lib_paths="$nvidia_lib_paths:$d"
  done

  local env_sh="$HOME/.nexus_deploy_env.sh"
  cat > "$env_sh" << ENV_EOF
# ============================================================================
# NEXUS DEPLOY 环境激活脚本 — 由 bootstrap_deploy.sh 生成 $(date)
# 用法:  source ~/.nexus_deploy_env.sh
# ============================================================================
# 清掉 conda 污染
unset PYTHONHOME PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER \\
      CONDA_SHLVL CONDA_EXE CONDA_PYTHON_EXE _CE_CONDA _CE_M || true

# ROS2 Humble
source /opt/ros/humble/setup.bash

# CUDA
export CUDA_HOME=/usr/local/cuda
export PATH=\$CUDA_HOME/bin:\$PATH
export LD_LIBRARY_PATH=\$CUDA_HOME/lib64:\$CUDA_HOME/extras/CUPTI/lib64:\${LD_LIBRARY_PATH:-}

# torch / cupy 自带 nvidia 库
export LD_LIBRARY_PATH="${nvidia_lib_paths#:}:\${LD_LIBRARY_PATH:-}"

# DEPLOY workspace
source "$DEPLOY_DIR/tools/elevation_mapping_cupy_ros2_ws/install/setup.bash"
source "$DEPLOY_DIR/install/setup.bash"

# 提示
echo "[NEXUS DEPLOY] 环境已激活。运行:  bash $DEPLOY_DIR/scripts/run_real_robot.sh"
ENV_EOF

  ok "环境激活脚本已生成: $env_sh"

  # 跑一次 check_env.sh 做最终验证
  if [ -f "$DEPLOY_DIR/scripts/check_env.sh" ]; then
    info "运行 check_env.sh 做最终验证..."
    bash "$DEPLOY_DIR/scripts/check_env.sh" 2>&1 | tee -a "$LOG_FILE" || true
  fi

  # 打印指引
  cat << BANNER | tee -a "$LOG_FILE"

${C_BOLD}${C_GREEN}================================================================${C_RST}
${C_BOLD}${C_GREEN}  NEXUS DEPLOY 部署完成!${C_RST}
${C_BOLD}${C_GREEN}================================================================${C_RST}

  已完成的阶段:${C_GREEN}$PHASES_RAN${C_RST}

  仓库路径:   $DEPLOY_DIR
  构建产物:   $DEPLOY_DIR/install/
  FAST-LIO2:  $DEPLOY_DIR/third_party/FASTLIO2_ROS2/install_nexus/
  日志文件:   $LOG_FILE

  ${C_BOLD}运行前必做${C_RST}:
    1. 配置 Livox MID360 网络 (静态 IP 192.168.1.50, 见 docs/SETUP_FROM_SCRATCH.md §11)
    2. 设置雷达 bd_code:
         export NEXUS_LIVOX_BD_CODE=<你的雷达 15 位序列号>

  ${C_BOLD}激活环境${C_RST} (新终端每次都要):
    source ~/.nexus_deploy_env.sh

  ${C_BOLD}启动部署栈${C_RST}:
    cd $DEPLOY_DIR
    bash scripts/run_real_robot.sh

  ${C_BOLD}常用变体${C_RST}:
    NEXUS_ENABLE_RVIZ=true bash scripts/run_real_robot.sh          # 带 RViz
    NEXUS_ENABLE_SAND_MPC=false bash scripts/run_real_robot.sh     # 关闭 sand MPC
    bash scripts/stop.sh                                            # 停止

  ${C_BOLD}集成测试 (可选, 用 Gazebo 喂数据)${C_RST}:
    MAP_SIM_GZCLIENT=0 timeout 240 bash scripts/test_with_sim_data.sh

  ${C_BOLD}完整文档${C_RST}:  $DEPLOY_DIR/docs/SETUP_FROM_SCRATCH.md
BANNER

  phase_done 14 "finalize"
}

# ── 参数解析 ───────────────────────────────────────────────────────────────
usage() {
  cat << USAGE
NEXUS DEPLOY 一键从零部署脚本

用法:
  bash scripts/bootstrap_deploy.sh [选项]

选项:
  --dry-run           只打印命令不执行
  --skip-system       跳过所有 sudo apt 步骤 (假设 ROS2/系统包已装)
  --skip-cuda         跳过 CUDA toolkit 安装 (无 N 卡)
  --skip-gtsam        跳过 GTSAM 编译 (FAST-LIO2 主里程计不需要, 但 pgo/hba 需要)
  -y, --yes           apt 自动 yes, sudo 不提示
  --only <num>        只跑指定阶段 (01-14)
  --from <num>        从指定阶段开始
  --to <num>          跑到指定阶段结束
  --deploy-url <url>  DEPLOY 仓库 URL (默认: $DEPLOY_URL)
  --fastlio-url <url> FAST-LIO2 fork URL (默认: $FASTLIO_URL)
  --deploy-dir <path> DEPLOY 仓库本地路径 (默认: ~/NEXUS_DEPLOY 或当前目录)
  -h, --help          显示本帮助

阶段:
  01 preflight        02 system         03 ros2           04 cuda
  05 livox_sdk        06 sophus         07 gtsam          08 ros_pkgs
  09 python           10 clone_deploy   11 clone_fastlio  12 build_deploy
  13 build_fastlio    14 finalize

示例:
  # 全量部署
  bash scripts/bootstrap_deploy.sh

  # 只装 Python 依赖
  bash scripts/bootstrap_deploy.sh --only 09

  # 跳过系统步骤, 只构建
  bash scripts/bootstrap_deploy.sh --skip-system --from 10

  # 跳过 GTSAM (省 10-30 分钟)
  bash scripts/bootstrap_deploy.sh --skip-gtsam
USAGE
}

SKIP_GTSAM=0
while [ "$#" -gt 0 ]; do
  case "$1" in
    --dry-run)        DRY_RUN=1; shift ;;
    --skip-system)    SKIP_SYSTEM=1; shift ;;
    --skip-cuda)      SKIP_CUDA=1; shift ;;
    --skip-gtsam)     SKIP_GTSAM=1; shift ;;
    -y|--yes)         AUTO_YES=1; shift ;;
    --only)           ONLY_PHASE="$2"; shift 2 ;;
    --from)           FROM_PHASE="$2"; shift 2 ;;
    --to)             TO_PHASE="$2"; shift 2 ;;
    --deploy-url)     DEPLOY_URL="$2"; shift 2 ;;
    --fastlio-url)    FASTLIO_URL="$2"; shift 2 ;;
    --deploy-dir)     DEPLOY_DIR="$2"; SCRIPT_DIR="$2/scripts"; shift ;;
    -h|--help)        usage; exit 0 ;;
    *)                fail "未知参数: $1"; usage; exit 1 ;;
  esac
done

# ── 主流程 ─────────────────────────────────────────────────────────────────
# 先做轻量级环境检测, 供后续阶段使用 (即使 --from 跳过 phase 01 也能正确判断)
detect_environment

# 如果当前在 DEPLOY 仓库内, 提前设置 DEPLOY_DIR (即使跳过 phase 10 也能用)
if [ -z "$DEPLOY_DIR" ] && [ -f "$SCRIPT_DIR/build_deploy.sh" ] && [ -d "$SCRIPT_DIR/../src" ]; then
  DEPLOY_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
elif [ -z "$DEPLOY_DIR" ] && [ -f "$SCRIPT_DIR/../scripts/build_deploy.sh" ]; then
  DEPLOY_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
fi

echo -e "${C_BOLD}${C_CYAN}"
echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║  NEXUS DEPLOY 一键从零部署                                        ║"
echo "║  日志: $LOG_FILE"
echo "╚══════════════════════════════════════════════════════════════════╝"
echo -e "${C_RST}"

# 如果 DEPLOY_DIR 已由 --deploy-dir 或 phase_10 设置, 跳过 10
if [ -n "$DEPLOY_DIR" ] && [ -f "$DEPLOY_DIR/scripts/build_deploy.sh" ]; then
  SCRIPT_DIR="$DEPLOY_DIR/scripts"
fi

# 阶段执行
if should_run_phase 01; then phase_01_preflight; fi
if should_run_phase 02; then phase_02_system; fi
if should_run_phase 03; then phase_03_ros2; fi
if should_run_phase 04; then phase_04_cuda; fi
if should_run_phase 05; then phase_05_livox_sdk; fi
if should_run_phase 06; then phase_06_sophus; fi
if should_run_phase 07 && [ "$SKIP_GTSAM" -eq 0 ]; then
  phase_07_gtsam
elif should_run_phase 07 && [ "$SKIP_GTSAM" -eq 1 ]; then
  warn "SKIP_GTSAM=1, 跳过 GTSAM 编译"
fi
if should_run_phase 08; then phase_08_ros_pkgs; fi
if should_run_phase 09; then phase_09_python; fi
if should_run_phase 10; then phase_10_clone_deploy; fi
if should_run_phase 11; then phase_11_clone_fastlio; fi
if should_run_phase 12; then phase_12_build_deploy; fi
if should_run_phase 13; then phase_13_build_fastlio; fi
if should_run_phase 14; then phase_14_finalize; fi

echo -e "${C_BOLD}${C_GREEN}全部完成。日志: $LOG_FILE${C_RST}"
