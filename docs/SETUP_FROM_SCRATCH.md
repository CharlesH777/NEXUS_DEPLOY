# NEXUS DEPLOY 从零部署指南

> 目标：在一台**全新 Ubuntu 22.04 LTS** 系统上，从无到有完成 NEXUS DEPLOY 真机部署栈的依赖安装、源码拉取、构建和运行。
>
> 本文档基于以下已验证版本组合：
> - Ubuntu 22.04.5 LTS (Jammy)
> - NVIDIA Driver ≥ 535（实测 580.159.03，CUDA Capability 13.0）
> - CUDA Toolkit 12.6（任何 12.x 均可，与 `cupy-cuda12x` 对应）
> - Python 3.10.12（系统自带）
> - ROS2 Humble
> - GTSAM 4.2.0 / Sophus 1.22.10 / Livox-SDK2（源码安装）
> - torch 2.10.0+cu128 + cupy-cuda12x 12.2.0 + gpytorch 1.11
>
> 全程**不需要 conda**。如果机器上有 conda，构建脚本会主动清理 conda 环境变量，但仍建议不要在 conda base 里跑。

---

## 0. 硬件与系统前提

| 项 | 要求 | 备注 |
|---|---|---|
| OS | Ubuntu 22.04 LTS (Jammy) | 20.04 / 24.04 不行，ROS2 Humble 官方只支持 22.04 |
| 架构 | x86_64 (amd64) | arm64 需自行调整 CUDA / torch wheel |
| GPU | NVIDIA GPU，显存 ≥ 4 GB | 高程图管线必须用 CUDA；无 N 卡可跑除 elevation 外的链路 |
| NVIDIA Driver | ≥ 535 | 用 `ubuntu-drivers devices` 选 `nvidia-driver-535` 或更新 |
| 网卡 | 有线网口，可设静态 IP 192.168.1.50 | 用于和 Livox MID360 直连 |
| LiDAR | Livox MID360 | 默认 IP 192.168.1.3，配置文件已内置 |
| sudo 权限 | 必需 | 大量 apt 安装 |

```bash
lsb_release -a                         # 确认 Ubuntu 22.04
uname -m                               # 确认 x86_64
nvidia-smi                             # 确认驱动 + CUDA Version
ubuntu-drivers devices                 # 查看推荐驱动
```

---

## 1. 系统更新 + 基础工具

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
  curl wget git vim build-essential cmake cmake-curses-gui \
  pkg-config lsb-release gnupg2 ca-certificates \
  software-properties-common apt-transport-https \
  python3 python3-dev python3-pip python3-venv python3-tk \
  libeigen3-dev libpcl-dev libfmt-dev libyaml-cpp-dev \
  libboost-all-dev libtbb-dev libglew-dev libgl1-mesa-dev \
  libsqlite3-dev libpng-dev libjpeg-dev \
  gdb htop tmux rsync zip unzip
```

确认 Python 版本 ≥ 3.10：

```bash
/usr/bin/python3 --version             # 应输出 Python 3.10.x
```

---

## 2. 安装 ROS2 Humble

按官方文档安装 Desktop 版（含 RViz）：

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository universe -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep python3-vcstool
```

初始化 rosdep（一次即可）：

```bash
sudo rosdep init || true
rosdep update
```

把 ROS source 写入 `~/.bashrc`：

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source /opt/ros/humble/setup.bash
ros2 --help                                    # 验证
```

---

## 3. 安装 CUDA Toolkit 12.x

elevation_mapping_cupy 依赖 `cupy-cuda12x`，必须有 CUDA 12.x toolkit（不只是 driver）。

### 3.1 安装 toolkit

从 NVIDIA 官网下载 runfile 或用 apt。推荐 apt 方式：

```bash
# 添加 CUDA 仓库（keyring 方式）
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update

# 安装 CUDA 12.6（也可选 12.4 / 12.5）
sudo apt install -y cuda-toolkit-12-6
```

### 3.2 配置环境变量

把以下内容追加到 `~/.bashrc`：

```bash
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$CUDA_HOME/extras/CUPTI/lib64:$LD_LIBRARY_PATH
```

立即生效并验证：

```bash
source ~/.bashrc
nvcc --version                                 # 应输出 12.6
nvidia-smi                                     # driver + CUDA Version
```

> ⚠️ 如果 `nvcc` 找不到，确认 `/usr/local/cuda` 是指向 `/usr/local/cuda-12.6` 的符号链接：
> ```bash
> ls -l /usr/local/cuda
> # 如果不是，手动建：
> sudo ln -sf /usr/local/cuda-12.6 /usr/local/cuda
> ```

---

## 4. 安装 Livox-SDK2

`livox_ros_driver2` 链接 `liblivox_lidar_sdk_shared.so`，必须先装 SDK。

```bash
cd ~/Downloads
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

验证：

```bash
ls /usr/local/lib/liblivox_lidar_sdk_shared.so     # 必须存在
ls /usr/local/lib/liblivox_lidar_sdk_static.a      # 必须存在
ls /usr/local/include/livox_lidar_api.h            # 必须存在
sudo ldconfig
```

---

## 5. 安装 Sophus 1.22.10

FAST-LIO2 强依赖 Sophus 1.22.10，且必须用 `SOPHUS_USE_BASIC_LOGGING=ON` 关掉 fmt 依赖。

```bash
cd ~/Downloads
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
make -j$(nproc)
sudo make install
```

验证：

```bash
ls /usr/local/include/sophus/                     # 应有大量 .hpp
ls /usr/local/lib/cmake/Sophus/                   # cmake config
```

---

## 6. 安装 GTSAM 4.2

FAST-LIO2 的 `pgo` / `hba` 子包需要 GTSAM（PGO 回环优化）。Ubuntu 22.04 apt 没有 `libgtsam-dev`，必须源码编译。

```bash
cd ~/Downloads
git clone --branch 4.2.0 https://github.com/borglab/gtsam.git
cd gtsam
mkdir build && cd build
cmake .. \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_UNSTABLE=ON \
  -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig
```

验证：

```bash
ls /usr/local/lib/libgtsam.so                     # 必须存在
ls /usr/local/lib/libgtsam.so.4                   # 必须存在
ls /usr/local/include/gtsam/                      # 头文件目录
```

> ℹ️ 只跑 FAST-LIO2 主里程计（`lio_node`）不需要 GTSAM。但若要用 `pgo`（回环）或 `hba`（地图一致性优化），必须装。DEPLOY 默认只跑 `lio_node`，GTSAM 可选，但强烈建议装上避免后续构建失败。

---

## 7. 安装 ROS2 系统包

DEPLOY 用到 Nav2 全栈、PCL、grid_map、tf2 等。一次性装齐：

```bash
sudo apt install -y \
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
```

验证关键包：

```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep -E "^(nav2_bt_navigator|nav2_controller|nav2_planner|pcl_ros|pcl_conversions|tf2_ros)$"
```

> ℹ️ **不要** apt 安装 `ros-humble-grid-map*`。`grid_map` 由第 12 节 `build_deploy.sh` 在 `tools/elevation_mapping_cupy_ros2_ws/` 内从源码克隆构建（`grid_map` humble 分支），与 apt 版本会冲突。

---

## 8. 安装 Python 依赖

### 8.1 系统 Python 包（apt）

```bash
sudo apt install -y \
  python3-numpy python3-scipy python3-yaml \
  python3-opencv python3-shapely \
  python3-ruamel.yaml python3-pytest \
  python3-matplotlib
```

### 8.2 pip 包（user site）

```bash
# 升级 pip
python3 -m pip install --user --upgrade pip setuptools wheel

# torch + cupy（CUDA 12.x 版本）
# torch wheel 的 CUDA 标签只需 ≤ 驱动支持的 CUDA 版本即可（wheel 自带 runtime）
# 驱动 535+ 装 cu124；驱动 545+ 装 cu126；驱动 550+ 装 cu128
# 实测驱动 580 + torch 2.10.0+cu128 通过
python3 -m pip install --user \
  torch --index-url https://download.pytorch.org/whl/cu128

python3 -m pip install --user cupy-cuda12x

# 高程图 ws 构建脚本会用到
python3 -m pip install --user ros2-numpy transforms3d simple-parsing

# sand_mpc 依赖
python3 -m pip install --user do-mpc casadi

# GP 建图（可选，默认未启用）
python3 -m pip install --user gpytorch
```

验证：

```bash
/usr/bin/python3 -c "import torch; print('torch', torch.__version__, 'cuda', torch.cuda.is_available())"
/usr/bin/python3 -c "import cupy; print('cupy', cupy.__version__)"
/usr/bin/python3 -c "import do_mpc, casadi, gpytorch, transforms3d, simple_parsing, ros2_numpy; print('all ok')"
/usr/bin/python3 -c "import tkinter; print('tkinter ok')"
```

> ⚠️ `torch.cuda.is_available()` 必须返回 `True`，否则高程图无法运行。
>
> ⚠️ 不要用 conda 装 torch / cupy —— 路径会和 ROS 的 Python 冲突，构建脚本会主动 unset conda 变量。

### 8.3 CUDA 运行时库路径（重要）

torch / cupy 的 wheel 自带 CUDA 库，但运行时某些库（如 `libcusparseLt.so.0`、`libnvshmem_host.so.3`）需要显式暴露。把以下内容加到 `~/.bashrc`：

```bash
# torch / cupy 自带 nvidia 库
export LD_LIBRARY_PATH="/usr/local/lib/python3.10/dist-packages/nvidia/*/lib:${LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="$HOME/.local/lib/python3.10/site-packages/nvidia/*/lib:${LD_LIBRARY_PATH}"
# CUDA toolkit
export LD_LIBRARY_PATH="/usr/local/cuda/targets/x86_64-linux/lib:/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"
```

注意 `*` 通配需要 bash 在 source 时展开；如果 `~/.bashrc` 里不展开，改成显式路径：

```bash
for d in /usr/local/lib/python3.10/dist-packages/nvidia/*/lib \
         $HOME/.local/lib/python3.10/site-packages/nvidia/*/lib; do
  [ -d "$d" ] && export LD_LIBRARY_PATH="$d:${LD_LIBRARY_PATH}"
done
```

---

## 9. 拉取 DEPLOY 源码

```bash
cd ~
git clone git@github.com:CharlesH777/NEXUS_DEPLOY.git NEXUS_DEPLOY
# 或者用 https:
# git clone https://github.com/CharlesH777/NEXUS_DEPLOY.git NEXUS_DEPLOY

cd NEXUS_DEPLOY
```

确认目录结构：

```bash
ls -la
# 应该看到 src/ launch/ config/ scripts/ tools/ third_party/ docs/ README.md 等
```

---

## 10. 拉取 FAST-LIO2 源码

> ⚠️ **重要**：DEPLOY 仓库的 git index 把 `third_party/FASTLIO2_ROS2` 记录为一个 submodule gitlink（mode 160000，指向 commit `f516daac`），但**没有 `.gitmodules` 文件**。这意味着 `git clone NEXUS_DEPLOY` 之后该目录是**空的**，必须手动克隆 FAST-LIO2 ROS2 源码进去。

```bash
cd ~/NEXUS_DEPLOY/third_party
# 新 clone 后该目录为空，直接删除占位符再克隆
rm -rf FASTLIO2_ROS2

# 已知上游 fork（如果 URL 失效，请向仓库维护者确认正确的 fork）
git clone https://github.com/yanliang-wang/FASTLIO2_ROS2.git FASTLIO2_ROS2
# 或者用你维护的 fork：
# git clone <你的 FAST-LIO2 ROS2 fork> FASTLIO2_ROS2

cd FASTLIO2_ROS2
ls
# 应该看到 fastlio2/ hba/ interface/ localizer/ pgo/ script/ README.md
```

验证主包源码存在：

```bash
ls ~/NEXUS_DEPLOY/third_party/FASTLIO2_ROS2/fastlio2/
# 应该有 CMakeLists.txt package.xml src/ launch/ 等
```

> ℹ️ `scripts/build_deploy.sh` 启动时会检测该目录是否为符号链接并警告，但不会自动克隆。`scripts/check_env.sh` 第 8 项会检查 `fastlio2` 源码是否存在。

---

## 11. 配置 Livox MID360 网络

### 11.1 主机网卡

把连雷达的网口设成静态 IP `192.168.1.50`（和 `src/livox_ros_driver2/config/MID360_config.json` 里的 `host_net_info` 对应）：

```bash
# 用 nmcli（NetworkManager）
nmcli con show                                    # 找到对应连接名，例如 "Wired connection 1"
sudo nmcli con modify "Wired connection 1" \
  ipv4.addresses 192.168.1.50/24 \
  ipv4.gateway 192.168.1.1 \
  ipv4.method manual
sudo nmcli con up "Wired connection 1"

ip addr show | grep 192.168.1.50                  # 验证
ping -c 3 192.168.1.3                             # ping 雷达（默认 IP）
```

### 11.2 雷达 bd_code（广播码）

每台 MID360 都有唯一 15 位序列号（贴在机身），格式类似 `7JHXXXXXXXXXXX`。在 `MID360_config.json` 的 `lidar_configs[0]` 里设置，或通过 launch 参数 `livox_bd_code` 传入。

如果暂时不知道 bd_code，先用默认值 `livox0000000001`（DEPLOY 默认值）让驱动起来，雷达连上后会广播自己的 bd_code，从 `ros2 topic echo /livox/lidar` 是否有数据判断连接是否成功。

> ℹ️ 实际部署时建议在 `scripts/run_real_robot.sh` 前显式设置：
> ```bash
> export NEXUS_LIVOX_BD_CODE=<你的雷达 bd_code>
> ```

---

## 12. 构建

### 12.1 一键构建 DEPLOY 主工作空间

```bash
cd ~/NEXUS_DEPLOY
bash scripts/build_deploy.sh
```

这个脚本会按顺序：
1. 跑 `scripts/check_env.sh` 检查所有依赖
2. 检测 `third_party/FASTLIO2_ROS2` 是否为符号链接（会警告）
3. source ROS
4. 如果 `tools/elevation_mapping_cupy_ros2_ws/install/setup.bash` 不存在，自动克隆 `elevation_mapping_cupy` + `grid_map` 并构建
5. `colcon build src/`（livox + 5 个 nexus_* 包 + third_party LRAE）

构建产物落在 `~/NEXUS_DEPLOY/install/`。

常见构建选项：

```bash
# 跳过环境检查（确认环境没问题后加速）
bash scripts/build_deploy.sh --skip-env

# 跳过 elevation workspace 构建（已经构建过）
NEXUS_BUILD_ELEVATION=0 bash scripts/build_deploy.sh
```

### 12.2 单独构建 FAST-LIO2

`build_deploy.sh` 不构建 FAST-LIO2 本体。需要单独跑：

```bash
cd ~/NEXUS_DEPLOY
bash scripts/build_fastlio.sh
```

产物落在 `third_party/FASTLIO2_ROS2/install_nexus/fastlio2/lib/fastlio2/lio_node`。

验证可执行：

```bash
ls -l ~/NEXUS_DEPLOY/third_party/FASTLIO2_ROS2/install_nexus/fastlio2/lib/fastlio2/lio_node
# 必须有 x 权限
```

### 12.3 完整构建顺序（首次部署）

```bash
cd ~/NEXUS_DEPLOY
bash scripts/build_deploy.sh            # 1. 主工作空间（含 elevation ws）
bash scripts/build_fastlio.sh           # 2. FAST-LIO2
```

---

## 13. 运行前最终检查

```bash
cd ~/NEXUS_DEPLOY
bash scripts/check_env.sh
```

期望输出（关键项）：

```
==================== NEXUS DEPLOY 环境检查 ====================
── 1. ROS2 Humble
  [OK]   /opt/ros/humble/setup.bash
  [OK]   ros2 命令可用
── 2. Python >= 3.10
  [OK]   /usr/bin/python3 Python 3.10.12
── 3. 构建工具
  [OK]   colcon
  [OK]   cmake
  [OK]   make
── 4. Livox SDK (livox_ros_driver2 依赖)
  [OK]   /usr/local/lib/liblivox_lidar_sdk_shared.so
  [OK]   /usr/local/include/livox_lidar_api.h
── 5. CUDA (elevation_mapping_cupy 依赖)
  [OK]   nvcc ...
── 6. ROS2 系统包
  [OK]   ros2 pkg: nav2_bt_navigator
  [OK]   ros2 pkg: nav2_controller
  [OK]   ros2 pkg: nav2_planner
  [OK]   ros2 pkg: pcl_ros
  [OK]   ros2 pkg: pcl_conversions
  [OK]   ros2 pkg: tf2_ros
── 7. Python 第三方包 (运行时依赖)
  [OK]   numpy (numpy)
  [OK]   scipy (scipy)
  [OK]   do_mpc (do-mpc)
  [OK]   casadi (casadi)
  [OK]   yaml (PyYAML)
  [OK]   tkinter (python3-tk)
  [OK]   torch (GP 建图)
  [OK]   gpytorch (GP 建图)
── 8. FAST-LIO2 源码
  [OK]   .../third_party/FASTLIO2_ROS2/fastlio2 源码存在
── 9. elevation_mapping_cupy workspace
  [OK]   .../elevation_mapping_cupy 源码存在
── 10. 构建产物残留 (应清理后再拷贝)
  [WARN] build/ 存在 ...
  [WARN] install/ 存在 ...
  [WARN] log/ 存在 ...
==================== 检查结果 ====================
  通过: XX   缺失: 0   警告: 3
[OK] 环境检查通过，可以构建。
```

> 第 10 项的 WARN 是正常的（说明已经构建过），不影响运行。

---

## 14. 启动部署栈

### 14.1 全栈默认启动

```bash
cd ~/NEXUS_DEPLOY
bash scripts/run_real_robot.sh
```

默认配置：
- 启动 Livox 驱动 + FAST-LIO2 + 高程图 + 通行图 + Nav2 + sand MPC + 探索
- 不启动 RViz（`NEXUS_ENABLE_RVIZ=false`）
- `use_sim_time=false`
- 全局坐标系 `map`，机器人坐标系 `base_footprint`
- `/cmd_vel` 由 sand MPC 输出

### 14.2 常用启动变体

```bash
# 带 RViz
NEXUS_ENABLE_RVIZ=true bash scripts/run_real_robot.sh

# 关闭 sand MPC（Nav2 直接出 /cmd_vel）
NEXUS_ENABLE_SAND_MPC=false bash scripts/run_real_robot.sh nav2_cmd_vel_topic:=/cmd_vel

# 指定 FAST-LIO2 二进制路径
NEXUS_FASTLIO2_BIN=/path/to/lio_node bash scripts/run_real_robot.sh

# 指定 Livox 配置文件
NEXUS_LIVOX_CONFIG=/path/to/MID360_config.json bash scripts/run_real_robot.sh

# 指定雷达 bd_code
NEXUS_LIVOX_BD_CODE=7JHXXXXXXXXXXX bash scripts/run_real_robot.sh

# 关闭 FAST-LIO2（需要自己提供 /odom 和 TF）
NEXUS_ENABLE_FASTLIO2=false NEXUS_LIVOX_XFER_FORMAT=0 bash scripts/run_real_robot.sh
```

### 14.3 停止

```bash
bash scripts/stop.sh
```

或者直接 Ctrl-C 后清理残留进程：

```bash
pkill -f lio_node
pkill -f real_robot_bringup
pkill -f elevation_mapping_node
pkill -f sand_mpc_compensator
pkill -f livox_lidar_publisher
```

---

## 15. 集成测试（可选，用 Gazebo 喂数据）

如果你想在不动真机的情况下验证整条数据链路，可以用 Gazebo 仿真数据灌进 DEPLOY：

```bash
# 前提：需要兄弟项目 NEXUS_GAZEBO_SIM 在 ../NEXUS_GAZEBO_SIM 路径
ls ~/NEXUS/NEXUS_LIDAR_SIM/NEXUS_GAZEBO_SIM

# 跑 240 秒集成测试（无 GUI）
MAP_SIM_GZCLIENT=0 timeout 240 bash scripts/test_with_sim_data.sh
```

详细验证项和已知警告见 [`docs/GAZEBO_DEPLOY_INTEGRATION.md`](GAZEBO_DEPLOY_INTEGRATION.md)。

---

## 16. 故障排查

### 16.1 `nvcc: command not found`

`/usr/local/cuda/bin` 不在 PATH。检查 `~/.bashrc` 里的 export，并 `source ~/.bashrc`。

### 16.2 `torch.cuda.is_available()` 返回 False

- NVIDIA driver 太旧：`nvidia-smi` 看 Driver Version，需要 ≥ 535
- torch wheel 和 CUDA 版本不匹配：torch wheel 的 CUDA 标签（cu124/cu126/cu128）只需 ≤ 驱动支持的 CUDA 版本。驱动 535 装 `cu124`，545 装 `cu126`，550+ 装 `cu128`。见第 8.2 节。
- 没装 CUDA toolkit：见第 3 节

### 16.3 `libcusparseLt.so.0: cannot open shared object file`

torch / cupy wheel 自带的 nvidia 库没暴露。见第 8.3 节，把 nvidia wheel lib 路径加进 `LD_LIBRARY_PATH`。

### 16.4 `liblivox_lidar_sdk_shared.so: not found`

Livox-SDK2 没装或没 `ldconfig`。见第 4 节，跑完 `sudo make install` 后执行 `sudo ldconfig`。

### 16.5 FAST-LIO2 构建报 `Sophus not found`

Sophus 没装或版本不对。必须是 1.22.10，且 cmake 加 `-DSOPHUS_USE_BASIC_LOGGING=ON`。见第 5 节。

### 16.6 FAST-LIO2 构建报 `GTSAM not found`

只构建 `fastlio2` 包不会报这个，但如果连带构建 `pgo` / `hba` 会。装 GTSAM 见第 6 节。或者只构建主包：

```bash
cd ~/NEXUS_DEPLOY
bash scripts/build_fastlio.sh
# 它内部就是 colcon build --packages-select fastlio2
```

### 16.7 构建时大量 Conda 路径污染

`build_deploy.sh` 和 `run_real_robot.sh` 内置 `sanitize_python_env` 函数会清掉 conda 变量。但如果 conda 还是干扰，临时退出 conda：

```bash
conda deactivate
# 或者直接不要 source conda 的 init 脚本
```

### 16.8 `livox_lidar_publisher` 启动但没有数据

- 检查网线、`ip addr` 确认 192.168.1.50 已生效
- `ping 192.168.1.3` 看雷达是否在线
- 检查 bd_code 是否对（雷达机身后贴的 15 位序列号）
- `ros2 topic hz /livox/lidar` 看是否有数据流

### 16.9 FAST-LIO2 报 `IMU Message is out of order`

已知警告，不影响主线运行。雷达 IMU 时间戳偶尔抖动，FAST-LIO2 内部会丢弃乱序帧。如果频繁报错，检查雷达固件版本。

### 16.10 Nav2 报 `empty path` 或 `missed control loop rates`

已知调参警告，参考 `config/nav2_mppi_real_params.yaml` 里的注释调整 MPPI 控制频率 / 容差。

### 16.11 `third_party/FASTLIO2_ROS2` 目录为空或为符号链接

新 clone DEPLOY 后该目录为空（submodule gitlink 无 .gitmodules，见第 10 节）。从别的机器拷过来可能是断链。`build_deploy.sh` 会警告。修复：

```bash
cd ~/NEXUS_DEPLOY/third_party
rm -rf FASTLIO2_ROS2
git clone https://github.com/yanliang-wang/FASTLIO2_ROS2.git FASTLIO2_ROS2
```

### 16.12 colcon 找不到 `ros2_numpy` / `elevation_map_msgs`

`ros2_numpy` 在 apt 里没有，需要 `pip install --user ros2-numpy`（见第 8.2 节）。`elevation_map_msgs` 是 `elevation_mapping_cupy` 仓库的子包，`build_deploy.sh` 会自动构建。

---

## 17. 已验证版本一览

| 组件 | 版本 | 安装方式 |
|---|---|---|
| Ubuntu | 22.04.5 LTS | 系统 |
| Kernel | 6.8.0-124-generic | 系统 |
| NVIDIA Driver | 580.159.03 | apt |
| CUDA Toolkit | 12.6.3 | apt (cuda-toolkit-12-6) |
| Python | 3.10.12 | 系统 |
| ROS2 | Humble | apt |
| colcon | colcon-common-extensions | apt |
| Livox-SDK2 | master (2025-12) | 源码 |
| Sophus | 1.22.10 | 源码 |
| GTSAM | 4.2.0 | 源码 |
| PCL | 1.12.1 | apt (libpcl-dev) |
| Eigen | 3.4.0 | apt (libeigen3-dev) |
| fmt | 8.1.1 | apt (libfmt-dev) |
| yaml-cpp | 0.7.0 | apt (libyaml-cpp-dev) |
| Nav2 | 1.1.20 | apt (ros-humble-navigation2) |
| grid_map | humble branch | 源码 (elevation ws 内自动克隆) |
| elevation_mapping_cupy | ros2 branch | 源码 (build_deploy.sh 自动克隆) |
| FAST-LIO2 (ROS2) | master | 源码 (third_party/) |
| torch | 2.10.0+cu128 | pip --user |
| cupy-cuda12x | 12.2.0 | pip --user |
| gpytorch | 1.11 | pip --user |
| do-mpc | 5.1.1 | pip --user |
| casadi | 3.7.2 | pip --user |
| numpy | 1.24.2 | pip --user |
| scipy | 1.8.0 | apt (python3-scipy) |
| transforms3d | 0.4.2 | pip --user |
| simple-parsing | 0.1.8 | pip --user |
| ros2-numpy | latest | pip --user |
| PyYAML | 6.0.3 | pip --user |
| opencv-python | 4.11.0.86 | pip --user |

---

## 18. 速查命令

```bash
# ── 一次性环境检查 ──
bash scripts/check_env.sh

# ── 构建 ──
bash scripts/build_deploy.sh           # 主工作空间
bash scripts/build_fastlio.sh          # FAST-LIO2

# ── 运行 ──
bash scripts/run_real_robot.sh         # 全栈
bash scripts/stop.sh                   # 停止

# ── 调试 ──
ros2 topic list
ros2 topic hz /livox/lidar
ros2 topic echo /fastlio2/lio_odom --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo map base_footprint
ros2 lifecycle list
ros2 node list

# ── 集成测试（Gazebo 喂数据）──
MAP_SIM_GZCLIENT=0 timeout 240 bash scripts/test_with_sim_data.sh
```

---

## 19. 数据流速查

```
/livox/lidar + /livox/imu
  → fastlio_lidar_adapter + fastlio_imu_adapter
  → FAST-LIO2 lio_node
  → /fastlio2/lio_odom + /fastlio2/world_cloud
  → fastlio_odom_bridge → /odom + /pose + map→base_footprint TF
  → elevation_mapping_node → /elevation_mapping_node/elevation_map
  → traversability_to_map → /traversability_map
  → Nav2 MPPI → /mppi/cmd_vel_raw
  → sand_mpc_compensator → /cmd_vel
```

参考：[`README.md`](../README.md) · [`docs/GAZEBO_DEPLOY_INTEGRATION.md`](GAZEBO_DEPLOY_INTEGRATION.md)
