#!/bin/bash
# =============================================================================
# dependency_install.bash
# ROS 2 Humble 项目完整依赖自动安装脚本
# 适用于 Ubuntu 22.04 + ROS 2 Humble
# 根据《项目额外依赖安装指南》编写，覆盖所有额外依赖
# =============================================================================

set -euo pipefail

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo_info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
echo_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
echo_error() { echo -e "${RED}[ERROR]${NC} $*"; }

# =============================================================================
# 1. 系统与 ROS 2 环境检查
# =============================================================================
echo_info "检查系统与 ROS 2 环境..."

if [[ ! -f /etc/os-release ]]; then
    echo_error "无法检测操作系统信息，请在 Ubuntu 系统上运行。"
    exit 1
fi

source /etc/os-release
if [[ "$ID" != "ubuntu" || "$VERSION_ID" != "22.04" ]]; then
    echo_warn "本脚本针对 Ubuntu 22.04 + ROS 2 Humble 设计。"
    echo_warn "当前系统: $PRETTY_NAME"
    read -p "是否继续？(y/N): " confirm
    [[ $confirm =~ ^[Yy]$ ]] || exit 0
fi

if [[ ! -d "/opt/ros/humble" ]]; then
    echo_error "未检测到 ROS 2 Humble，请先安装 ROS 2 Humble Desktop。"
    echo_info "推荐命令：sudo apt install ros-humble-desktop"
    exit 1
else
    echo_info "ROS 2 Humble 已安装。"
    # 兼容 ROS setup 脚本中的未定义变量
    set +u
    source /opt/ros/humble/setup.bash
    set -u
fi

# =============================================================================
# 2. 更新 apt 并安装系统库依赖
# =============================================================================
echo_info "更新软件源并安装系统库依赖..."

sudo apt update

sudo apt install -y \
    libserialport-dev \
    libapr1-dev \
    libpcl-dev \
    pcl-tools \
    libopencv-dev \
    libopen3d-dev \
    libeigen3-dev \
    libomp-dev \
    python3-dev \
    libpcap-dev \
    libpng-dev

# =============================================================================
# 3. 安装 ROS 2 额外包
# =============================================================================
echo_info "安装 ROS 2 Humble 额外依赖包..."

sudo apt install -y \
    ros-humble-can-msgs \
    ros-humble-laser-geometry \
    ros-humble-pcl-ros \
    ros-humble-pcl-msgs \
    ros-humble-pcl-conversions \
    ros-humble-navigation2 \
    ros-humble-nav2-common \
    ros-humble-nav2-core \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-map-server \
    ros-humble-nav2-amcl \
    ros-humble-nav2-planner \
    ros-humble-nav2-controller \
    ros-humble-nav2-behavior-tree \
    ros-humble-nav2-rviz-plugins \
    ros-humble-rosbag2 \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-urdf \
    ros-humble-xacro

# =============================================================================
# 4. 安装 Python 系统级依赖
# =============================================================================
echo_info "安装 Python 系统依赖..."

sudo apt install -y \
    python3-numpy \
    python3-pil \
    python3-yaml \
    python3-scipy

# =============================================================================
# 5. 安装 pip Python 包（优先用户级别安装）
# =============================================================================
echo_info "安装 pip Python 包..."

if ! command -v pip3 &> /dev/null; then
    sudo apt install -y python3-pip
fi


# =============================================================================
# 6. 从源码安装 Livox SDK（如果尚未安装）
# =============================================================================
echo_info "检查并安装 Livox SDK..."

LIVOX_LIB="/usr/local/lib/liblivox_lidar_sdk_shared.so"
if [[ ! -f "$LIVOX_LIB" ]]; then
    echo_info "Livox SDK 未安装，开始从源码编译..."

    if ! command -v git &> /dev/null; then
        sudo apt install -y git
    fi
    if ! command -v cmake &> /dev/null; then
        sudo apt install -y cmake
    fi

    cd /tmp
    git clone https://github.com/Livox-SDK/Livox-SDK.git
    cd Livox-SDK
    mkdir -p build && cd build
    cmake .. && make -j$(nproc)
    sudo make install
    cd /tmp
    rm -rf Livox-SDK

    echo_info "Livox SDK 安装完成。"
else
    echo_info "Livox SDK 已存在，跳过安装。"
fi

# =============================================================================
# 7. 设置 LD_LIBRARY_PATH（Livox SDK 需要）
# =============================================================================
echo_info "配置 LD_LIBRARY_PATH..."

if ! grep -q "/usr/local/lib" ~/.bashrc; then
    echo '' >> ~/.bashrc
    echo '# Added by install_dependency.bash for Livox SDK' >> ~/.bashrc
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib' >> ~/.bashrc
    echo_info "已将 /usr/local/lib 添加到 ~/.bashrc"
fi

# =============================================================================
# 8. 完成提示与验证建议
# =============================================================================
echo_info "====================================================================="
echo_info "所有项目依赖安装完成！"
echo_info "====================================================================="
echo_info "请执行以下操作以应用环境变更："
echo ""
echo "    source ~/.bashrc"
echo "    source /opt/ros/humble/setup.bash"
echo ""
echo_info "验证关键依赖（可选）："
echo ""
echo "    pkg-config --modversion pcl_common"
echo "    pkg-config --modversion opencv4"
echo "    pkg-config --modversion eigen3"
echo "    ls /usr/local/lib/liblivox_lidar_sdk_shared.so"
echo "    ros2 pkg list | grep -E '(nav2|pcl|can|laser)'"
echo ""

exit 0