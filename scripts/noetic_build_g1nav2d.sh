#!/usr/bin/env bash
set -euo pipefail

# 在 wk_noetic 容器内构建 G1Nav2D。
# 脚本功能：
# 1) 强制使用系统工具链（/usr/bin），避免 conda 的 python/cmake 干扰 ROS Noetic 编译
# 2) 安装必需依赖（python3-empy、GTSAM stable/unstable）
# 3) 编译 livox_ros_driver2 -> 验证 CustomMsg.h -> 编译 fastlio -> 全量编译工作空间

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
THIRD_PARTY_DIR="${ROOT_DIR}/third_party"
LIVOX_SDK2_DIR=""
if [ -d "${ROOT_DIR}/Livox-SDK2" ]; then
  LIVOX_SDK2_DIR="${ROOT_DIR}/Livox-SDK2"
elif [ -d "${THIRD_PARTY_DIR}/Livox-SDK2" ]; then
  LIVOX_SDK2_DIR="${THIRD_PARTY_DIR}/Livox-SDK2"
fi

# 将 /usr/bin 放在 PATH 前面，避免 conda（/root/miniconda3/...）的 CMake 4 / Python 3.10 影响 Noetic 编译。
export PATH="/usr/bin:/bin:${PATH}"

# ROS 环境脚本里会引用未设置变量，不兼容 set -u；这里临时关闭 -u。
set +u
source /opt/ros/noetic/setup.bash
set -u

# 容器内默认以 root 运行，直接用 apt-get 安装依赖（不依赖 sudo）。
if [ "$(id -u)" != "0" ]; then
  echo "ERROR: please run as root inside wk_noetic container (no sudo assumed)."
  exit 1
fi

echo "== Install deps (idempotent) =="
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y \
  python3-empy \
  libgtsam-dev \
  libgtsam4 \
  libgtsam-unstable4 \
  libgtsam-unstable-dev
ldconfig

if [ -n "${LIVOX_SDK2_DIR}" ]; then
  echo "== Build Livox-SDK2 (optional) =="
  rm -rf /tmp/Livox-SDK2-build
  mkdir -p /tmp/Livox-SDK2-build
  cd /tmp/Livox-SDK2-build
  cmake "${LIVOX_SDK2_DIR}"
  cmake --build . -j"$(nproc)"
  cmake --install .
fi

if [ -d "${THIRD_PARTY_DIR}/gtsam" ]; then
  echo "== Build third_party gtsam (optional) =="
  rm -rf /tmp/gtsam-build
  mkdir -p /tmp/gtsam-build
  cd /tmp/gtsam-build
  cmake "${THIRD_PARTY_DIR}/gtsam" -DGTSAM_BUILD_EXAMPLES=OFF -DGTSAM_BUILD_TESTS=OFF
  cmake --build . -j"$(nproc)"
  cmake --install .
fi

cd "${ROOT_DIR}/G1Nav2D"
rm -rf build devel

echo "== Build livox_ros_driver2 =="
catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DROS_EDITION=ROS1 -DPYTHON_EXECUTABLE=/usr/bin/python3 --pkg livox_ros_driver2

echo "== Verify generated headers (CustomMsg.h) =="
ls -la devel/include/livox_ros_driver2

echo "== Build fastlio =="
catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DROS_EDITION=ROS1 -DPYTHON_EXECUTABLE=/usr/bin/python3 --pkg fastlio -j"$(nproc)"

echo "== Full workspace build =="
catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DROS_EDITION=ROS1 -DPYTHON_EXECUTABLE=/usr/bin/python3 -j"$(nproc)"

echo "== Source workspace =="
set +u
source devel/setup.bash
set -u

echo "== Done =="
