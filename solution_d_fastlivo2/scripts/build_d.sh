#!/bin/bash
# ============================================================================
# solution_d（FAST-LIVO2）依赖准备 + 构建
# ----------------------------------------------------------------------------
# 做三件事（全部可重复执行，已完成的步骤自动跳过）：
#   1. Sophus：apt-get download + dpkg -x 到 .deps/sophus（不需要 sudo，不污染系统）
#   2. U-AMC rpg_vikit（FAST-LIVO2 专用 fork，含 RationalPolynomial / camera_loader）
#      - vikit_common：plain CMake 编译安装到 .deps/vikit，手写 Config（同官方 Dockerfile）
#      - vikit_ros   ：ament 包，交给 colcon
#   3. colcon 构建 vikit_ros + fast_livo 到 solution_d_fastlivo2/ws/install
#
# 用法：
#   ./build_d.sh              # 全量（已构建部分会复用）
#   JOBS=2 ./build_d.sh       # 限制并行（RK3588 上 4 核跑满会明显发热）
#   REBUILD=1 ./build_d.sh    # 强制重编 fast_livo
#
# 前置：本机已装 ros-humble（含 pcl_ros/image_transport/cv_bridge/rviz2）
#       以及 livox_ros_driver2（/home/ztl/PointPanoramaTex/livox_driver2/install）
# ============================================================================
set -e

ROOT=/home/ztl/git/sl_ws/Super-LIVO
SOL="$ROOT/solution_d_fastlivo2"
FASTLIVO="$ROOT/third_party/FAST-LIVO2-ROS2"
VIKIT_SRC="$ROOT/third_party/rpg_vikit_rational_polynomial"
D="$SOL/.deps"
SOPHUS_PREFIX="$D/sophus/opt/ros/humble"
LIVOX=/home/ztl/PointPanoramaTex/livox_driver2
JOBS=${JOBS:-4}

say() { echo -e "\n\033[1;36m[d] $*\033[0m"; }

source /opt/ros/humble/setup.bash
source "$LIVOX/install/setup.bash"

# ---------------------------------------------------------------- Sophus ----
if [ ! -d "$SOPHUS_PREFIX/include/sophus" ]; then
  say "准备 Sophus（ros-humble-sophus 22.10）"
  mkdir -p "$D/sophus" /tmp/d_sophus_deb
  ( cd /tmp/d_sophus_deb && apt-get download ros-humble-sophus >/dev/null )
  dpkg -x /tmp/d_sophus_deb/ros-humble-sophus_*.deb "$D/sophus"
else
  say "Sophus 已就绪：$SOPHUS_PREFIX"
  [ -d "$SOPHUS_PREFIX/include/sophus" ] || { echo "Sophus 解包异常"; exit 1; }
fi

# ----------------------------------------------------------------- vikit ----
if [ ! -d "$VIKIT_SRC/vikit_ros" ]; then
  say "克隆 U-AMC rpg_vikit_rational_polynomial（需要 GitHub 可达）"
  git clone --depth=1 https://github.com/U-AMC/rpg_vikit_rational_polynomial.git "$VIKIT_SRC"
fi

# 本地补丁（如 wait_for_service 超时放宽）：幂等，已应用的自动跳过
for p in "$SOL"/patches/vikit-*.patch; do
  [ -e "$p" ] || continue
  if git -C "$VIKIT_SRC" apply --reverse --check "$p" 2>/dev/null; then
    say "vikit 补丁已应用：$(basename "$p")"
  else
    say "应用 vikit 补丁：$(basename "$p")"
    git -C "$VIKIT_SRC" apply "$p"
  fi
done

if [ ! -f "$D/vikit/lib/libvikit_common.so" ]; then
  say "编译 vikit_common（plain CMake）"
  cmake -S "$VIKIT_SRC/vikit_common" -B /tmp/d_vikit_common_build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$D/vikit" \
    -DCMAKE_PREFIX_PATH="$SOPHUS_PREFIX" \
    -DSophus_INCLUDE_DIRS="$SOPHUS_PREFIX/include"
  cmake --build /tmp/d_vikit_common_build -j"$JOBS"
  cmake --install /tmp/d_vikit_common_build
  # vikit_common 的 CMakeLists 只装 include/vikit 与 .so，Config 由官方 Dockerfile 手写
  mkdir -p "$D/vikit/lib/cmake/vikit_common"
  cat > "$D/vikit/lib/cmake/vikit_common/vikit_commonConfig.cmake" <<EOF
set(vikit_common_SOURCE_DIR "$VIKIT_SRC/vikit_common")
set(vikit_common_DIR "$D/vikit/lib/cmake/vikit_common")
set(vikit_common_INCLUDE_DIR "$D/vikit/include")
set(vikit_common_INCLUDE_DIRS "$D/vikit/include")
set(vikit_common_LIBRARIES "$D/vikit/lib/libvikit_common.so")
set(vikit_common_LIBRARY "$D/vikit/lib/libvikit_common.so")
set(vikit_common_LIBRARY_DIR "$D/vikit/lib")
set(vikit_common_LIBRARY_DIRS "$D/vikit/lib")
EOF
fi

# ------------------------------------------------------------ colcon ws ----
mkdir -p "$SOL/ws/src"
ln -sfn "$VIKIT_SRC/vikit_ros" "$SOL/ws/src/vikit_ros"
ln -sfn "$FASTLIVO" "$SOL/ws/src/fast_livo"

export CMAKE_PREFIX_PATH="$D/vikit:$SOPHUS_PREFIX:$CMAKE_PREFIX_PATH"
# 显式给 aarch64 优化：colcon 子构建里 CMAKE_SYSTEM_PROCESSOR 可能为空，
# 上游 CMakeLists 会误走 x86 分支（实际仍能编过，但少了 -ffast-math）。
COLCON_ARGS=(--cmake-args -DCMAKE_BUILD_TYPE=Release
             -DSophus_INCLUDE_DIRS="$SOPHUS_PREFIX/include"
             -DCMAKE_CXX_FLAGS_RELEASE="-O3 -mcpu=native -mtune=native -ffast-math")

# colcon/CMake 的 FindPython3 在部分环境会偶发 "list index out of range"
# （超时/输出被污染导致解释器属性解析失败），清掉缓存的 build 目录重试即可。
build_pkg() {
  local pkg=$1; shift
  local i
  for i in 1 2 3; do
    if ( cd "$SOL/ws" && colcon build --packages-select "$pkg" "$@" ); then
      return 0
    fi
    if grep -q "list index" "$SOL/ws/log/latest_build/$pkg/stderr.log" 2>/dev/null; then
      echo "[retry] $pkg 配置阶段 FindPython3 偶发失败，清理 build/$pkg 后重试"
      rm -rf "$SOL/ws/build/$pkg"
      continue
    fi
    return 1
  done
  return 1
}

if [ ! -f "$SOL/ws/install/vikit_ros/local_setup.bash" ]; then
  say "colcon 编译 vikit_ros"
  build_pkg vikit_ros "${COLCON_ARGS[@]}"
fi

say "colcon 编译 fast_livo（-j$JOBS，4 核上约需数分钟）"
if [ "${REBUILD:-0}" = "1" ]; then
  rm -rf "$SOL/ws/build/fast_livo" "$SOL/ws/install/fast_livo"
fi
build_pkg fast_livo --parallel-workers "$JOBS" "${COLCON_ARGS[@]}"

say "完成。可执行文件：$SOL/ws/install/fast_livo/lib/fast_livo/fastlivo_mapping"
echo "运行：$SOL/scripts/run_d_live.sh [输出目录]"
