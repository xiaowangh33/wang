#!/usr/bin/env bash
# 编译脚本（默认：MuJoCo C++ 仿真；实机请用 ./build.sh hw）
# 用法：./build.sh          — 仿真模式增量编译 (MuJoCo C++)
#       ./build.sh hw       — 实机模式（硬件 HAL）
#       ./build.sh mj       — 显式 MuJoCo C++ 仿真（与默认相同）
#       ./build.sh clean    — 清除后全量重编

DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${DIR}/build_local"
BUILD_TYPE="${BUILD_TYPE:-Release}"

# RoboLibPro 安装路径（与 run.sh 保持一致）
ROBO_PRO_INSTALL_PATH="${ROBO_PRO_INSTALL_PATH:-/usr/local/RoboLibPro}"

# 默认：仿真（MuJoCo C++）。若需实机：./build.sh hw
BUILD_SIM=ON

# 解析参数
if [ "$1" = "clean" ]; then
    echo "清除构建目录: $BUILD_DIR"
    rm -rf "$BUILD_DIR"
    shift
fi

if [ "$1" = "hw" ] || [ "$1" = "real" ]; then
    BUILD_SIM=OFF
    echo "=== 实机模式编译 (BUILD_SIM=OFF) ==="
elif [ "$1" = "mj" ] || [ "$1" = "mujoco" ] || [ "$1" = "sim" ]; then
    BUILD_SIM=ON
    echo "=== 仿真模式编译 (MuJoCo C++, BUILD_SIM=ON) ==="
elif [ -n "$1" ]; then
    echo "未知参数: $1" >&2
    echo "用法: ./build.sh [clean] [hw|real|mj|mujoco|sim]" >&2
    exit 2
else
    echo "=== 仿真模式编译 (MuJoCo C++, 默认) ==="
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# A copied/moved workspace leaves absolute source and binary paths in the CMake
# cache. Reusing that cache can write into the old tree (or a read-only path).
if [ -f CMakeCache.txt ]; then
    CACHED_HOME_DIR=$(grep -m1 '^CMAKE_HOME_DIRECTORY:INTERNAL=' CMakeCache.txt 2>/dev/null | cut -d= -f2-)
    if [ -n "$CACHED_HOME_DIR" ] && [ "$CACHED_HOME_DIR" != "$DIR" ]; then
        echo "检测到构建缓存来自旧源码目录 ($CACHED_HOME_DIR)，重建 $BUILD_DIR..."
        cd "$DIR" || exit 1
        rm -rf "$BUILD_DIR"
        mkdir -p "$BUILD_DIR"
        cd "$BUILD_DIR" || exit 1
    fi
fi

CMAKE_EXTRA_SIM=(
    -DUSE_RAISIM=OFF
    -DUSE_MJCPP=ON
)

NEED_CMAKE=0
if [ ! -f Makefile ]; then
    NEED_CMAKE=1
elif [ -f CMakeCache.txt ]; then
    CACHED_BSIM=$(grep -m1 '^BUILD_SIM:BOOL=' CMakeCache.txt 2>/dev/null | cut -d= -f2)
    if [ -n "$CACHED_BSIM" ] && [ "$CACHED_BSIM" != "$BUILD_SIM" ]; then
        echo "检测到 BUILD_SIM 缓存 ($CACHED_BSIM) 与当前目标 ($BUILD_SIM) 不一致，重新配置 CMake..."
        NEED_CMAKE=1
    fi
    CACHED_BUILD_TYPE=$(grep -m1 '^CMAKE_BUILD_TYPE:STRING=' CMakeCache.txt 2>/dev/null | cut -d= -f2)
    if [ -n "$CACHED_BUILD_TYPE" ] && [ "$CACHED_BUILD_TYPE" != "$BUILD_TYPE" ]; then
        echo "检测到构建类型缓存 ($CACHED_BUILD_TYPE) 与当前目标 ($BUILD_TYPE) 不一致，重新配置 CMake..."
        NEED_CMAKE=1
    fi
fi

if [ "$NEED_CMAKE" -eq 1 ]; then
    echo "运行 CMake 配置..."
    if [ "$BUILD_SIM" = "ON" ]; then
        cmake "$DIR" \
            -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
            -DROBO_PRO_INSTALL_PATH="${ROBO_PRO_INSTALL_PATH}" \
            -DBUILD_PLATFORM=x86 \
            -DBUILD_SIM=ON \
            -DSEND_REMOTE=OFF \
            "${CMAKE_EXTRA_SIM[@]}"
    else
        cmake "$DIR" \
            -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
            -DROBO_PRO_INSTALL_PATH="${ROBO_PRO_INSTALL_PATH}" \
            -DBUILD_PLATFORM=x86 \
            -DBUILD_SIM=OFF \
            -DSEND_REMOTE=OFF
    fi
fi

echo "编译中..."
cmake --build . --parallel "$(nproc)"
BUILD_RESULT=$?

if [ $BUILD_RESULT -eq 0 ]; then
    printf '%s\n' "$BUILD_SIM" > "${BUILD_DIR}/build_mode.txt"
    printf '%s\n' "$DIR" > "${BUILD_DIR}/source_root.txt"
    echo ""
    echo "✓ 编译成功: ${BUILD_DIR}/rl_deploy"
    if [ "$BUILD_SIM" = "ON" ]; then
        echo "  MuJoCo C++: cd ${DIR} && sudo ./run_mj.sh"
    else
        echo "  启动方式: cd ${DIR} && sudo ./run.sh"
    fi
else
    echo ""
    echo "✗ 编译失败 (exit code: $BUILD_RESULT)"
fi

exit $BUILD_RESULT
