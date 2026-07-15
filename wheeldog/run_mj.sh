#!/usr/bin/env bash
# MuJoCo C++ 仿真入口。

set -o pipefail

dirname="$( cd "$( dirname "$0" )" >/dev/null 2>&1 && pwd )"
cd "$dirname" || exit 1
export PROGRAM_ROOT_PATH="$dirname"
export ROBO_PRO_INSTALL_PATH="${ROBO_PRO_INSTALL_PATH:-/usr/local/RoboLibPro}"
export ROBO_WORKSPACE="wheeldog"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+${LD_LIBRARY_PATH}:}${ROBO_PRO_INSTALL_PATH}/lib:${dirname}/third_party/mujoco/x86/lib:${dirname}/third_party/onnxruntime/x86/lib"

prepare_logfile() {
    local candidate="$1"
    local parent=""
    parent="$(dirname "$candidate")"
    mkdir -p "$parent" || return 1
    if ( : > "$candidate" ) 2>/dev/null; then
        if [ -n "${SUDO_UID:-}" ] && [ -n "${SUDO_GID:-}" ]; then
            chown "${SUDO_UID}:${SUDO_GID}" "$candidate" 2>/dev/null || true
        fi
        LOGFILE="$candidate"
        return 0
    fi
    return 1
}

if [ ! -f build_local/build_mode.txt ]; then
    echo "错误: 缺少构建模式标记，请先执行 ./build.sh" >&2
    exit 1
fi
if [ "$(cat build_local/build_mode.txt)" != "ON" ]; then
    echo "错误: build_local 不是仿真版本，请先执行 ./build.sh" >&2
    exit 1
fi
if [ ! -x build_local/rl_deploy ]; then
    echo "错误: 缺少 build_local/rl_deploy，请先执行 ./build.sh" >&2
    exit 1
fi
if [ ! -x "${ROBO_PRO_INSTALL_PATH}/bin/robo_core_main" ]; then
    echo "错误: 找不到 ${ROBO_PRO_INSTALL_PATH}/bin/robo_core_main" >&2
    exit 1
fi

rm -f /dev/shm/"${ROBO_WORKSPACE}"*

if [ "${MUJOCO_FLOAT_MODE}" = "1" ]; then
    echo "MUJOCO_FLOAT_MODE=1: robot base fixed in space"
fi

if [ -n "${MUJOCO_SCENE}" ]; then
    echo "MUJOCO_SCENE=${MUJOCO_SCENE}: using terrain scene"
fi

timestamp="$(date +%Y%m%d_%H%M%S)"
LOGFILE="${LOGFILE:-${dirname}/log/standup_monitor_mj_${timestamp}.log}"
if ! prepare_logfile "$LOGFILE"; then
    fallback_logfile="${dirname}/log/standup_monitor_mj_${timestamp}_$$.log"
    echo "警告: 无法写入 LOGFILE=$LOGFILE，改用 $fallback_logfile" >&2
    prepare_logfile "$fallback_logfile" || {
        echo "错误: 无法创建日志文件 $fallback_logfile" >&2
        exit 1
    }
fi

echo "=== MuJoCo 仿真日志: $LOGFILE ==="

stdbuf -oL "${ROBO_PRO_INSTALL_PATH}/bin/robo_core_main" \
    "PROGRAM_ROOT_PATH" "/Sources/quardLeg_mj.json" 2>&1 | tee -a "$LOGFILE"
