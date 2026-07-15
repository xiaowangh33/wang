#!/usr/bin/env bash
# 真机入口：BUILD_SIM=OFF 使用 HiPNUC IMU + RS06/RS01 16 电机真实后端。
# 常用环境变量：
#   WHEELDOG_IMU_PORT=/dev/ttyUSB0
#   WHEELDOG_IMU_BAUD=115200       # 当前 HI91 实测上电波特率
#   WHEELDOG_IMU_AXIS_MAP=y,-x,z      # 实测 HI91 gyro/acc -> URDF body: +X前 +Y左 +Z上
#   WHEELDOG_IMU_RPY_MAP=x,-y,z       # 实测 HI91 RPY：roll同向、pitch反向、yaw同向
#   MCU标定已处理轮电机安装镜像；策略双向轮符号（FL,FR,HL,HR）=[+1,+1,+1,+1]。
#   WHEELDOG_HW_TICK_HZ=200           # 高层状态机 tick；策略仍按 0.02s elapsed-time 触发（50 Hz）
#   RL_HIP_KD=2.0                     # 真机RL HipX/HipY阻尼下限；默认与Kd=2训练一致
#   RL_KNEE_KD=2.0                    # 真机RL Knee阻尼下限；默认与Kd=2训练一致
#   RL_STABILITY_MONITOR=1            # 5 Hz单行稳定裕度监测；默认开启
#   RL_VERBOSE_POLICY_IO=0            # 旧PolicyObs/PolicyWheel长打印；默认关闭
#   RL_TORQUE_ALIGNMENT_DIAG=0        # 旧PC力矩对齐打印；默认关闭
#   WHEELDOG_TORQUE_TELEMETRY_REPORT_S=0 # 旧MCU力矩5 Hz打印；0为关闭
#   WHEELDOG_MOTOR_PORTS=/dev/ttyACM0,/dev/ttyACM1  # 一个物理 USB 下的两个 CDC 端点
#   WHEELDOG_MCU_SETPOINT_HZ=200       # PC→双 MCU 五元组复用/保持频率，不改变50 Hz ONNX推理
#   WHEELDOG_MCU_COMMAND_TIMEOUT_MS=300 # MCU本地命令看门狗；连续失联才归零
#   WHEELDOG_RUNTIME_FEEDBACK_TIMEOUT_MS=500 # PC最终一致反馈锁停门限
#   MCU task 为 1 kHz；PD 只消费新的 500 Hz 合格反馈，并以 500 Hz 刷新 CAN。
#   启动自检全程零命令，不做逐电机资格激励；机械方向由吊装后的 z 检查。
#   RL 电机观测按短历史匹配双 MCU 同 setpoint 序号；样本年龄<=10 ms、到达偏差<=30 ms。
#   当前 RS06 腿关节力矩上限：36.0 N.m；腿指令限速8 rad/s且保留腿超速保护。
#   RS01轮目标使用协议最大+/-44 rad/s；轮实测速降额/锁停和600 Nm/s增扭斜率已禁用。
#   RS01轮驱动峰值力矩上限仍为17.0 N.m.；失联、过温、驱动故障等保护保持不变。
#   WHEELDOG_MOTOR_VERIFIED_ENABLE=1  # 反馈验证使能，不做运动验证
#   WHEELDOG_MOTOR_MOTION_VERIFY=0
#   WHEELDOG_MOTOR_REQUIRED=1

set -o pipefail

dirname="$( cd "$( dirname "$0" )" >/dev/null 2>&1 && pwd )"
cd "$dirname" || exit 1
export PROGRAM_ROOT_PATH="$dirname"
export ROBO_PRO_INSTALL_PATH="${ROBO_PRO_INSTALL_PATH:-/usr/local/RoboLibPro}"
export ROBO_WORKSPACE="wheeldog"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+${LD_LIBRARY_PATH}:}${ROBO_PRO_INSTALL_PATH}/lib:${dirname}/third_party/onnxruntime/x86/lib"
export WHEELDOG_REQUIRE_ONNX="${WHEELDOG_REQUIRE_ONNX:-1}"
export WHEELDOG_HW_TICK_HZ="${WHEELDOG_HW_TICK_HZ:-200}"
export WHEELDOG_MCU_SETPOINT_HZ="${WHEELDOG_MCU_SETPOINT_HZ:-200}"
export WHEELDOG_MCU_COMMAND_TIMEOUT_MS="${WHEELDOG_MCU_COMMAND_TIMEOUT_MS:-300}"
export WHEELDOG_RUNTIME_FEEDBACK_TIMEOUT_MS="${WHEELDOG_RUNTIME_FEEDBACK_TIMEOUT_MS:-500}"
# Installed HI91 alignment measured with the robot body level and by applying
# isolated roll/pitch/yaw motions. Keep caller overrides available for bench
# diagnostics, but never fall back to the old identity map on real hardware.
export WHEELDOG_IMU_AXIS_MAP="${WHEELDOG_IMU_AXIS_MAP:-y,-x,z}"
export WHEELDOG_IMU_RPY_MAP="${WHEELDOG_IMU_RPY_MAP:-x,-y,z}"
export WHEELDOG_POLICY_PATH="${WHEELDOG_POLICY_PATH:-${dirname}/policy/ppo/policy.onnx}"
export RL_HIP_KD="${RL_HIP_KD:-2.0}"
export RL_KNEE_KD="${RL_KNEE_KD:-2.0}"
export RL_STABILITY_MONITOR="${RL_STABILITY_MONITOR:-1}"
export RL_VERBOSE_POLICY_IO="${RL_VERBOSE_POLICY_IO:-0}"
export RL_DEBUG_OUTPUT="${RL_DEBUG_OUTPUT:-0}"
export RL_TORQUE_ALIGNMENT_DIAG="${RL_TORQUE_ALIGNMENT_DIAG:-0}"
export WHEELDOG_TORQUE_TELEMETRY_REPORT_S="${WHEELDOG_TORQUE_TELEMETRY_REPORT_S:-0}"

find_sudo_user_home() {
    local user_home=""
    if [ -n "${SUDO_USER:-}" ]; then
        local passwd_entry=""
        passwd_entry="$(getent passwd "$SUDO_USER" 2>/dev/null || true)"
        if [ -n "$passwd_entry" ]; then
            user_home="$(printf '%s\n' "$passwd_entry" | awk -F: '{print $6}')"
        fi
        if [ -z "$user_home" ] && [ -d "/home/${SUDO_USER}" ]; then
            user_home="/home/${SUDO_USER}"
        fi
    fi
    printf '%s\n' "$user_home"
}

python_has_pyserial() {
    [ -n "$1" ] && "$1" -c 'import serial' >/dev/null 2>&1
}

select_motor_python() {
    if [ -n "${WHEELDOG_PYTHON:-}" ]; then
        if python_has_pyserial "$WHEELDOG_PYTHON"; then
            export WHEELDOG_PYTHON
            echo "=== 电机桥 Python: $WHEELDOG_PYTHON ==="
            return 0
        fi
        echo "错误: WHEELDOG_PYTHON=$WHEELDOG_PYTHON 无法 import serial，请安装 pyserial" >&2
        return 1
    fi

    local candidates=()
    local user_home=""
    local command_python=""

    if [ -n "${CONDA_PREFIX:-}" ]; then
        candidates+=("${CONDA_PREFIX}/bin/python3" "${CONDA_PREFIX}/bin/python")
    fi

    user_home="$(find_sudo_user_home)"
    if [ -n "$user_home" ]; then
        candidates+=(
            "${user_home}/anaconda3/bin/python3"
            "${user_home}/anaconda3/bin/python"
            "${user_home}/miniconda3/bin/python3"
            "${user_home}/miniconda3/bin/python"
            "${user_home}/miniforge3/bin/python3"
            "${user_home}/miniforge3/bin/python"
            "${user_home}/mambaforge/bin/python3"
            "${user_home}/mambaforge/bin/python"
        )
    fi

    if [ -n "${HOME:-}" ]; then
        candidates+=(
            "${HOME}/anaconda3/bin/python3"
            "${HOME}/anaconda3/bin/python"
            "${HOME}/miniconda3/bin/python3"
            "${HOME}/miniconda3/bin/python"
            "${HOME}/miniforge3/bin/python3"
            "${HOME}/miniforge3/bin/python"
        )
    fi

    command_python="$(command -v python3 2>/dev/null || true)"
    if [ -n "$command_python" ]; then
        candidates+=("$command_python")
    fi

    local py=""
    for py in "${candidates[@]}"; do
        [ -x "$py" ] || continue
        if python_has_pyserial "$py"; then
            export WHEELDOG_PYTHON="$py"
            echo "=== 电机桥 Python: $WHEELDOG_PYTHON ==="
            return 0
        fi
    done

    echo "错误: 找不到已安装 pyserial 的 Python，电机桥无法启动" >&2
    echo "提示: 可执行 /home/gu/anaconda3/bin/python3 -m pip install pyserial，或设置 WHEELDOG_PYTHON=/path/to/python" >&2
    return 1
}

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
    echo "错误: 缺少构建模式标记，请先执行 ./build.sh hw" >&2
    exit 1
fi
if [ "$(cat build_local/build_mode.txt)" != "OFF" ]; then
    echo "错误: build_local 不是实机版本，请先执行 ./build.sh hw" >&2
    exit 1
fi
if [ ! -f build_local/source_root.txt ] ||
   [ "$(cat build_local/source_root.txt 2>/dev/null)" != "$dirname" ]; then
    echo "错误: build_local 不是从当前源码目录构建，请执行 ./build.sh hw" >&2
    exit 1
fi
if [ ! -x build_local/rl_deploy ]; then
    echo "错误: 缺少 build_local/rl_deploy，请先执行 ./build.sh hw" >&2
    exit 1
fi
if [ ! -x "${ROBO_PRO_INSTALL_PATH}/bin/robo_core_main" ]; then
    echo "错误: 找不到 ${ROBO_PRO_INSTALL_PATH}/bin/robo_core_main" >&2
    exit 1
fi

if [ "${WHEELDOG_MOTOR_REQUIRED:-1}" = "0" ]; then
    select_motor_python || echo "警告: 未找到可用 pyserial；WHEELDOG_MOTOR_REQUIRED=0，继续启动" >&2
else
    select_motor_python || exit 1
fi

rm -f /dev/shm/"${ROBO_WORKSPACE}"*

timestamp="$(date +%Y%m%d_%H%M%S)"
LOGFILE="${LOGFILE:-${dirname}/log/standup_monitor_${timestamp}.log}"
if ! prepare_logfile "$LOGFILE"; then
    fallback_logfile="${dirname}/log/standup_monitor_${timestamp}_$$.log"
    echo "警告: 无法写入 LOGFILE=$LOGFILE，改用 $fallback_logfile" >&2
    prepare_logfile "$fallback_logfile" || {
        echo "错误: 无法创建日志文件 $fallback_logfile" >&2
        exit 1
    }
fi

echo "=== 真机控制日志: $LOGFILE ==="
policy_sha="$(sha256sum "$WHEELDOG_POLICY_PATH" 2>/dev/null | awk '{print $1}')"
if [ -z "$policy_sha" ]; then
    echo "错误: 无法读取 ONNX 策略 $WHEELDOG_POLICY_PATH" >&2
    exit 1
fi
echo "=== ONNX: $WHEELDOG_POLICY_PATH sha256=$policy_sha ===" | tee -a "$LOGFILE"
echo "=== RL: gains=robot_model_config.hpp, Kd floors(hip/knee)=${RL_HIP_KD}/${RL_KNEE_KD}, policy=50Hz, monitor=5Hz ===" | tee -a "$LOGFILE"
echo "=== Transport: PC->MCU=${WHEELDOG_MCU_SETPOINT_HZ}Hz, MCU watchdog=${WHEELDOG_MCU_COMMAND_TIMEOUT_MS}ms, PC feedback latch=${WHEELDOG_RUNTIME_FEEDBACK_TIMEOUT_MS}ms ===" | tee -a "$LOGFILE"
stdbuf -oL "${ROBO_PRO_INSTALL_PATH}/bin/robo_core_main" \
    "PROGRAM_ROOT_PATH" "/Sources/quardLeg.json" 2>&1 | tee -a "$LOGFILE"
