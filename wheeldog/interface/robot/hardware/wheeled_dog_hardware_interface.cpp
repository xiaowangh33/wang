#include "hardware/wheeled_dog_hardware_interface.hpp"

#include "custom_types.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace interface {
namespace {

constexpr uint32_t kAllMotorsMask = (1u << robot_model::kTotalDof) - 1u;
constexpr uint32_t kMaxObservationAgeMs = 10u;
// Matching is keyed by MCU observation_seq.  Thirty milliseconds absorbs a
// short shared-hub/CDC scheduling delay without accepting mixed epochs.
constexpr uint32_t kMaxObservationSkewUs = 30000u;

std::string ShmOpenName(const std::string& name) {
    return "/" + name;
}

std::string ToString(int value) {
    return std::to_string(value);
}

std::string ToString(float value) {
    return std::to_string(value);
}

}  // namespace

WheeledDogHardwareInterface::WheeledDogHardwareInterface(const std::string& robot_name)
    : RobotInterface(robot_name, robot_model::kTotalDof) {
    joint_cmd_ = MatXf::Zero(dof_num_, 5);
    joint_pos_ = VecXf::Zero(dof_num_);
    joint_vel_ = VecXf::Zero(dof_num_);
    joint_tau_ = VecXf::Zero(dof_num_);
    imu_ = std::make_unique<hal::HipnucImuHAL>();

    high_level_tick_hz_ = ReadIntEnv(
        "WHEELDOG_HW_TICK_HZ", robot_model::kHardwareHighLevelTickHz, 20, 4000);
    setpoint_loop_hz_ = ReadIntEnv(
        "WHEELDOG_MCU_SETPOINT_HZ",
        robot_model::kHardwarePcToMcuSetpointHz,
        10,
        robot_model::kHardwarePcToMcuSetpointHz);
    motor_required_ = ReadBoolEnv("WHEELDOG_MOTOR_REQUIRED", true);
    motor_enable_on_start_ = ReadBoolEnv("WHEELDOG_MOTOR_ENABLE_ON_START", true);
    motor_verified_enable_ = ReadBoolEnv("WHEELDOG_MOTOR_VERIFIED_ENABLE", true);
    motor_motion_verify_ = ReadBoolEnv("WHEELDOG_MOTOR_MOTION_VERIFY", false);
    runtime_feedback_timeout_ms_ =
        ReadIntEnv("WHEELDOG_RUNTIME_FEEDBACK_TIMEOUT_MS", 500, 100, 2000);
    mcu_command_timeout_ms_ =
        ReadIntEnv("WHEELDOG_MCU_COMMAND_TIMEOUT_MS", 300, 100, 2000);
}

WheeledDogHardwareInterface::~WheeledDogHardwareInterface() {
    Stop();
}

bool WheeledDogHardwareInterface::ReadBoolEnv(const char* name, bool fallback) {
    const char* value = std::getenv(name);
    if (!value || value[0] == '\0') {
        return fallback;
    }
    return value[0] == '1' || value[0] == 'y' || value[0] == 'Y' ||
           value[0] == 't' || value[0] == 'T';
}

int WheeledDogHardwareInterface::ReadIntEnv(
    const char* name, int fallback, int min_value, int max_value) {
    const char* value = std::getenv(name);
    if (!value || value[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    if (end == value || *end != '\0' || parsed < min_value || parsed > max_value) {
        std::cerr << "[HardwareInterface] Ignoring invalid " << name << "="
                  << value << ", using " << fallback << "\n";
        return fallback;
    }
    return static_cast<int>(parsed);
}

float WheeledDogHardwareInterface::ReadFloatEnv(
    const char* name, float fallback, float min_value, float max_value) {
    const char* value = std::getenv(name);
    if (!value || value[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const float parsed = std::strtof(value, &end);
    if (end == value || *end != '\0' || !std::isfinite(parsed) ||
        parsed < min_value || parsed > max_value) {
        std::cerr << "[HardwareInterface] Ignoring invalid " << name << "="
                  << value << ", using " << fallback << "\n";
        return fallback;
    }
    return parsed;
}

std::string WheeledDogHardwareInterface::ReadStringEnv(
    const char* name, const std::string& fallback) {
    const char* value = std::getenv(name);
    return (value && value[0] != '\0') ? std::string(value) : fallback;
}

void WheeledDogHardwareInterface::Start() {
    if (start_flag_.load(std::memory_order_acquire)) {
        return;
    }

    start_time_ = std::chrono::steady_clock::now();
    last_feedback_time_ = start_time_;
    interface_time_s_.store(0.0, std::memory_order_release);

    const bool imu_required = ReadBoolEnv("WHEELDOG_IMU_REQUIRED", true);
    if (!imu_->Start()) {
        if (imu_required) {
            throw std::runtime_error("HiPNUC IMU failed to start");
        }
        std::cerr << "[HardwareInterface] HiPNUC IMU unavailable; continuing with fallback IMU "
                  << "because WHEELDOG_IMU_REQUIRED=0\n";
    }

    try {
        StartBridge();
        const float bridge_timeout_s =
            ReadFloatEnv("WHEELDOG_MOTOR_START_TIMEOUT_S", 70.0f, 1.0f, 120.0f);
        if (!WaitForBridgeReady(bridge_timeout_s)) {
            const std::string msg = "motor bridge did not become ready: " + BridgeStatusMessage();
            if (motor_required_) {
                throw std::runtime_error(msg);
            }
            std::cerr << "[HardwareInterface] " << msg << "\n";
        }
        const float feedback_timeout_s =
            ReadFloatEnv("WHEELDOG_MOTOR_FEEDBACK_TIMEOUT_S", 5.0f, 0.2f, 60.0f);
        if (!WaitForOnlineFeedback(feedback_timeout_s)) {
            const std::string msg = "motor feedback not online: " + BridgeStatusMessage();
            if (motor_required_) {
                throw std::runtime_error(msg);
            }
            std::cerr << "[HardwareInterface] " << msg << "\n";
        }
        ConfirmAbsoluteCalibration();
        runtime_safety_latched_.store(false, std::memory_order_release);
    } catch (...) {
        StopBridge();
        imu_->Stop();
        throw;
    }

    start_flag_.store(true, std::memory_order_release);
    state_clock_thread_ = std::thread(&WheeledDogHardwareInterface::StateClockLoop, this);
    motor_thread_ = std::thread(&WheeledDogHardwareInterface::MotorLoop, this);

    std::cout << "[HardwareInterface] Started. High-level tick=" << high_level_tick_hz_
              << " Hz, PC->MCU setpoint=" << setpoint_loop_hz_
              << " Hz, bridge_hz=" << (shm_ ? shm_->actual_hz : 0.0)
              << ", MCU control="
              << (shm_ ? std::min(shm_->mcu_control_hz[0], shm_->mcu_control_hz[1]) : 0.0f)
              << " Hz, observation_age<="
              << (shm_ ? shm_->observation_max_sample_age_ms : 0u)
              << " ms, dual_MCU_arrival_skew="
              << (shm_ ? (static_cast<double>(shm_->observation_bridge_skew_us) / 1000.0)
                       : 0.0)
              << " ms\n";
}

void WheeledDogHardwareInterface::Stop() {
    start_flag_.store(false, std::memory_order_release);
    if (motor_thread_.joinable() &&
        motor_thread_.get_id() != std::this_thread::get_id()) {
        motor_thread_.join();
    }
    if (state_clock_thread_.joinable() &&
        state_clock_thread_.get_id() != std::this_thread::get_id()) {
        state_clock_thread_.join();
    }
    StopBridge();
    if (imu_) {
        imu_->Stop();
    }
}

bool WheeledDogHardwareInterface::IsRuntimeSafetyLatched() const {
    return runtime_safety_latched_.load(std::memory_order_acquire);
}

void WheeledDogHardwareInterface::StartBridge() {
    if (shm_) {
        return;
    }

    shm_name_ = "wheeldog_motor_" + std::to_string(getpid()) + "_" +
                std::to_string(reinterpret_cast<uintptr_t>(this));
    const std::string open_name = ShmOpenName(shm_name_);
    shm_fd_ = shm_open(open_name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
    if (shm_fd_ < 0) {
        throw std::runtime_error("shm_open failed: " + std::string(std::strerror(errno)));
    }
    if (ftruncate(shm_fd_, sizeof(hardware::MotorBridgeSharedData)) != 0) {
        const std::string err = std::strerror(errno);
        shm_unlink(open_name.c_str());
        throw std::runtime_error("ftruncate motor bridge shm failed: " + err);
    }
    void* mapped = mmap(nullptr, sizeof(hardware::MotorBridgeSharedData),
                        PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (mapped == MAP_FAILED) {
        const std::string err = std::strerror(errno);
        shm_unlink(open_name.c_str());
        throw std::runtime_error("mmap motor bridge shm failed: " + err);
    }
    shm_ = static_cast<hardware::MotorBridgeSharedData*>(mapped);
    std::memset(shm_, 0, sizeof(hardware::MotorBridgeSharedData));
    shm_->magic = hardware::kMotorBridgeMagic;
    shm_->version = hardware::kMotorBridgeVersion;
    shm_->motor_count = hardware::kMotorBridgeMotorCount;
    std::snprintf(shm_->status_message, sizeof(shm_->status_message), "created by C++");

    const std::string python = ReadStringEnv("WHEELDOG_PYTHON", "python3");
    const std::string script = ReadStringEnv(
        "WHEELDOG_MOTOR_BRIDGE_SCRIPT",
        GetAbsPath() + "/third_party/rs06_mit_control/wheeldog_mcu_bridge.py");
    const std::string ports =
        ReadStringEnv("WHEELDOG_MOTOR_PORTS", "/dev/ttyACM0,/dev/ttyACM1");
    const int baudrate = ReadIntEnv("WHEELDOG_MOTOR_BAUD", 921600, 9600, 4000000);

    std::vector<std::string> args = {
        python,
        script,
        "--shm-name",
        shm_name_,
        "--ports",
        ports,
        "--baudrate",
        ToString(baudrate),
        "--hz",
        ToString(static_cast<float>(setpoint_loop_hz_)),
        "--timeout-ms",
        ToString(mcu_command_timeout_ms_),
        "--parent-pid",
        ToString(static_cast<int>(getpid())),
    };
    if (motor_enable_on_start_) {
        args.push_back(motor_verified_enable_ ? "--verified-enable" : "--enable");
        if (motor_motion_verify_) {
            args.push_back("--motion-verify");
        }
    }

    bridge_pid_ = fork();
    if (bridge_pid_ < 0) {
        throw std::runtime_error("fork motor bridge failed: " +
                                 std::string(std::strerror(errno)));
    }
    if (bridge_pid_ == 0) {
        std::vector<char*> argv;
        argv.reserve(args.size() + 1);
        for (auto& arg : args) {
            argv.push_back(arg.data());
        }
        argv.push_back(nullptr);
        execvp(argv[0], argv.data());
        std::cerr << "execvp motor bridge failed: " << std::strerror(errno) << "\n";
        _exit(127);
    }
}

void WheeledDogHardwareInterface::StopBridge() {
    if (shm_) {
        uint32_t seq = __atomic_load_n(&shm_->command_seq, __ATOMIC_RELAXED);
        if ((seq & 1u) != 0u) {
            ++seq;
        }
        __atomic_store_n(&shm_->command_seq, seq + 1u, __ATOMIC_SEQ_CST);
        for (int i = 0; i < robot_model::kTotalDof; ++i) {
            shm_->command_kp[i] = 0.0f;
            shm_->command_q_des[i] = 0.0f;
            shm_->command_kd[i] = 0.0f;
            shm_->command_dq_des[i] = 0.0f;
            shm_->command_tau_ff[i] = 0.0f;
        }
        __atomic_thread_fence(__ATOMIC_RELEASE);
        __atomic_store_n(&shm_->command_seq, seq + 2u, __ATOMIC_RELEASE);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        shm_->control_flags |= hardware::kMotorBridgeControlExit;
    }

    if (bridge_pid_ > 0) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        int status = 0;
        while (std::chrono::steady_clock::now() < deadline) {
            const pid_t ret = waitpid(bridge_pid_, &status, WNOHANG);
            if (ret == bridge_pid_) {
                bridge_pid_ = -1;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (bridge_pid_ > 0) {
            kill(bridge_pid_, SIGTERM);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            if (waitpid(bridge_pid_, &status, WNOHANG) == 0) {
                kill(bridge_pid_, SIGKILL);
                waitpid(bridge_pid_, &status, 0);
            }
            bridge_pid_ = -1;
        }
    }

    if (shm_) {
        munmap(shm_, sizeof(hardware::MotorBridgeSharedData));
        shm_ = nullptr;
    }
    if (shm_fd_ >= 0) {
        close(shm_fd_);
        shm_fd_ = -1;
    }
    if (!shm_name_.empty()) {
        shm_unlink(ShmOpenName(shm_name_).c_str());
        shm_name_.clear();
    }
}

bool WheeledDogHardwareInterface::WaitForBridgeReady(double timeout_s) {
    if (!shm_) {
        return false;
    }
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(timeout_s));
    while (std::chrono::steady_clock::now() < deadline) {
        int status = 0;
        if (bridge_pid_ > 0 && waitpid(bridge_pid_, &status, WNOHANG) == bridge_pid_) {
            bridge_pid_ = -1;
            return false;
        }
        if (shm_->status_flags & hardware::kMotorBridgeStatusFault) {
            return false;
        }
        const bool ready = shm_->status_flags & hardware::kMotorBridgeStatusReady;
        const bool enabled_ok = !motor_enable_on_start_ ||
                                (shm_->status_flags & hardware::kMotorBridgeStatusEnabled);
        if (ready && enabled_ok) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

bool WheeledDogHardwareInterface::WaitForOnlineFeedback(double timeout_s) {
    if (!shm_) {
        return false;
    }
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(timeout_s));
    while (std::chrono::steady_clock::now() < deadline) {
        if (shm_->status_flags & hardware::kMotorBridgeStatusFault) {
            return false;
        }
        if ((shm_->online_mask & kAllMotorsMask) == kAllMotorsMask &&
            shm_->feedback_seq > 0) {
            return true;
        }
        if (!motor_required_ && shm_->feedback_seq > 0) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return !motor_required_ && shm_->feedback_seq > 0;
}

void WheeledDogHardwareInterface::ConfirmAbsoluteCalibration() {
    if (!shm_) {
        return;
    }
    UpdateStateFromBridge();
    std::cout << "[HardwareInterface] MCU supplies calibrated absolute URDF "
              << "joint feedback with independent raw encoder boundaries.\n";
}

void WheeledDogHardwareInterface::StateClockLoop() {
    const auto period = std::chrono::duration<double>(
        1.0 / static_cast<double>(high_level_tick_hz_));
    auto next = std::chrono::steady_clock::now();
    while (start_flag_.load(std::memory_order_acquire)) {
        const auto now = std::chrono::steady_clock::now();
        interface_time_s_.store(
            std::chrono::duration<double>(now - start_time_).count(),
            std::memory_order_release);
        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(next);
        if (std::chrono::steady_clock::now() > next + std::chrono::milliseconds(20)) {
            next = std::chrono::steady_clock::now();
        }
    }
}

void WheeledDogHardwareInterface::MotorLoop() {
    const auto period = std::chrono::duration<double>(
        1.0 / static_cast<double>(setpoint_loop_hz_));
    auto next = std::chrono::steady_clock::now();
    while (start_flag_.load(std::memory_order_acquire)) {
        UpdateStateFromBridge();
        MatXf cmd;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            cmd = joint_cmd_;
        }
        if (!runtime_safety_latched_.load(std::memory_order_acquire)) {
            std::string reason;
            if (!RuntimeControlHealthy(&reason)) {
                runtime_safety_latched_.store(true, std::memory_order_release);
                std::cerr << "[HardwareInterface] RUNTIME SAFETY LATCH: " << reason
                          << "; commands forced to zero until process restart\n";
            }
        }
        if (runtime_safety_latched_.load(std::memory_order_acquire)) {
            cmd = MatXf::Zero(dof_num_, 5);
        }
        PublishMotorCommands(cmd);

        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(next);
        if (std::chrono::steady_clock::now() > next + std::chrono::milliseconds(20)) {
            next = std::chrono::steady_clock::now();
        }
    }
}

bool WheeledDogHardwareInterface::RuntimeControlHealthy(std::string* reason) const {
    const auto fail = [reason](const std::string& text) {
        if (reason) {
            *reason = text;
        }
        return false;
    };

    if (!imu_ || !imu_->IsDataValid()) {
        return fail("IMU feedback stale or invalid");
    }

    // A no-power/dry-run preflight intentionally has no motor observations.
    if (!motor_enable_on_start_) {
        return true;
    }
    if (!shm_) {
        return fail("motor bridge shared memory unavailable");
    }
    if ((shm_->status_flags & hardware::kMotorBridgeStatusFault) != 0u) {
        return fail("motor bridge reported fault: " + BridgeStatusMessage());
    }
    if ((shm_->status_flags & hardware::kMotorBridgeStatusReady) == 0u ||
        (shm_->status_flags & hardware::kMotorBridgeStatusEnabled) == 0u) {
        return fail("motor bridge lost ready/enabled state");
    }
    if ((shm_->online_mask & kAllMotorsMask) != kAllMotorsMask ||
        (shm_->fresh_mask & kAllMotorsMask) != kAllMotorsMask ||
        (shm_->fast_feedback_valid_mask & kAllMotorsMask) != kAllMotorsMask) {
        return fail("one or more motor observations are offline/stale/unqualified");
    }
    if ((shm_->fault_mask & kAllMotorsMask) != 0u) {
        return fail("one or more motors reported a fault");
    }
    if (shm_->observation_max_sample_age_ms > kMaxObservationAgeMs ||
        shm_->observation_bridge_skew_us > kMaxObservationSkewUs) {
        return fail("dual-MCU observation age/skew exceeded limit");
    }
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_feedback_time_).count();
    if (age_ms > runtime_feedback_timeout_ms_) {
        return fail("coherent motor feedback stopped");
    }
    return true;
}

void WheeledDogHardwareInterface::UpdateStateFromBridge() {
    if (!shm_) {
        return;
    }

    VecXf pos = VecXf::Zero(dof_num_);
    VecXf vel = VecXf::Zero(dof_num_);
    VecXf tau = VecXf::Zero(dof_num_);
    uint32_t seq = 0u;
    bool snapshot_ok = false;

    for (int attempt = 0; attempt < 4 && !snapshot_ok; ++attempt) {
        const uint32_t begin =
            __atomic_load_n(&shm_->feedback_seq, __ATOMIC_ACQUIRE);
        if ((begin & 1u) != 0u) {
            continue;
        }
        for (int i = 0; i < dof_num_; ++i) {
            const float q = shm_->feedback_position[i];
            const float dq = shm_->feedback_velocity[i];
            const float joint_tau = shm_->feedback_torque[i];
            pos(i) = IsFinite(q) ? q : 0.0f;
            vel(i) = IsFinite(dq) ? dq : 0.0f;
            tau(i) = IsFinite(joint_tau) ? joint_tau : 0.0f;
        }
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        const uint32_t end =
            __atomic_load_n(&shm_->feedback_seq, __ATOMIC_ACQUIRE);
        snapshot_ok = begin == end && (end & 1u) == 0u;
        seq = end;
    }
    if (!snapshot_ok) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        joint_pos_ = pos;
        joint_vel_ = vel;
        joint_tau_ = tau;
    }

    if (seq != last_feedback_seq_) {
        last_feedback_seq_ = seq;
        last_feedback_time_ = std::chrono::steady_clock::now();
    }
}

void WheeledDogHardwareInterface::PublishMotorCommands(const MatXf& joint_cmd) {
    if (!shm_ || joint_cmd.rows() < dof_num_ || joint_cmd.cols() < 5) {
        return;
    }

    // Sanitize and clamp outside the seqlock write window.  The Python bridge
    // only needs the sequence to remain odd while the final arrays are copied.
    std::array<float, robot_model::kTotalDof> command_kp{};
    std::array<float, robot_model::kTotalDof> command_q_des{};
    std::array<float, robot_model::kTotalDof> command_kd{};
    std::array<float, robot_model::kTotalDof> command_dq_des{};
    std::array<float, robot_model::kTotalDof> command_tau_ff{};
    for (int i = 0; i < dof_num_; ++i) {
        const auto mode = robot_model::kJoints[i].mode;
        float kp = IsFinite(joint_cmd(i, 0))
                       ? std::max(0.0f, joint_cmd(i, 0))
                       : 0.0f;
        float q_des = IsFinite(joint_cmd(i, 1)) ? joint_cmd(i, 1) : 0.0f;
        float kd = IsFinite(joint_cmd(i, 2))
                       ? std::max(0.0f, joint_cmd(i, 2))
                       : 0.0f;
        float dq_des = IsFinite(joint_cmd(i, 3)) ? joint_cmd(i, 3) : 0.0f;
        float tau_ff = IsFinite(joint_cmd(i, 4)) ? joint_cmd(i, 4) : 0.0f;
        const float velocity_limit =
            (mode == robot_model::ActuatorMode::Velocity)
                ? robot_model::kHardwareWheelMotionVirtualVelocityLimitRadps
                : robot_model::kHardwareLegVelocityLimitRadps;
        dq_des = std::clamp(dq_des, -velocity_limit, velocity_limit);
        tau_ff = std::clamp(tau_ff,
                            -robot_model::kHardwareBringupTorqueLimitNm,
                            robot_model::kHardwareBringupTorqueLimitNm);
        if (mode == robot_model::ActuatorMode::Velocity) {
            kp = 0.0f;
            q_des = 0.0f;
            // RS01 communication type 1 consumes the training actuator's Kd
            // directly: tau = Kd * (dq_des - dq). dq_des is therefore a
            // virtual torque-producing target (up to the protocol's 44
            // rad/s). The MCU clamps wheel torque to 17 Nm; measured wheel
            // speed derating/tripping is intentionally disabled.
            tau_ff = 0.0f;
        }
        command_kp[i] = kp;
        command_q_des[i] = q_des;
        command_kd[i] = kd;
        command_dq_des[i] = dq_des;
        command_tau_ff[i] = tau_ff;
    }

    uint32_t seq = __atomic_load_n(&shm_->command_seq, __ATOMIC_RELAXED);
    if ((seq & 1u) != 0u) {
        ++seq;
    }
    __atomic_store_n(&shm_->command_seq, seq + 1u, __ATOMIC_SEQ_CST);
    std::memcpy(shm_->command_kp, command_kp.data(), sizeof(shm_->command_kp));
    std::memcpy(
        shm_->command_q_des, command_q_des.data(), sizeof(shm_->command_q_des));
    std::memcpy(shm_->command_kd, command_kd.data(), sizeof(shm_->command_kd));
    std::memcpy(
        shm_->command_dq_des, command_dq_des.data(), sizeof(shm_->command_dq_des));
    std::memcpy(
        shm_->command_tau_ff, command_tau_ff.data(), sizeof(shm_->command_tau_ff));
    __atomic_thread_fence(__ATOMIC_RELEASE);
    __atomic_store_n(&shm_->command_seq, seq + 2u, __ATOMIC_RELEASE);
}

bool WheeledDogHardwareInterface::IsFinite(float value) const {
    return std::isfinite(value);
}

std::string WheeledDogHardwareInterface::BridgeStatusMessage() const {
    if (!shm_) {
        return "bridge shm not mapped";
    }
    const size_t len = strnlen(shm_->status_message, sizeof(shm_->status_message));
    return std::string(shm_->status_message, shm_->status_message + len);
}

double WheeledDogHardwareInterface::GetInterfaceTimeStamp() {
    return interface_time_s_.load(std::memory_order_acquire);
}

VecXf WheeledDogHardwareInterface::GetJointPosition() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return joint_pos_;
}

VecXf WheeledDogHardwareInterface::GetJointVelocity() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return joint_vel_;
}

VecXf WheeledDogHardwareInterface::GetJointTorque() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return joint_tau_;
}

Vec3f WheeledDogHardwareInterface::GetImuRpy() {
    return imu_ ? imu_->GetRpy() : Vec3f::Zero();
}

Vec3f WheeledDogHardwareInterface::GetImuAcc() {
    return imu_ ? imu_->GetAcc() : Vec3f(0.0f, 0.0f, types::gravity);
}

Vec3f WheeledDogHardwareInterface::GetImuOmega() {
    return imu_ ? imu_->GetOmega() : Vec3f::Zero();
}

void WheeledDogHardwareInterface::SetJointCommand(
    Eigen::Matrix<float, Eigen::Dynamic, 5> input) {
    if (input.rows() < dof_num_ || input.cols() < 5) {
        runtime_safety_latched_.store(true, std::memory_order_release);
        return;
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    joint_cmd_ = input.topRows(dof_num_);
}

MatXf WheeledDogHardwareInterface::GetJointCommand() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return joint_cmd_;
}

VecXf WheeledDogHardwareInterface::GetContactForce() {
    return VecXf::Zero(4);
}

}  // namespace interface
