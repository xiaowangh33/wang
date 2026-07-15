#include "hardware/hipnuc_imu_hal.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

extern "C" {
#include "hipnuc_dec.h"
#include "serial_port.h"
}

namespace interface::hal {
namespace {

constexpr float kDegToRad = 0.01745329251994329577f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kStandardGravityMps2 = 9.81f;

std::string Trim(std::string s) {
    const auto not_space = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
}

const char* GetEnvFirst(const char* first, const char* second = nullptr) {
    const char* value = std::getenv(first);
    if (value && value[0] != '\0') {
        return value;
    }
    if (!second) {
        return nullptr;
    }
    value = std::getenv(second);
    return (value && value[0] != '\0') ? value : nullptr;
}

int EnvInt(const char* name, int fallback, const char* legacy_name = nullptr) {
    const char* value = GetEnvFirst(name, legacy_name);
    if (!value) {
        return fallback;
    }
    char* end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    if (end == value || *end != '\0') {
        std::cerr << "[HiPNUC IMU] Ignoring invalid " << name << "=" << value << "\n";
        return fallback;
    }
    return static_cast<int>(parsed);
}

float EnvFloat(const char* name, float fallback, const char* legacy_name = nullptr) {
    const char* value = GetEnvFirst(name, legacy_name);
    if (!value) {
        return fallback;
    }
    char* end = nullptr;
    const float parsed = std::strtof(value, &end);
    if (end == value || *end != '\0' || !std::isfinite(parsed)) {
        std::cerr << "[HiPNUC IMU] Ignoring invalid " << name << "=" << value << "\n";
        return fallback;
    }
    return parsed;
}

bool EnvBool(const char* name, bool fallback, const char* legacy_name = nullptr) {
    const char* value = GetEnvFirst(name, legacy_name);
    if (!value) {
        return fallback;
    }
    std::string s = Trim(value);
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (s == "1" || s == "true" || s == "yes" || s == "on") {
        return true;
    }
    if (s == "0" || s == "false" || s == "no" || s == "off") {
        return false;
    }
    std::cerr << "[HiPNUC IMU] Ignoring invalid " << name << "=" << value << "\n";
    return fallback;
}

bool ParseAxisRemap(const char* text, ImuAxisRemap& out) {
    if (!text || text[0] == '\0') {
        return false;
    }

    std::array<int, 3> index{{0, 1, 2}};
    std::array<float, 3> sign{{1.0f, 1.0f, 1.0f}};
    std::array<bool, 3> used{{false, false, false}};
    std::stringstream ss(text);
    std::string token;

    for (int axis = 0; axis < 3; ++axis) {
        if (!std::getline(ss, token, ',')) {
            return false;
        }
        token = Trim(token);
        if (token.empty()) {
            return false;
        }

        float sgn = 1.0f;
        if (token.front() == '+') {
            token.erase(token.begin());
        } else if (token.front() == '-') {
            sgn = -1.0f;
            token.erase(token.begin());
        }
        token = Trim(token);
        if (token.size() != 1) {
            return false;
        }
        const char c = static_cast<char>(std::tolower(static_cast<unsigned char>(token[0])));
        int src = -1;
        if (c == 'x') src = 0;
        if (c == 'y') src = 1;
        if (c == 'z') src = 2;
        if (src < 0 || used[src]) {
            return false;
        }
        used[src] = true;
        index[axis] = src;
        sign[axis] = sgn;
    }

    if (std::getline(ss, token, ',')) {
        return false;
    }

    out.index = index;
    out.sign = sign;
    return true;
}

std::string FormatAxisRemap(const ImuAxisRemap& map) {
    static constexpr char names[] = {'x', 'y', 'z'};
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < 3; ++i) {
        if (i > 0) {
            oss << ", ";
        }
        oss << (map.sign[i] < 0.0f ? "-" : "+") << names[map.index[i]];
    }
    oss << "]";
    return oss.str();
}

types::Mat3f AxisRemapMatrix(const ImuAxisRemap& map) {
    types::Mat3f m = types::Mat3f::Zero();
    for (int row = 0; row < 3; ++row) {
        m(row, map.index[row]) = map.sign[row];
    }
    return m;
}

types::Mat3f RpyToMatrix(const types::Vec3f& rpy) {
    const Eigen::AngleAxisf yaw(rpy(2), types::Vec3f::UnitZ());
    const Eigen::AngleAxisf pitch(rpy(1), types::Vec3f::UnitY());
    const Eigen::AngleAxisf roll(rpy(0), types::Vec3f::UnitX());
    return (yaw * pitch * roll).matrix();
}

types::Vec3f MatrixToRpy(const types::Mat3f& rm) {
    types::Vec3f rpy;
    const float pitch_arg = std::clamp(-rm(2, 0), -1.0f, 1.0f);
    rpy(0) = std::atan2(rm(2, 1), rm(2, 2));
    rpy(1) = std::asin(pitch_arg);
    rpy(2) = std::atan2(rm(1, 0), rm(0, 0));
    return rpy;
}

}  // namespace

struct HipnucImuHAL::Impl {
    explicit Impl(Config c) : config(std::move(c)) {}

    Config config;
    int fd = -1;
    hipnuc_raw_t raw{};

    std::thread read_thread;
    std::atomic<bool> running{false};
    std::atomic<bool> start_finished{false};
    std::atomic<bool> start_ok{false};
    std::atomic<bool> valid{false};
    std::atomic<uint64_t> frame_count{0};
    std::atomic<double> last_packet_time_s{-1.0};

    mutable std::mutex data_mutex;
    types::Vec3f rpy = types::Vec3f::Zero();
    types::Vec3f acc = types::Vec3f(0.0f, 0.0f, types::gravity);
    types::Vec3f omega = types::Vec3f::Zero();

    types::Vec3f acc_lpf = types::Vec3f::Zero();
    types::Vec3f gyro_bias = types::Vec3f::Zero();
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last_filter_time = start_time;
    bool acc_lpf_initialized = false;
    bool warned_invalid_orientation_map = false;

    double NowSec() const {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    }

    types::Vec3f ApplyMap(const types::Vec3f& in, const ImuAxisRemap& map) const {
        types::Vec3f out;
        for (int i = 0; i < 3; ++i) {
            out(i) = map.sign[i] * in(map.index[i]);
        }
        return out;
    }

    void ResetRuntimeState() {
        std::memset(&raw, 0, sizeof(raw));
        valid.store(false, std::memory_order_release);
        frame_count.store(0, std::memory_order_release);
        last_packet_time_s.store(-1.0, std::memory_order_release);
        start_finished.store(false, std::memory_order_release);
        start_ok.store(false, std::memory_order_release);
        start_time = std::chrono::steady_clock::now();
        last_filter_time = start_time;
        acc_lpf_initialized = false;
        acc_lpf.setZero();
        gyro_bias.setZero();
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            rpy.setZero();
            acc = types::Vec3f(0.0f, 0.0f, types::gravity);
            omega.setZero();
        }
    }

    void ReadLoop() {
        uint8_t read_buf[1024];
        fd = serial_port_open(config.port.c_str());
        if (fd < 0) {
            std::cerr << "[HiPNUC IMU] Failed to open " << config.port << "\n";
            start_finished.store(true, std::memory_order_release);
            return;
        }

        if (serial_port_configure(fd, config.baud) < 0) {
            std::cerr << "[HiPNUC IMU] Failed to configure baud " << config.baud << "\n";
            serial_port_close(fd);
            fd = -1;
            start_finished.store(true, std::memory_order_release);
            return;
        }

        start_ok.store(true, std::memory_order_release);
        start_finished.store(true, std::memory_order_release);

        if (config.send_log_enable) {
            char recv_buf[1024];
            const int ret = serial_send_then_recv_str(
                fd, "LOG ENABLE\r\n", "OK\r\n", recv_buf, sizeof(recv_buf), 200);
            if (ret < 0) {
                std::cerr << "[HiPNUC IMU] LOG ENABLE did not return OK; continuing to read stream.\n";
            }
        }

        while (running.load(std::memory_order_acquire)) {
            const int len = serial_port_read(fd, reinterpret_cast<char*>(read_buf), sizeof(read_buf));
            if (len > 0) {
                for (int i = 0; i < len; ++i) {
                    if (hipnuc_input(&raw, read_buf[i]) > 0) {
                        UpdateFromRaw();
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }

        serial_port_close(fd);
        fd = -1;
    }

    void UpdateFromRaw() {
        types::Vec3f new_rpy = types::Vec3f::Zero();
        types::Vec3f new_acc = types::Vec3f::Zero();
        types::Vec3f new_omega = types::Vec3f::Zero();
        bool updated = false;

        if (raw.hi91.tag == 0x91) {
            new_rpy << raw.hi91.roll * kDegToRad,
                       raw.hi91.pitch * kDegToRad,
                       raw.hi91.yaw * kDegToRad;
            new_acc << raw.hi91.acc[0] * kStandardGravityMps2,
                       raw.hi91.acc[1] * kStandardGravityMps2,
                       raw.hi91.acc[2] * kStandardGravityMps2;
            new_omega << raw.hi91.gyr[0] * kDegToRad,
                         raw.hi91.gyr[1] * kDegToRad,
                         raw.hi91.gyr[2] * kDegToRad;
            updated = true;
        } else if (raw.hi83.tag == 0x83) {
            new_rpy << raw.hi83.rpy[0] * kDegToRad,
                       raw.hi83.rpy[1] * kDegToRad,
                       raw.hi83.rpy[2] * kDegToRad;
            new_acc << raw.hi83.acc_b[0], raw.hi83.acc_b[1], raw.hi83.acc_b[2];
            new_omega << raw.hi83.gyr_b[0] * kDegToRad,
                         raw.hi83.gyr_b[1] * kDegToRad,
                         raw.hi83.gyr_b[2] * kDegToRad;
            updated = true;
        }

        if (!updated) {
            return;
        }

        if (config.use_rpy_component_map) {
            new_rpy = ApplyMap(new_rpy, config.rpy_map);
        } else {
            const types::Mat3f sensor_to_body = AxisRemapMatrix(config.vector_map);
            if (sensor_to_body.determinant() > 0.5f) {
                new_rpy = MatrixToRpy(RpyToMatrix(new_rpy) * sensor_to_body.transpose());
            } else {
                if (!warned_invalid_orientation_map) {
                    std::cerr << "[HiPNUC IMU] Axis map is not a right-handed rotation; "
                              << "falling back to component RPY mapping.\n";
                    warned_invalid_orientation_map = true;
                }
                new_rpy = ApplyMap(new_rpy, config.vector_map);
            }
        }
        new_acc = ApplyMap(new_acc, config.vector_map);
        new_omega = ApplyMap(new_omega, config.vector_map);

        types::Vec3f acc_out = new_acc;
        types::Vec3f omega_out = new_omega;
        if (config.software_filter) {
            const auto now = std::chrono::steady_clock::now();
            if (!acc_lpf_initialized) {
                acc_lpf = new_acc;
                last_filter_time = now;
                acc_lpf_initialized = true;
            } else {
                float dt = std::chrono::duration<float>(now - last_filter_time).count();
                last_filter_time = now;
                dt = std::clamp(dt, 1e-5f, 0.05f);
                const float wc = 2.0f * kPi * config.acc_lpf_cutoff_hz;
                const float alpha = (wc * dt) / (1.0f + wc * dt);
                acc_lpf += alpha * (new_acc - acc_lpf);
            }
            acc_out = acc_lpf;

            if (new_omega.norm() < config.gyro_static_norm_radps) {
                gyro_bias = gyro_bias * (1.0f - config.gyro_bias_learn_rate) +
                            new_omega * config.gyro_bias_learn_rate;
            }
            omega_out = new_omega - gyro_bias;
        }

        {
            std::lock_guard<std::mutex> lock(data_mutex);
            rpy = new_rpy;
            acc = acc_out;
            omega = omega_out;
        }

        last_packet_time_s.store(NowSec(), std::memory_order_release);
        frame_count.fetch_add(1, std::memory_order_acq_rel);
        valid.store(true, std::memory_order_release);
    }
};

HipnucImuHAL::HipnucImuHAL() : HipnucImuHAL(ConfigFromEnv()) {}

HipnucImuHAL::HipnucImuHAL(Config config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

HipnucImuHAL::~HipnucImuHAL() {
    Stop();
}

bool HipnucImuHAL::Start() {
    if (impl_->running.load(std::memory_order_acquire)) {
        return true;
    }

    impl_->ResetRuntimeState();
    std::cout << "[HiPNUC IMU] Starting port=" << impl_->config.port
              << " baud=" << impl_->config.baud
              << " vector_map=" << FormatAxisRemap(impl_->config.vector_map)
              << " rpy=" << (impl_->config.use_rpy_component_map ? "component_map" : "matrix_from_axis_map")
              << " rpy_map=" << FormatAxisRemap(impl_->config.rpy_map)
              << " frame=URDF body (+X forward, +Y left, +Z up)"
              << "\n";

    impl_->running.store(true, std::memory_order_release);
    impl_->read_thread = std::thread(&Impl::ReadLoop, impl_.get());

    const auto start_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!impl_->start_finished.load(std::memory_order_acquire) &&
           std::chrono::steady_clock::now() < start_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!impl_->start_finished.load(std::memory_order_acquire) ||
        !impl_->start_ok.load(std::memory_order_acquire)) {
        Stop();
        return false;
    }

    const auto frame_deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<float>(impl_->config.first_frame_timeout_s));
    while (!impl_->valid.load(std::memory_order_acquire) &&
           std::chrono::steady_clock::now() < frame_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (!impl_->valid.load(std::memory_order_acquire)) {
        std::cerr << "[HiPNUC IMU] No valid 0x91/0x83 frame within "
                  << impl_->config.first_frame_timeout_s << " s\n";
        Stop();
        return false;
    }

    std::cout << "[HiPNUC IMU] First frame received. frame_count="
              << impl_->frame_count.load(std::memory_order_acquire) << "\n";
    return true;
}

void HipnucImuHAL::Stop() {
    impl_->running.store(false, std::memory_order_release);
    if (impl_->read_thread.joinable()) {
        impl_->read_thread.join();
    }
}

types::Vec3f HipnucImuHAL::GetRpy() {
    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    return impl_->rpy;
}

types::Vec3f HipnucImuHAL::GetAcc() {
    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    return impl_->acc;
}

types::Vec3f HipnucImuHAL::GetOmega() {
    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    return impl_->omega;
}

bool HipnucImuHAL::IsDataValid() {
    if (!impl_->valid.load(std::memory_order_acquire)) {
        return false;
    }
    const double last = impl_->last_packet_time_s.load(std::memory_order_acquire);
    return last >= 0.0 && (impl_->NowSec() - last) <= impl_->config.stale_timeout_s;
}

uint64_t HipnucImuHAL::GetFrameCount() const {
    return impl_->frame_count.load(std::memory_order_acquire);
}

double HipnucImuHAL::GetLastPacketTimeSec() const {
    return impl_->last_packet_time_s.load(std::memory_order_acquire);
}

HipnucImuHAL::Config HipnucImuHAL::ConfigFromEnv() {
    Config cfg;
    if (const char* port = GetEnvFirst("WHEELDOG_IMU_PORT", "MYDOG_IMU_PORT")) {
        cfg.port = port;
    }
    cfg.baud = EnvInt("WHEELDOG_IMU_BAUD", cfg.baud, "MYDOG_IMU_BAUD");
    cfg.send_log_enable = EnvBool("WHEELDOG_IMU_LOG_ENABLE", cfg.send_log_enable);
    cfg.software_filter = EnvBool("WHEELDOG_IMU_SOFTWARE_FILTER",
                                  cfg.software_filter,
                                  "IMU_SOFTWARE_FILTER");
    cfg.first_frame_timeout_s = EnvFloat("WHEELDOG_IMU_FIRST_FRAME_TIMEOUT_S",
                                         cfg.first_frame_timeout_s);
    cfg.stale_timeout_s = EnvFloat("WHEELDOG_IMU_STALE_TIMEOUT_S", cfg.stale_timeout_s);
    cfg.acc_lpf_cutoff_hz = EnvFloat("WHEELDOG_IMU_ACC_LPF_HZ", cfg.acc_lpf_cutoff_hz);
    cfg.gyro_static_norm_radps = EnvFloat("WHEELDOG_IMU_GYRO_STATIC_RADPS",
                                          cfg.gyro_static_norm_radps);
    cfg.gyro_bias_learn_rate = EnvFloat("WHEELDOG_IMU_GYRO_BIAS_ALPHA",
                                        cfg.gyro_bias_learn_rate);

    if (const char* map = std::getenv("WHEELDOG_IMU_AXIS_MAP")) {
        if (!ParseAxisRemap(map, cfg.vector_map)) {
            std::cerr << "[HiPNUC IMU] Ignoring invalid WHEELDOG_IMU_AXIS_MAP="
                      << map << " (expected e.g. x,y,z or x,-y,-z)\n";
        }
    }
    if (const char* map = std::getenv("WHEELDOG_IMU_RPY_MAP")) {
        if (!ParseAxisRemap(map, cfg.rpy_map)) {
            std::cerr << "[HiPNUC IMU] Ignoring invalid WHEELDOG_IMU_RPY_MAP="
                      << map << " (expected e.g. x,y,z or x,-y,-z)\n";
        } else {
            cfg.use_rpy_component_map = true;
        }
    }

    return cfg;
}

}  // namespace interface::hal
