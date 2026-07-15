#pragma once

#include "imu_hal.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace interface::hal {

struct ImuAxisRemap {
    std::array<int, 3> index{{0, 1, 2}};
    std::array<float, 3> sign{{1.0f, 1.0f, 1.0f}};
};

class HipnucImuHAL final : public ImuHAL {
public:
    struct Config {
        std::string port = "/dev/ttyUSB0";
        // Current bench HI91 powers up at 115200 baud. This value was
        // verified from raw framing and parsed frames on 2026-07-10.
        int baud = 115200;
        bool send_log_enable = true;
        bool software_filter = true;
        float first_frame_timeout_s = 5.0f;
        float stale_timeout_s = 0.25f;
        float acc_lpf_cutoff_hz = 30.0f;
        float gyro_static_norm_radps = 0.12f;
        float gyro_bias_learn_rate = 0.004f;
        // HI91 fields were measured on the installed sensor on 2026-07-14.
        // Its acc/gyro vector fields are rotated +90 deg relative to the URDF
        // body axes, while its reported Euler components use the expected
        // roll axis but the opposite pitch convention.
        //
        // URDF body vector = [raw_y, -raw_x, raw_z]
        ImuAxisRemap vector_map{{1, 0, 2}, {1.0f, -1.0f, 1.0f}};
        // URDF RPY = [raw_roll, -raw_pitch, raw_yaw]
        ImuAxisRemap rpy_map{{0, 1, 2}, {1.0f, -1.0f, 1.0f}};
        bool use_rpy_component_map = true;
    };

    HipnucImuHAL();
    explicit HipnucImuHAL(Config config);
    ~HipnucImuHAL() override;

    bool Start() override;
    void Stop() override;
    types::Vec3f GetRpy() override;
    types::Vec3f GetAcc() override;
    types::Vec3f GetOmega() override;
    bool IsDataValid() override;

    uint64_t GetFrameCount() const;
    double GetLastPacketTimeSec() const;

    static Config ConfigFromEnv();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace interface::hal
