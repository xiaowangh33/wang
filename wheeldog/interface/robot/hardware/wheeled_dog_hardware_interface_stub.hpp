#pragma once

#include "hardware/hipnuc_imu_hal.hpp"
#include "robot_interface.h"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace interface {

class WheeledDogHardwareInterfaceStub : public RobotInterface {
public:
    explicit WheeledDogHardwareInterfaceStub(const std::string& robot_name)
        : RobotInterface(robot_name, robot_model::kTotalDof) {
        joint_cmd_ = MatXf::Zero(dof_num_, 5);
        joint_pos_ = VecXf::Zero(dof_num_);
        joint_vel_ = VecXf::Zero(dof_num_);
        joint_tau_ = VecXf::Zero(dof_num_);
        imu_ = std::make_unique<hal::HipnucImuHAL>();
        tick_hz_ = ReadTickHzFromEnv();
    }

    ~WheeledDogHardwareInterfaceStub() override { Stop(); }

    void Start() override {
        if (start_flag_.load(std::memory_order_acquire)) {
            return;
        }

        start_time_ = std::chrono::steady_clock::now();
        interface_time_s_.store(0.0, std::memory_order_release);
        start_flag_.store(true, std::memory_order_release);
        clock_thread_ = std::thread(&WheeledDogHardwareInterfaceStub::ClockLoop, this);

        const bool imu_required = ReadBoolEnv("WHEELDOG_IMU_REQUIRED", true);
        if (!imu_->Start()) {
            if (imu_required) {
                Stop();
                throw std::runtime_error("HiPNUC IMU failed to start");
            }
            std::cerr << "[HardwareInterface] HiPNUC IMU unavailable; continuing with fallback IMU "
                      << "because WHEELDOG_IMU_REQUIRED=0\n";
        }

        std::cout << "[HardwareInterface] Started. High-level tick=" << tick_hz_
                  << " Hz, motor backend is still stubbed.\n";
    }

    void Stop() override {
        start_flag_.store(false, std::memory_order_release);
        if (imu_) {
            imu_->Stop();
        }
        if (clock_thread_.joinable() &&
            clock_thread_.get_id() != std::this_thread::get_id()) {
            clock_thread_.join();
        }
    }

    double GetInterfaceTimeStamp() override {
        return interface_time_s_.load(std::memory_order_acquire);
    }

    VecXf GetJointPosition() override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return joint_pos_;
    }
    VecXf GetJointVelocity() override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return joint_vel_;
    }
    VecXf GetJointTorque() override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return joint_tau_;
    }
    Vec3f GetImuRpy() override {
        return imu_ ? imu_->GetRpy() : Vec3f::Zero();
    }
    Vec3f GetImuAcc() override {
        return imu_ ? imu_->GetAcc() : Vec3f(0.0f, 0.0f, types::gravity);
    }
    Vec3f GetImuOmega() override {
        return imu_ ? imu_->GetOmega() : Vec3f::Zero();
    }
    void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) override {
        std::lock_guard<std::mutex> lock(command_mutex_);
        joint_cmd_ = input;
    }
    MatXf GetJointCommand() override {
        std::lock_guard<std::mutex> lock(command_mutex_);
        return joint_cmd_;
    }
    VecXf GetContactForce() override { return VecXf::Zero(4); }

private:
    static bool ReadBoolEnv(const char* name, bool fallback) {
        const char* value = std::getenv(name);
        if (!value || value[0] == '\0') {
            return fallback;
        }
        return value[0] == '1' || value[0] == 'y' || value[0] == 'Y' ||
               value[0] == 't' || value[0] == 'T';
    }

    static int ReadTickHzFromEnv() {
        const char* value = std::getenv("WHEELDOG_HW_TICK_HZ");
        if (!value || value[0] == '\0') {
            return robot_model::kHardwareHighLevelTickHz;
        }
        char* end = nullptr;
        const long parsed = std::strtol(value, &end, 10);
        if (end == value || *end != '\0' || parsed < 20 || parsed > 4000) {
            std::cerr << "[HardwareInterface] Ignoring invalid WHEELDOG_HW_TICK_HZ="
                      << value << ", using "
                      << robot_model::kHardwareHighLevelTickHz << " Hz\n";
            return robot_model::kHardwareHighLevelTickHz;
        }
        return static_cast<int>(parsed);
    }

    void ClockLoop() {
        const auto period = std::chrono::duration<double>(1.0 / static_cast<double>(tick_hz_));
        auto next = std::chrono::steady_clock::now();
        while (start_flag_.load(std::memory_order_acquire)) {
            const auto now = std::chrono::steady_clock::now();
            interface_time_s_.store(std::chrono::duration<double>(now - start_time_).count(),
                                    std::memory_order_release);
            next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
            std::this_thread::sleep_until(next);
            if (std::chrono::steady_clock::now() > next + std::chrono::milliseconds(20)) {
                next = std::chrono::steady_clock::now();
            }
        }
    }

    std::unique_ptr<hal::HipnucImuHAL> imu_;
    std::thread clock_thread_;
    std::chrono::steady_clock::time_point start_time_;
    std::atomic<double> interface_time_s_{0.0};
    int tick_hz_ = robot_model::kHardwareHighLevelTickHz;
    mutable std::mutex state_mutex_;
    mutable std::mutex command_mutex_;
    VecXf joint_pos_;
    VecXf joint_vel_;
    VecXf joint_tau_;
};

}  // namespace interface
