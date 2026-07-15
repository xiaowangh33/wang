#pragma once

#include "hardware/hipnuc_imu_hal.hpp"
#include "hardware/motor_bridge_shared.hpp"
#include "robot_interface.h"

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <sys/types.h>

namespace interface {

class WheeledDogHardwareInterface : public RobotInterface {
public:
    explicit WheeledDogHardwareInterface(const std::string& robot_name);
    ~WheeledDogHardwareInterface() override;

    void Start() override;
    void Stop() override;

    double GetInterfaceTimeStamp() override;
    VecXf GetJointPosition() override;
    VecXf GetJointVelocity() override;
    VecXf GetJointTorque() override;
    Vec3f GetImuRpy() override;
    Vec3f GetImuAcc() override;
    Vec3f GetImuOmega() override;
    void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) override;
    MatXf GetJointCommand() override;
    bool IsRuntimeSafetyLatched() const override;
    VecXf GetContactForce() override;

private:
    static bool ReadBoolEnv(const char* name, bool fallback);
    static int ReadIntEnv(const char* name, int fallback, int min_value, int max_value);
    static float ReadFloatEnv(const char* name, float fallback, float min_value, float max_value);
    static std::string ReadStringEnv(const char* name, const std::string& fallback);

    void StartBridge();
    void StopBridge();
    bool WaitForBridgeReady(double timeout_s);
    bool WaitForOnlineFeedback(double timeout_s);
    void ConfirmAbsoluteCalibration();

    void StateClockLoop();
    void MotorLoop();
    void UpdateStateFromBridge();
    void PublishMotorCommands(const MatXf& joint_cmd);
    bool RuntimeControlHealthy(std::string* reason) const;

    bool IsFinite(float value) const;
    std::string BridgeStatusMessage() const;

    std::unique_ptr<hal::HipnucImuHAL> imu_;

    std::thread state_clock_thread_;
    std::thread motor_thread_;
    std::chrono::steady_clock::time_point start_time_;
    std::atomic<bool> start_flag_{false};
    std::atomic<double> interface_time_s_{0.0};

    int high_level_tick_hz_ = robot_model::kHardwareHighLevelTickHz;
    int setpoint_loop_hz_ = robot_model::kHardwarePcToMcuSetpointHz;
    bool motor_required_ = true;
    bool motor_enable_on_start_ = true;
    bool motor_verified_enable_ = true;
    bool motor_motion_verify_ = false;
    int runtime_feedback_timeout_ms_ = 500;
    int mcu_command_timeout_ms_ = 300;
    std::atomic<bool> runtime_safety_latched_{false};

    mutable std::mutex state_mutex_;
    mutable std::mutex command_mutex_;
    VecXf joint_pos_;
    VecXf joint_vel_;
    VecXf joint_tau_;

    uint32_t last_feedback_seq_ = 0;
    std::chrono::steady_clock::time_point last_feedback_time_;

    int shm_fd_ = -1;
    std::string shm_name_;
    hardware::MotorBridgeSharedData* shm_ = nullptr;
    pid_t bridge_pid_ = -1;
};

}  // namespace interface
