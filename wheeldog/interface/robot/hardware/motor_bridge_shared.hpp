#pragma once

#include <cstdint>

namespace interface::hardware {

inline constexpr uint32_t kMotorBridgeMagic = 0x57444D42u;  // "WDMB"
inline constexpr uint32_t kMotorBridgeVersion = 4u;
inline constexpr uint32_t kMotorBridgeMotorCount = 16u;

inline constexpr uint32_t kMotorBridgeControlExit = 1u << 0;

inline constexpr uint32_t kMotorBridgeStatusReady = 1u << 0;
inline constexpr uint32_t kMotorBridgeStatusOpen = 1u << 1;
inline constexpr uint32_t kMotorBridgeStatusEnabled = 1u << 2;
inline constexpr uint32_t kMotorBridgeStatusFault = 1u << 3;

#pragma pack(push, 1)
struct MotorBridgeSharedData {
    uint32_t magic;
    uint32_t version;
    uint32_t motor_count;
    uint32_t command_seq;
    uint32_t feedback_seq;
    uint32_t control_flags;
    uint32_t status_flags;
    uint32_t enabled_mask;
    uint32_t online_mask;
    uint32_t fresh_mask;
    uint32_t fault_mask;

    // Joint-coordinate setpoints. The MCU owns PD, safety limiting and the
    // motor-unit/sign conversion. command_seq is an even/odd seqlock.
    float command_kp[kMotorBridgeMotorCount];
    float command_q_des[kMotorBridgeMotorCount];
    float command_kd[kMotorBridgeMotorCount];
    float command_dq_des[kMotorBridgeMotorCount];
    float command_tau_ff[kMotorBridgeMotorCount];

    // Feedback is already in MCU logical joint coordinates.
    float feedback_position[kMotorBridgeMotorCount];
    float feedback_velocity[kMotorBridgeMotorCount];
    float feedback_torque[kMotorBridgeMotorCount];
    float feedback_temperature[kMotorBridgeMotorCount];
    uint32_t feedback_fault_bits[kMotorBridgeMotorCount];

    uint32_t mcu_status_flags[2];
    float mcu_control_hz[2];
    float motor_command_hz[kMotorBridgeMotorCount];
    float motor_feedback_hz[kMotorBridgeMotorCount];
    uint32_t fast_feedback_valid_mask;
    uint32_t reserved0;
    uint32_t reserved1;
    uint32_t observation_seq;
    uint32_t observation_max_sample_age_ms;
    uint32_t observation_bridge_skew_us;
    uint32_t dropped_observation_epochs;
    float motor_sample_age_ms[kMotorBridgeMotorCount];
    double bridge_time_sec;
    double actual_hz;
    uint64_t cycle_count;
    char status_message[256];
};
#pragma pack(pop)

static_assert(sizeof(MotorBridgeSharedData) == 1200,
              "MotorBridgeSharedData layout must match wheeldog_mcu_bridge.py");

}  // namespace interface::hardware
