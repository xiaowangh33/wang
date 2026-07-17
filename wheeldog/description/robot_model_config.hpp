#pragma once

#include <array>
#include <cstdint>

namespace robot_model {

inline constexpr int kLegDof = 12;
inline constexpr int kWheelDof = 4;
inline constexpr int kTotalDof = kLegDof + kWheelDof;
inline constexpr int kLegJointsPerLeg = 3;
inline constexpr int kDofPerWheelLeg = 4;

// Wheel-legged dog policy/deployment alignment:
// sim.dt = 0.005, env.decimation = 4 -> policy/control step = 0.02 s (50 Hz).
// Control/MuJoCo joint order follows the updated URDF:
//   FL, FR, HL, HR, each as HipX, HipY, Knee, Ankle.
// Policy order keeps the existing split layout:
//   12 leg position joints first, then 4 wheel velocity joints.
inline constexpr float kSimulationDt = 0.005f;
inline constexpr int kPolicyDecimation = 4;
inline constexpr float kPolicyPeriodSec = kSimulationDt * static_cast<float>(kPolicyDecimation);
inline constexpr int kPolicyInferenceHz = 50;
static_assert(kPolicyPeriodSec > 0.019999f && kPolicyPeriodSec < 0.020001f,
              "deployment policy period must remain training-aligned at 20 ms / 50 Hz");
// Hardware transport runs independently of policy inference.  A 200 Hz state
// tick republishes the most recent 50 Hz policy five-tuple four times; it must
// never cause extra ONNX evaluations.
inline constexpr int kHardwareHighLevelTickHz = 200;
inline constexpr int kHardwarePcToMcuSetpointHz = 200;
static_assert(kHardwarePcToMcuSetpointHz % kPolicyInferenceHz == 0,
              "PC-to-MCU transport must be an integer multiple of policy inference");
inline constexpr float kPolicyActionClip = 100.0f;
inline constexpr float kCommandLinVelXMax = 1.0f;
inline constexpr float kCommandLinVelYMax = 1.0f;
inline constexpr float kCommandAngVelZMax = 1.0f;
/* Hardware bring-up limits.
 *
 * Leg dq_des remains a literal joint-speed command and is capped at 8 rad/s.
 * A wheel action is different: in RS01 motion-control mode it is a virtual
 * velocity target used by tau = Kd * (dq_des - dq), not permission for the
 * wheel to reach that speed. Keep the training actuator's 50 rad/s range in
 * simulation and cap the deployed target at +/-20 rad/s. The native RS01 CAN
 * encoder must retain its +/-44 rad/s scaling, while its independent torque
 * limit remains the rated 17 Nm. Real-wheel measured-speed derating/tripping
 * is disabled; the drive's own native protections remain active. */
inline constexpr float kHardwareBringupTorqueLimitNm = 36.0f;
inline constexpr float kHardwareLegVelocityLimitRadps = 8.0f;
inline constexpr float kTrainingWheelVelocityLimitRadps = 50.0f;
inline constexpr float kHardwareWheelMotionVirtualVelocityLimitRadps = 20.0f;
// Current real-machine damping selected by the Kp=50/Kd=3 stability test.
// RobotLab training used Kd=2; keep this deployment value in one place so
// controlled sim-to-real gain offsets remain explicit.
inline constexpr float kTrainingLegKd = 3.0f;

// Hardware-interface <-> training-URDF wheel mapping in policy wheel order
// [FL, FR, HL, HR]. The MCU calibration table already compensates the actual
// left/right motor mounting polarity, so policy deployment must not mirror a
// side again. The 2026-07-14 supported RL tests visually confirmed that the
// additional right-side -1 mapping reversed FR and HR. Apply this identity
// table consistently to velocity observation and action.
inline constexpr std::array<float, kWheelDof>
    kWheelUrdfHardwareVelocitySigns = {1.0f, 1.0f, 1.0f, 1.0f};

// Normalized policy-command slew rates. At the trained +/-1 command range,
// forward/yaw take about 1.67 s and lateral takes 2.0 s from zero to full.
// The same limits apply while returning to zero after keyboard release.
inline constexpr float kCommandForwardSlewPerSec = 0.6f;
inline constexpr float kCommandLateralSlewPerSec = 0.5f;
inline constexpr float kCommandYawSlewPerSec = 0.6f;

enum class ActuatorMode : uint8_t { PositionPD = 0, Velocity = 1 };
enum class SafeModeBehavior : uint8_t { LegDamping = 0, WheelZeroVel = 1 };

struct JointConfig {
    const char* name;
    const char* mjcf_joint_name;
    int urdf_index;
    int control_index;
    int policy_index;
    ActuatorMode mode;
    SafeModeBehavior safe;
    float urdf_lower_rad;
    float urdf_upper_rad;
    float effort_limit_nm;
    float velocity_limit_radps;
    float kp_default;
    float kd_default;
    float action_scale;
    float pos_default_rad;
};

inline constexpr std::array<int, 4> kWheelControlIndices = {3, 7, 11, 15};
inline constexpr std::array<int, 4> kHipYControlIndices = {1, 5, 9, 13};
inline constexpr std::array<int, 4> kKneeControlIndices = {2, 6, 10, 14};

inline constexpr std::array<float, kTotalDof> kPolicyDefaultPosition = {
    0.0f, 0.5f, -1.0f,
    0.0f, 0.5f, -1.0f,
    0.0f, -0.5f, 1.0f,
    0.0f, -0.5f, 1.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
};

inline constexpr std::array<float, kTotalDof> kRobotDefaultPosition = {
    0.0f, 0.5f, -1.0f, 0.0f,
    0.0f, 0.5f, -1.0f, 0.0f,
    0.0f, -0.5f, 1.0f, 0.0f,
    0.0f, -0.5f, 1.0f, 0.0f,
};

inline constexpr std::array<float, kTotalDof> kLieDownPosition = {
    // Absolute URDF targets; MCU independently enforces calibrated raw walls.
    0.0f, 1.45f, -2.00f, 0.0f,
    0.0f, 1.45f, -2.00f, 0.0f,
    0.0f, -1.45f, 2.00f, 0.0f,
    0.0f, -1.45f, 2.00f, 0.0f,
};

struct MotorAbsoluteCalibration {
    int index;
    int can_number;
    int motor_id;
    const char* symbol;
    ActuatorMode mode;
    float direction_sign;
    float raw_zero_rad;
    float raw_hard_lower_rad;
    float raw_hard_upper_rad;
    float joint_lower_rad;
    float joint_upper_rad;
    bool raw_position_limit_enabled;
};

// Single source shared with MCU C. This is the easy-edit motor symbol/sign/
// zero/boundary table requested after mechanical calibration.
inline constexpr std::array<MotorAbsoluteCalibration, kTotalDof>
    kMotorAbsoluteCalibration = {{
#define WD_MOTOR_CALIBRATION_ENTRY(index, can, id, symbol, mode, sign, zero, raw_lo, raw_hi, joint_lo, joint_hi, enabled) \
    {index, can, id, symbol, static_cast<ActuatorMode>(mode), \
     static_cast<float>(sign), static_cast<float>(zero), \
     static_cast<float>(raw_lo), static_cast<float>(raw_hi), \
     static_cast<float>(joint_lo), static_cast<float>(joint_hi), \
     static_cast<bool>(enabled)},
#include "motor_absolute_calibration_table.inc"
#undef WD_MOTOR_CALIBRATION_ENTRY
}};

// Logical deployment model. Joint ranges follow wheel_legged_dog.urdf.
// Default pose, actuator gains, velocity caps, and action scales follow RobotLab
// wheel_legged_dog custom.py / rough_env_cfg.py. Concrete motor backends own all
// drive-native details.
inline constexpr std::array<JointConfig, kTotalDof> kJoints = {{
    {"FL_HipX", "FL_HipX_joint", 0, 0, 0, ActuatorMode::PositionPD, SafeModeBehavior::LegDamping,
     -0.7854f, 0.7854f, 36.0f, 20.0f, 50.0f, kTrainingLegKd, 0.125f, 0.0f},
    {"FL_HipY", "FL_HipY_joint", 1, 1, 1, ActuatorMode::PositionPD, SafeModeBehavior::LegDamping,
     -2.618f, 2.618f, 36.0f, 20.0f, 50.0f, kTrainingLegKd, 0.25f, 0.5f},
    {"FL_Knee", "FL_Knee_joint", 2, 2, 2, ActuatorMode::PositionPD, SafeModeBehavior::LegDamping,
     -2.65f, 2.65f, 36.0f, 20.0f, 50.0f, kTrainingLegKd, 0.25f, -1.0f},
    {"FL_Ankle", "FL_Ankle", 3, 3, 12, ActuatorMode::Velocity, SafeModeBehavior::WheelZeroVel,
     -1000.0f, 1000.0f, 17.0f, kTrainingWheelVelocityLimitRadps, 0.0f, 0.4f, 5.0f, 0.0f},

    {"FR_HipX", "FR_HipX_joint", 4, 4, 3, ActuatorMode::PositionPD, SafeModeBehavior::LegDamping,
     -0.7854f, 0.7854f, 36.0f, 20.0f, 50.0f, kTrainingLegKd, 0.125f, 0.0f},
    {"FR_HipY", "FR_HipY_joint", 5, 5, 4, ActuatorMode::PositionPD, SafeModeBehavior::LegDamping,
     -2.618f, 2.618f, 36.0f, 20.0f, 50.0f, kTrainingLegKd, 0.25f, 0.5f},
    {"FR_Knee", "FR_Knee_joint", 6, 6, 5, ActuatorMode::PositionPD, SafeModeBehavior::LegDamping,
     -2.65f, 2.65f, 36.0f, 20.0f, 50.0f, kTrainingLegKd, 0.25f, -1.0f},
    {"FR_Ankle", "FR_Ankle", 7, 7, 13, ActuatorMode::Velocity, SafeModeBehavior::WheelZeroVel,
     -1000.0f, 1000.0f, 17.0f, kTrainingWheelVelocityLimitRadps, 0.0f, 0.4f, 5.0f, 0.0f},

    {"HL_HipX", "HL_HipX_joint", 8, 8, 6, ActuatorMode::PositionPD, SafeModeBehavior::LegDamping,
     -0.7854f, 0.7854f, 36.0f, 20.0f, 50.0f, kTrainingLegKd, 0.125f, 0.0f},
    {"HL_HipY", "HL_HipY_joint", 9, 9, 7, ActuatorMode::PositionPD, SafeModeBehavior::LegDamping,
     -2.618f, 2.618f, 36.0f, 20.0f, 50.0f, kTrainingLegKd, 0.25f, -0.5f},
    {"HL_Knee", "HL_Knee_joint", 10, 10, 8, ActuatorMode::PositionPD, SafeModeBehavior::LegDamping,
     -2.65f, 2.65f, 36.0f, 20.0f, 50.0f, kTrainingLegKd, 0.25f, 1.0f},
    {"HL_Ankle", "HL_Ankle", 11, 11, 14, ActuatorMode::Velocity, SafeModeBehavior::WheelZeroVel,
     -1000.0f, 1000.0f, 17.0f, kTrainingWheelVelocityLimitRadps, 0.0f, 0.4f, 5.0f, 0.0f},

    {"HR_HipX", "HR_HipX_joint", 12, 12, 9, ActuatorMode::PositionPD, SafeModeBehavior::LegDamping,
     -0.7854f, 0.7854f, 36.0f, 20.0f, 50.0f, kTrainingLegKd, 0.125f, 0.0f},
    {"HR_HipY", "HR_HipY_joint", 13, 13, 10, ActuatorMode::PositionPD, SafeModeBehavior::LegDamping,
     -2.618f, 2.618f, 36.0f, 20.0f, 50.0f, kTrainingLegKd, 0.25f, -0.5f},
    {"HR_Knee", "HR_Knee_joint", 14, 14, 11, ActuatorMode::PositionPD, SafeModeBehavior::LegDamping,
     -2.65f, 2.65f, 36.0f, 20.0f, 50.0f, kTrainingLegKd, 0.25f, 1.0f},
    {"HR_Ankle", "HR_Ankle", 15, 15, 15, ActuatorMode::Velocity, SafeModeBehavior::WheelZeroVel,
     -1000.0f, 1000.0f, 17.0f, kTrainingWheelVelocityLimitRadps, 0.0f, 0.4f, 5.0f, 0.0f},
}};

inline constexpr float kOmegaScale = 0.25f;
inline constexpr float kJointVelScale = 0.05f;
inline constexpr float kProjectedGravityScale = 1.0f;
inline constexpr int kSplitLegActionDof = kLegDof;
inline constexpr int kObsTotal = 3 + 3 + 3 + kTotalDof + kTotalDof + kTotalDof;

inline constexpr float kThighLen = 0.22f;
inline constexpr float kShankLen = 0.21975f;
inline constexpr float kHipLen = 0.07f;
inline constexpr float kBodyLenX = 0.372f;
inline constexpr float kBodyLenY = 0.145f;
inline constexpr float kPreHeight = 0.12f;
inline constexpr float kStandHeight = 0.33f;
inline constexpr float kLieDownBaseHeight = 0.16f;
// Keyboard 'z' stand-up trajectory duration. The state machine evaluates this
// independently from the 50 Hz policy loop.
inline constexpr float kStandDuration = 10.0f;
// StandUp applies this scale to the current deployment damping, then RL entry
// interpolates smoothly back to kTrainingLegKd.
inline constexpr float kStandLegKdScale = 1.5f;
// Runtime floors make the real-machine command explicit and remain overrideable
// for controlled A/B tests without changing the 50 Hz policy timing.
inline constexpr float kHardwareRlHipKd = kTrainingLegKd;
inline constexpr float kHardwareRlKneeKd = kTrainingLegKd;
// Editable stand-up wheel-motion parameters:
//   kStandWheelTravelRad: signed travel magnitude per wheel. Front wheels use
//     +travel and rear wheels -travel. Set to 0 for no wheel travel.
//   kStandWheelVelocityLimitRadps: stand-up-only wheel speed ceiling. Set to 0
//     to command exactly zero wheel speed for the entire z trajectory.
// Keep the speed ceiling at or above 1.875*travel/kStandDuration if the full
// quintic travel must complete within the stand-up duration.
inline constexpr float kStandWheelTravelRad = 0.0f;
inline constexpr float kStandWheelVelocityLimitRadps = 0.0f;

}  // namespace robot_model
