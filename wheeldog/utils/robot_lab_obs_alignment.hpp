/**
 * @file robot_lab_obs_alignment.hpp
 * @brief ONNX policy observation dimensions and scales for the wheel-legged dog deploy path.
 *
 * The model has 12 leg joints + 4 wheel joints.
 * Policy joint order is 12 leg position joints first, then 4 wheel velocity joints.
 *
 * Deployment checklist against the RobotLab training cfg:
 * - base_ang_vel.scale = 0.25  <-> kScaleBaseAngVel
 * - joint_vel.scale = 0.05     <-> kScaleJointVel
 * - joint_pos.scale = 1.0      <-> kScaleJointPos
 * - action clip = [-100, 100]  <-> kPolicyActionClip
 * - commands = [-1, 1]         <-> kCommand*Max
 * - sim.dt = 0.005, decimation = 4, policy/control frequency = 50 Hz
 * - wheel joint position       <-> forced to zero in observation; wheels use velocity control
 * - real-wheel velocity frame  <-> sign-flipped at the hardware/URDF boundary in both
 *                                 observation and action; MuJoCo stays identity
 * - hardware wheel dq_des      <-> RS01 virtual motion target (+/-44 rad/s);
 *                                 MCU separately enforces the 17 Nm torque ceiling
 * - observation order must match the policy network:
 *   omega, projected_gravity, cmd_vel, joint_pos, joint_vel, last_action
 */

#pragma once

#include "robot_model_config.hpp"

namespace robot_lab_obs {

// Matches MydogTestPolicyRunnerONNX::current_obs_ concatenation order: 57 dimensions.
inline constexpr int kDimBaseOmega = 3;
inline constexpr int kDimProjectedGravity = 3;
inline constexpr int kDimCmdVel = 3;
inline constexpr int kDimJointPos = robot_model::kTotalDof;
inline constexpr int kDimJointVel = robot_model::kTotalDof;
inline constexpr int kDimLastAction = robot_model::kTotalDof;
inline constexpr int kObsTotal = kDimBaseOmega + kDimProjectedGravity + kDimCmdVel +
                                 kDimJointPos + kDimJointVel + kDimLastAction;
inline constexpr int kSplitLegActionDof = robot_model::kSplitLegActionDof;

// Keep these values synchronized with the training config used to export the ONNX policy.
inline constexpr float kScaleBaseAngVel = robot_model::kOmegaScale;
inline constexpr float kScaleJointVel = robot_model::kJointVelScale;
inline constexpr float kScaleJointPos = 1.0f;
inline constexpr float kScaleProjectedGravityExpected = robot_model::kProjectedGravityScale;
inline constexpr float kPolicyActionClip = robot_model::kPolicyActionClip;
inline constexpr int kPolicyDecimation = robot_model::kPolicyDecimation;
inline constexpr float kCommandLinVelXMax = robot_model::kCommandLinVelXMax;
inline constexpr float kCommandLinVelYMax = robot_model::kCommandLinVelYMax;
inline constexpr float kCommandAngVelZMax = robot_model::kCommandAngVelZMax;

}  // namespace robot_lab_obs
