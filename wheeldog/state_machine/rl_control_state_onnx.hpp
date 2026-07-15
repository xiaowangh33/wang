/**
 * @file rl_control_state_onnx.hpp
 * @brief rl policy runnning state using onnx
 * @author Bo (Percy) Peng
 * @version 1.0
 * @date 2025-08-10
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */




#ifndef RL_CONTROL_STATE_ONNX_HPP_
#define RL_CONTROL_STATE_ONNX_HPP_

#include "state_base.h"
#include "policy_runner_base.hpp"
#include "Mydog_test_policy_runner_onnx.hpp"
#include "quardLeg_defines.h"
#include "robot_model_config.hpp"
#include <iomanip>
#include <cstdlib>  // for getenv
#include <mutex>
#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <sstream>



class RLControlStateONNX : public StateBase
{
private:
    RobotBasicState rbs_;
    /** 主线程 UpdateRobotObservation 写、PolicyRunner 读，必须同锁避免数据竞争 */
    std::mutex rbs_mutex_;
    std::atomic<int> state_run_cnt_{-1};

    std::shared_ptr<PolicyRunnerBase> policy_ptr_;
    std::shared_ptr<MydogTestPolicyRunnerONNX> test_policy_;


    
    std::thread run_policy_thread_;
    std::atomic<bool> start_flag_{false};

    std::atomic<float> policy_cost_time_{1.0f};
    Vec3f smoothed_cmd_vel_ = Vec3f::Zero();
    bool cmd_vel_initialized_ = false;
    std::chrono::steady_clock::time_point last_cmd_slew_update_;

    // 站立→RL 目标位置平滑过渡：首帧从当前实际关节角出发逐步过渡到策略目标
    int rl_transition_cnt_ = 0;
    static constexpr int kRlTransitionSteps = 100;  // 策略步数（~2s @50Hz）
    VecXf transition_start_pos_;  // 正常从 StandUp 切入时使用其目标，而非实际角度
    bool transition_start_set_ = false;
    bool transition_from_standup_goal_ = false;

    // 策略目标位置低通滤波 (EMA): goal_filtered = alpha*raw + (1-alpha)*prev
    // alpha=1.0→直通无滤波; alpha=0.2→τ≈5步≈100ms @50Hz; alpha=0→冻结
    // 运行时通过环境变量 RL_GOAL_FILTER_ALPHA 调节，默认 1.0（直通）
    VecXf goal_filtered_;
    bool goal_filter_init_ = false;
    float goal_filter_alpha_ = []() -> float {
        const char* env = std::getenv("RL_GOAL_FILTER_ALPHA");
        if (env) { float v = std::strtof(env, nullptr); if (v >= 0.0f && v <= 1.0f) return v; }
        return 1.0f;  // 默认直通无滤波
    }();

    // Permanently rate-limit commanded leg position targets after the two-second
    // RL entry blend. The real-hardware default shares the 8 rad/s deployment
    // limit; RL_LEG_GOAL_SLEW_RADPS remains available for controlled tuning.
    VecXf last_commanded_leg_goal_;
    bool last_commanded_leg_goal_valid_ = false;
    float leg_goal_slew_rate_radps_ = []() -> float {
        const char* env = std::getenv("RL_LEG_GOAL_SLEW_RADPS");
        if (env) {
            const float value = std::strtof(env, nullptr);
            if (value >= 0.1f && value <= 20.0f) return value;
        }
#ifdef BUILD_SIMULATION
        return 20.0f;
#else
        return robot_model::kHardwareLegVelocityLimitRadps;
#endif
    }();
    // Hardware defaults now match the trained Kd=2 on all 12 leg joints. Keep
    // separate Hip/Knee runtime overrides for controlled, supported A/B tests
    // without changing MuJoCo or rebuilding the deployment binary.
    float hardware_hip_kd_floor_ = []() -> float {
#ifdef BUILD_SIMULATION
        return 0.0f;
#else
        const char* env = std::getenv("RL_HIP_KD");
        if (env) {
            const float value = std::strtof(env, nullptr);
            if (std::isfinite(value) && value >= 0.5f && value <= 10.0f) {
                return value;
            }
        }
        return robot_model::kHardwareRlHipKd;
#endif
    }();

    float hardware_knee_kd_floor_ = []() -> float {
#ifdef BUILD_SIMULATION
        return 0.0f;
#else
        const char* env = std::getenv("RL_KNEE_KD");
        if (env) {
            const float value = std::strtof(env, nullptr);
            if (std::isfinite(value) && value >= 0.5f && value <= 10.0f) {
                return value;
            }
        }
        return robot_model::kHardwareRlKneeKd;
#endif
    }();

    static bool IsHipJoint(int joint_index) {
        const int leg_local_index = joint_index % robot_model::kDofPerWheelLeg;
        return leg_local_index == 0 || leg_local_index == 1;
    }

    static bool IsKneeJoint(int joint_index) {
        return joint_index % robot_model::kDofPerWheelLeg == 2;
    }

    // A single 5 Hz line contains the signals needed to judge damping margin.
    // The older multi-line policy and torque dumps remain opt-in.
    bool stability_monitor_enabled_ = []() {
        const char* value = std::getenv("RL_STABILITY_MONITOR");
        return value == nullptr || !(value[0] == '0' && value[1] == '\0');
    }();
    bool torque_alignment_diag_enabled_ = []() {
        const char* value = std::getenv("RL_TORQUE_ALIGNMENT_DIAG");
        return value && (value[0] == '1' || value[0] == 't' || value[0] == 'T' ||
                         value[0] == 'y' || value[0] == 'Y');
    }();
    static constexpr int kStabilityMonitorIntervalSteps = 10;  // 5 Hz @ 50 Hz

    // Hip closed-loop direction/overshoot diagnostics. These are read-only:
    // they inspect the final command matrix sent to the MCU and the coherent
    // measured state, but never modify either one.
    std::array<float, robot_model::kTotalDof> last_hip_error_{};
    std::array<float, robot_model::kTotalDof> last_hip_dq_{};
    std::array<float, robot_model::kTotalDof> last_hip_q_des_{};
    std::array<int, robot_model::kTotalDof> hip_error_crossings_{};
    std::array<bool, robot_model::kTotalDof> hip_history_valid_{};
    std::array<bool, robot_model::kTotalDof> hip_motion_active_{};
    int hip_diag_counter_ = 0;

    // At the instant a new policy result is prepared, joint_tau still belongs
    // to the command held during the preceding 20 ms policy interval. Retain
    // that command so diagnostics compare like with like instead of comparing
    // new-command PD torque against old-command motor feedback.
    MatXf previous_torque_command_;
    bool previous_torque_command_valid_ = false;
    int torque_alignment_counter_ = 0;

    static const char* HipMotionLabel(float error, float relative_velocity) {
        constexpr float kDeadband = 0.02f;
        const float progress = error * relative_velocity;
        if (progress > kDeadband) return "toward-target";
        if (progress < -kDeadband) return "away-from-target";
        return "near-hold";
    }

    void PrintStabilityDiagnostics(const MatXf& command,
                                   const RobotBasicState& state,
                                   int slew_limited_count) {
        static constexpr std::array<int, 4> kHipX = {0, 4, 8, 12};
        static constexpr std::array<int, 4> kHipY = {1, 5, 9, 13};
        constexpr float kMotionOnsetRadps = 0.5f;
        const float dt = robot_model::kPolicyPeriodSec;

        struct HipSample {
            int index = -1;
            float error = 0.0f;
            float dq = 0.0f;
            float q_des_rate = 0.0f;
            float relative_velocity = 0.0f;
            float ddq = 0.0f;
            float tau_pd = 0.0f;
            float tau_limited = 0.0f;
            float tau_measured = 0.0f;
            float score = -1.0f;
        };

        const auto inspect_group = [&](const std::array<int, 4>& indices) {
            HipSample worst;
            for (const int joint : indices) {
                const float q = state.joint_pos(joint);
                const float dq = state.joint_vel(joint);
                const float q_des = command(joint, 1);
                const float error = q_des - q;
                const bool history = hip_history_valid_[joint];
                const float q_des_rate = history
                    ? (q_des - last_hip_q_des_[joint]) / dt
                    : 0.0f;
                const float relative_velocity = dq - q_des_rate;
                const float ddq = history
                    ? (dq - last_hip_dq_[joint]) / dt
                    : 0.0f;
                const float tau_pd = command(joint, 0) * error
                                   - command(joint, 2) * dq
                                   + command(joint, 4);
                const float tau_limited = std::clamp(
                    tau_pd,
                    -robot_model::kHardwareBringupTorqueLimitNm,
                    robot_model::kHardwareBringupTorqueLimitNm);

                // A target-error sign crossing is an overshoot candidate. Use
                // relative joint/target speed so a moving policy target is not
                // mistaken for the plant crossing a stationary target.
                if (history && last_hip_error_[joint] * error < 0.0f &&
                    std::abs(relative_velocity) > 0.3f &&
                    std::abs(dq) > std::abs(q_des_rate) + 0.1f) {
                    ++hip_error_crossings_[joint];
                    const int crossings = hip_error_crossings_[joint];
                    if (crossings <= 2 || crossings % 10 == 0) {
                        std::cout << "[HipErrorCross] joint="
                                  << robot_model::kJoints[joint].name
                                  << " count=" << crossings
                                  << " q=" << q << " q_des=" << q_des
                                  << " error=" << error << " dq=" << dq
                                  << " q_des_rate=" << q_des_rate
                                  << " tau_new=" << tau_pd
                                  << " kd=" << command(joint, 2)
                                  << std::endl;
                    }
                }

                if (!hip_motion_active_[joint] &&
                    std::abs(dq) >= kMotionOnsetRadps) {
                    hip_motion_active_[joint] = true;
                    std::cout << "[HipMotionOnset] joint="
                              << robot_model::kJoints[joint].name
                              << " q=" << q << " q_des=" << q_des
                              << " error=" << error << " dq=" << dq
                              << " motion="
                              << HipMotionLabel(error, relative_velocity)
                              << " tau_new=" << tau_pd
                              << " tau_fb_prev=" << state.joint_tau(joint)
                              << " kp=" << command(joint, 0)
                              << " kd=" << command(joint, 2)
                              << std::endl;
                }

                const bool moving_away =
                    error * relative_velocity < -0.02f;
                const float score = 2.0f * std::abs(error)
                                  + 0.2f * std::abs(relative_velocity)
                                  + (moving_away ? 2.0f : 0.0f);
                if (score > worst.score) {
                    worst = {joint, error, dq, q_des_rate,
                             relative_velocity, ddq, tau_pd, tau_limited,
                             state.joint_tau(joint), score};
                }

                last_hip_error_[joint] = error;
                last_hip_dq_[joint] = dq;
                last_hip_q_des_[joint] = q_des;
                hip_history_valid_[joint] = true;
            }
            return worst;
        };

        const HipSample worst_x = inspect_group(kHipX);
        const HipSample worst_y = inspect_group(kHipY);
        ++hip_diag_counter_;
        if (!stability_monitor_enabled_ ||
            (hip_diag_counter_ % kStabilityMonitorIntervalSteps) != 0) {
            return;
        }

        const float elapsed = hip_diag_counter_ * dt;
        int worst_error_joint = -1;
        int worst_speed_joint = -1;
        int worst_torque_joint = -1;
        float worst_error = -1.0f;
        float worst_speed = -1.0f;
        float worst_torque = 0.0f;
        float worst_torque_utilization = -1.0f;
        float max_wheel_speed = 0.0f;
        int max_wheel_speed_joint = -1;
        int saturated_leg_count = 0;
        int total_hip_crossings = 0;

        for (int joint = 0; joint < robot_model::kTotalDof; ++joint) {
            const auto& config = robot_model::kJoints[joint];
            if (config.mode == robot_model::ActuatorMode::PositionPD) {
                const float error = command(joint, 1) - state.joint_pos(joint);
                const float speed = std::abs(state.joint_vel(joint));
                const float torque = command(joint, 0) * error
                                   + command(joint, 2) *
                                         (command(joint, 3) - state.joint_vel(joint))
                                   + command(joint, 4);
                const float torque_limit = std::min(
                    config.effort_limit_nm,
                    robot_model::kHardwareBringupTorqueLimitNm);
                const float utilization = std::abs(torque) / torque_limit;
                if (std::abs(error) > worst_error) {
                    worst_error = std::abs(error);
                    worst_error_joint = joint;
                }
                if (speed > worst_speed) {
                    worst_speed = speed;
                    worst_speed_joint = joint;
                }
                if (utilization > worst_torque_utilization) {
                    worst_torque_utilization = utilization;
                    worst_torque = torque;
                    worst_torque_joint = joint;
                }
                if (utilization >= 1.0f) {
                    ++saturated_leg_count;
                }
            } else {
                const float wheel_speed = std::abs(state.joint_vel(joint));
                if (wheel_speed > max_wheel_speed) {
                    max_wheel_speed = wheel_speed;
                    max_wheel_speed_joint = joint;
                }
            }
        }
        for (const int joint : kHipX) total_hip_crossings += hip_error_crossings_[joint];
        for (const int joint : kHipY) total_hip_crossings += hip_error_crossings_[joint];

        std::ostringstream line;
        line << std::fixed << std::setprecision(3)
             << "[Stability] t=" << elapsed << "s"
             << " rp_deg=(" << state.base_rpy(0) * 180.0f / M_PI
             << "," << state.base_rpy(1) * 180.0f / M_PI << ")"
             << " omega=(" << state.base_omega(0) << ","
             << state.base_omega(1) << "," << state.base_omega(2) << ")";
        const auto append_hip = [&](const char* label, const HipSample& sample) {
            if (sample.index < 0) return;
            line << " " << label << "="
                 << robot_model::kJoints[sample.index].name
                 << ":e=" << sample.error
                 << ",dq=" << sample.dq
                 << ",rel=" << sample.relative_velocity;
        };
        append_hip("hx", worst_x);
        append_hip("hy", worst_y);
        if (worst_error_joint >= 0) {
            line << " max_e=" << robot_model::kJoints[worst_error_joint].name
                 << ":" << worst_error;
        }
        if (worst_speed_joint >= 0) {
            line << " max_dq=" << robot_model::kJoints[worst_speed_joint].name
                 << ":" << worst_speed;
        }
        if (worst_torque_joint >= 0) {
            line << " max_tau=" << robot_model::kJoints[worst_torque_joint].name
                 << ":" << worst_torque
                 << "(" << 100.0f * worst_torque_utilization << "%)";
        }
        line << " sat=" << saturated_leg_count << "/12"
             << " slew=" << slew_limited_count << "/12"
             << " hip_cross=" << total_hip_crossings;
        if (max_wheel_speed_joint >= 0) {
            line << " wheel=" << robot_model::kJoints[max_wheel_speed_joint].name
                 << ":dq=" << state.joint_vel(max_wheel_speed_joint)
                 << ",des=" << command(max_wheel_speed_joint, 3)
                 << ",protocol_max="
                 << robot_model::kHardwareWheelMotionVirtualVelocityLimitRadps;
        }
        line << " infer_ms=" << policy_cost_time_.load();
        std::cout << line.str() << std::endl;
    }

    static float EstimateHeldCommandTorque(const MatXf& command,
                                           const RobotBasicState& state,
                                           int joint) {
        const auto mode = robot_model::kJoints[joint].mode;
        float kp = std::isfinite(command(joint, 0))
                     ? std::max(0.0f, command(joint, 0)) : 0.0f;
        float q_des = std::isfinite(command(joint, 1))
                         ? command(joint, 1) : 0.0f;
        float kd = std::isfinite(command(joint, 2))
                     ? std::max(0.0f, command(joint, 2)) : 0.0f;
        float dq_des = std::isfinite(command(joint, 3))
                          ? command(joint, 3) : 0.0f;
        float tau_ff = std::isfinite(command(joint, 4))
                          ? command(joint, 4) : 0.0f;

        const float velocity_limit =
            mode == robot_model::ActuatorMode::Velocity
                ? robot_model::kHardwareWheelMotionVirtualVelocityLimitRadps
                : robot_model::kHardwareLegVelocityLimitRadps;
        dq_des = std::clamp(dq_des, -velocity_limit, velocity_limit);
        tau_ff = std::clamp(tau_ff,
                            -robot_model::kHardwareBringupTorqueLimitNm,
                            robot_model::kHardwareBringupTorqueLimitNm);
        if (mode == robot_model::ActuatorMode::Velocity) {
            kp = 0.0f;
            q_des = 0.0f;
            tau_ff = 0.0f;
        }

        float torque = kp * (q_des - state.joint_pos(joint))
                     + kd * (dq_des - state.joint_vel(joint))
                     + tau_ff;
        const float torque_limit = std::min(
            robot_model::kJoints[joint].effort_limit_nm,
            robot_model::kHardwareBringupTorqueLimitNm);
        torque = std::clamp(torque, -torque_limit, torque_limit);

        return torque;
    }

    void PrintTorqueAlignmentDiagnostics(const MatXf& command,
                                         const RobotBasicState& state) {
        if (previous_torque_command_valid_ &&
            (torque_alignment_counter_ % 10) == 0) {
            std::array<float, robot_model::kTotalDof> reference{};
            int worst_leg = -1;
            float worst_leg_error = -1.0f;
            for (int joint = 0; joint < robot_model::kTotalDof; ++joint) {
                reference[joint] = EstimateHeldCommandTorque(
                    previous_torque_command_, state, joint);
                if (robot_model::kJoints[joint].mode !=
                    robot_model::ActuatorMode::PositionPD) {
                    continue;
                }
                const float error = std::abs(
                    reference[joint] - state.joint_tau(joint));
                if (error > worst_leg_error) {
                    worst_leg_error = error;
                    worst_leg = joint;
                }
            }

            std::cout << "[TorqueAlign] held previous policy command vs current feedback";
            if (worst_leg >= 0) {
                std::cout << "; leg_worst="
                          << robot_model::kJoints[worst_leg].name
                          << " ref=" << reference[worst_leg]
                          << " fb=" << state.joint_tau(worst_leg)
                          << " abs_err=" << worst_leg_error;
            }
            std::cout << "; wheel_ref=(";
            for (std::size_t i = 0;
                 i < robot_model::kWheelControlIndices.size(); ++i) {
                if (i != 0) std::cout << ' ';
                std::cout << reference[robot_model::kWheelControlIndices[i]];
            }
            std::cout << "); wheel_fb=(";
            for (std::size_t i = 0;
                 i < robot_model::kWheelControlIndices.size(); ++i) {
                if (i != 0) std::cout << ' ';
                std::cout << state.joint_tau(
                    robot_model::kWheelControlIndices[i]);
            }
            std::cout << "); wheel_dq=(";
            for (std::size_t i = 0;
                 i < robot_model::kWheelControlIndices.size(); ++i) {
                if (i != 0) std::cout << ' ';
                std::cout << state.joint_vel(
                    robot_model::kWheelControlIndices[i]);
            }
            std::cout << ")" << std::endl;
        }

        ++torque_alignment_counter_;
        previous_torque_command_ = command;
        previous_torque_command_valid_ = true;
    }
    
    // 调试相关
    static bool enable_debug_output_;  // 是否启用调试输出（可通过环境变量控制）
    static int debug_print_counter_;   // 调试打印计数器
    static const int DEBUG_PRINT_INTERVAL_ = 50;  // 每50次策略调用打印一次（约每0.5秒）

    void UpdateRobotObservation(){
        std::lock_guard<std::mutex> lock(rbs_mutex_);
        rbs_.base_rpy     = ri_ptr_->GetImuRpy();
        //rbs_.base_rpy = {0,0,0};
        rbs_.base_rot_mat = RpyToRm(rbs_.base_rpy);
        rbs_.projected_gravity = RmToProjectedGravity(rbs_.base_rot_mat);
        rbs_.base_omega   = ri_ptr_->GetImuOmega();
        //rbs_.base_omega={0,0,0};
        rbs_.base_acc     = ri_ptr_->GetImuAcc();
        //rbs_.base_acc = {0,0,0};
        rbs_.joint_pos    = ri_ptr_->GetJointPosition();
        rbs_.joint_vel    = ri_ptr_->GetJointVelocity();
        rbs_.joint_tau    = ri_ptr_->GetJointTorque();

        


        copy_robo_state_to_shared_memory(rbs_, robot_model::kTotalDof);//

        // The keyboard produces a +/-1 step and returns it to zero after its
        // hold timeout. Feed the policy a bounded acceleration/deceleration so
        // that a short key pulse cannot excite a large whole-body transient.
        const auto user_cmd = uc_ptr_->GetUserCommand();
        Vec3f cmd_vel_input(user_cmd.forward_vel_scale,
                            user_cmd.side_vel_scale,
                            user_cmd.turnning_vel_scale);
        for (int axis = 0; axis < 3; ++axis) {
            cmd_vel_input(axis) = std::clamp(cmd_vel_input(axis), -1.0f, 1.0f);
        }

        const auto now = std::chrono::steady_clock::now();
        if (!cmd_vel_initialized_) {
            smoothed_cmd_vel_.setZero();
            cmd_vel_initialized_ = true;
            last_cmd_slew_update_ = now;
        }
        float dt_s = std::chrono::duration<float>(now - last_cmd_slew_update_).count();
        last_cmd_slew_update_ = now;
        if (!std::isfinite(dt_s) || dt_s < 0.0f) {
            dt_s = 0.0f;
        }
        // Do not turn a paused/descheduled high-level loop into a command jump.
        dt_s = std::min(dt_s, 0.02f);
        const Vec3f slew_rate(robot_model::kCommandForwardSlewPerSec,
                              robot_model::kCommandLateralSlewPerSec,
                              robot_model::kCommandYawSlewPerSec);
        for (int axis = 0; axis < 3; ++axis) {
            const float max_delta = slew_rate(axis) * dt_s;
            const float delta = std::clamp(cmd_vel_input(axis) - smoothed_cmd_vel_(axis),
                                           -max_delta,
                                           max_delta);
            smoothed_cmd_vel_(axis) += delta;
        }
        rbs_.cmd_vel_normlized = smoothed_cmd_vel_;
        
    }

    void PrintPolicyDebugInfo(const RobotAction& ra, const RobotBasicState& obs_snap) {
        if (!enable_debug_output_) {
            return;
        }
        
        debug_print_counter_++;
        if (debug_print_counter_ % DEBUG_PRINT_INTERVAL_ != 0) return;
        
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "\n========== RL策略调试信息 ==========\n";
        std::cout << "策略调用次数: " << debug_print_counter_ << "\n";
        std::cout << "推理耗时: " << policy_cost_time_.load() << " ms\n\n";
        
        // 用户命令
        std::cout << "[用户命令]\n";
        std::cout << "  前进速度 (归一化): " << obs_snap.cmd_vel_normlized(0) 
                  << " (实际: " << obs_snap.cmd_vel_normlized(0) * robot_model::kCommandLinVelXMax << " m/s)\n";
        std::cout << "  侧移速度 (归一化): " << obs_snap.cmd_vel_normlized(1) 
                  << " (实际: " << obs_snap.cmd_vel_normlized(1) * robot_model::kCommandLinVelYMax << " m/s)\n";
        std::cout << "  转向速度 (归一化): " << obs_snap.cmd_vel_normlized(2) 
                  << " (实际: " << obs_snap.cmd_vel_normlized(2) * robot_model::kCommandAngVelZMax << " rad/s)\n";
        
        // 命令状态提示
        if (std::abs(obs_snap.cmd_vel_normlized(0)) > 0.01) {
            std::cout << "  ⚠️  有前进/后退命令，策略应该响应\n";
        }
        if (std::abs(obs_snap.cmd_vel_normlized(1)) > 0.01) {
            std::cout << "  ⚠️  有侧移命令，注意稳定性\n";
        }
        if (std::abs(obs_snap.cmd_vel_normlized(2)) > 0.01) {
            std::cout << "  ⚠️  有转向命令\n";
        }
        std::cout << "\n";
        
        // IMU状态
        std::cout << "[IMU状态]\n";
        std::cout << "  RPY (度): [" << obs_snap.base_rpy(0)*180/M_PI << ", " 
                  << obs_snap.base_rpy(1)*180/M_PI << ", " << obs_snap.base_rpy(2)*180/M_PI << "]\n";
        std::cout << "  角速度 (rad/s): [" << obs_snap.base_omega(0) << ", " 
                  << obs_snap.base_omega(1) << ", " << obs_snap.base_omega(2) << "]\n";
        std::cout << "  投影重力: [" << obs_snap.projected_gravity(0) << ", " 
                  << obs_snap.projected_gravity(1) << ", " << obs_snap.projected_gravity(2) << "]\n\n";
        
        // 策略输出（目标位置）
        std::cout << "[策略输出 - 目标位置 (rad)]\n";
        const int leg_joint_indices_flat[] = {
            0, 1, 2,
            4, 5, 6,
            8, 9, 10,
            12, 13, 14,
        };
        const char* joint_names[] = {
            "FL_HipX", "FL_HipY", "FL_Knee",
            "FR_HipX", "FR_HipY", "FR_Knee",
            "HL_HipX", "HL_HipY", "HL_Knee",
            "HR_HipX", "HR_HipY", "HR_Knee"
        };
        for (int n = 0; n < 12; n++) {
            const int i = leg_joint_indices_flat[n];
            std::cout << "  " << joint_names[n] << ": " << ra.goal_joint_pos(i);
            if ((n + 1) % 3 == 0) std::cout << "\n";
        }
        std::cout << "  ℹ️  说明：策略输出为 URDF 坐标系（与仿真一致）\n";
        std::cout << "  ℹ️  新标定后 joint_pos_ 和策略输出均在 URDF 坐标系，无需 sign_flip\n\n";
        
        // 当前关节状态
        std::cout << "[当前关节状态]\n";
        std::cout << "位置 (rad) [URDF坐标系]:\n";
        for (int n = 0; n < 12; n++) {
            const int i = leg_joint_indices_flat[n];
            std::cout << "  " << joint_names[n] << ": " << obs_snap.joint_pos(i);
            if ((n + 1) % 3 == 0) std::cout << "\n";
        }
        std::cout << "速度 (rad/s):\n";
        for (int n = 0; n < 12; n++) {
            const int i = leg_joint_indices_flat[n];
            std::cout << "  " << joint_names[n] << ": " << obs_snap.joint_vel(i);
            if ((n + 1) % 3 == 0) std::cout << "\n";
        }
        std::cout << "力矩 (Nm):\n";
        for (int n = 0; n < 12; n++) {
            const int i = leg_joint_indices_flat[n];
            std::cout << "  " << joint_names[n] << ": " << obs_snap.joint_tau(i);
            if ((n + 1) % 3 == 0) std::cout << "\n";
        }
        std::cout << "\n";
        
        // 策略目标与反馈均使用 URDF 坐标系，可直接比较。
        std::cout << "[位置误差 (目标 - 实际, rad)]\n";
        for (int n = 0; n < 12; n++) {
            const int i = leg_joint_indices_flat[n];
            float pos_error = ra.goal_joint_pos(i) - obs_snap.joint_pos(i);
            std::cout << "  " << joint_names[n] << ": " << pos_error;
            if ((n + 1) % 3 == 0) std::cout << "\n";
        }
        std::cout << "\n";
        
        // 控制参数
        std::cout << "[控制参数]\n";
        std::cout << "  Kp: " << ra.kp.transpose() << "\n";
        std::cout << "  Kd: " << ra.kd.transpose() << "\n";
        std::cout << "\n";
        
        // ========== 策略意图分析 ==========
        std::cout << "========== 策略意图分析 ==========\n";
        
        // 命令响应检查
        float forward_cmd = obs_snap.cmd_vel_normlized(0);
        float side_cmd = obs_snap.cmd_vel_normlized(1);
        float turn_cmd = obs_snap.cmd_vel_normlized(2);
        
        if (std::abs(forward_cmd) > 0.01 || std::abs(side_cmd) > 0.01 || std::abs(turn_cmd) > 0.01) {
            std::cout << "[命令响应检查]\n";
            if (std::abs(forward_cmd) > 0.01) {
                std::cout << "  前进命令: " << forward_cmd;
                // 检查HipX关节是否有响应（前进主要影响HipX）
                float hipx_change = std::abs(ra.goal_joint_pos(0) - obs_snap.joint_pos(0)) + 
                                     std::abs(ra.goal_joint_pos(4) - obs_snap.joint_pos(4)) +
                                     std::abs(ra.goal_joint_pos(8) - obs_snap.joint_pos(8)) +
                                     std::abs(ra.goal_joint_pos(12) - obs_snap.joint_pos(12));
                if (hipx_change > 0.1) {
                    std::cout << " ✓ HipX关节有响应\n";
                } else {
                    std::cout << " ✗ HipX关节响应不足（变化: " << hipx_change << "）\n";
                }
            }
            if (std::abs(side_cmd) > 0.01) {
                std::cout << "  侧移命令: " << side_cmd;
                // 检查HipY关节是否有响应（侧移主要影响HipY）
                float hipy_change = std::abs(ra.goal_joint_pos(1) - obs_snap.joint_pos(1)) + 
                                     std::abs(ra.goal_joint_pos(5) - obs_snap.joint_pos(5)) +
                                     std::abs(ra.goal_joint_pos(9) - obs_snap.joint_pos(9)) +
                                     std::abs(ra.goal_joint_pos(13) - obs_snap.joint_pos(13));
                if (hipy_change > 0.1) {
                    std::cout << " ✓ HipY关节有响应\n";
                } else {
                    std::cout << " ✗ HipY关节响应不足（变化: " << hipy_change << "）\n";
                }
            }
            if (std::abs(turn_cmd) > 0.01) {
                std::cout << "  转向命令: " << turn_cmd << "\n";
            }
            std::cout << "\n";
        }
        
        // 1. 整体运动意图（基于用户命令）
        std::cout << "[整体运动意图]\n";
        
        if (std::abs(forward_cmd) < 0.01 && std::abs(side_cmd) < 0.01 && std::abs(turn_cmd) < 0.01) {
            std::cout << "  🐕 策略意图：保持平稳/站立\n";
        } else {
            std::cout << "  🐕 策略意图：";
            if (forward_cmd > 0.01) std::cout << " 前进(" << forward_cmd << ")";
            if (forward_cmd < -0.01) std::cout << " 后退(" << forward_cmd << ")";
            if (side_cmd > 0.01) std::cout << " 右移(" << side_cmd << ")";
            if (side_cmd < -0.01) std::cout << " 左移(" << side_cmd << ")";
            if (turn_cmd > 0.01) std::cout << " 右转(" << turn_cmd << ")";
            if (turn_cmd < -0.01) std::cout << " 左转(" << turn_cmd << ")";
            std::cout << "\n";
        }
        
        // 2. 姿态稳定性（基于RPY角度）
        float roll_deg = obs_snap.base_rpy(0) * 180.0 / M_PI;
        float pitch_deg = obs_snap.base_rpy(1) * 180.0 / M_PI;
        std::cout << "  📊 当前姿态：Roll=" << roll_deg << "°, Pitch=" << pitch_deg << "°\n";
        if (std::abs(roll_deg) < 5.0 && std::abs(pitch_deg) < 5.0) {
            std::cout << "  ✅ 姿态稳定\n";
        } else {
            std::cout << "  ⚠️  姿态需要调整\n";
        }
        std::cout << "\n";
        
        // 3. 各腿运动方向分析（仅按 URDF 位置误差做启发式判断）
        std::cout << "[各腿运动方向分析]\n";
        std::cout << "  说明：基于位置误差判断关节运动方向（上/下，IMU z轴正方向为上）\n";
        std::cout << "  格式：关节名[误差(rad)] → 方向\n\n";
        
        // 腿名称和对应的关节索引
        const char* leg_names[] = {"FL(左前)", "FR(右前)", "HL(左后)", "HR(右后)"};
        int leg_joint_indices[4][3] = {
            {0, 1, 2},   // FL: HipX, HipY, Knee
            {4, 5, 6},   // FR: HipX, HipY, Knee
            {8, 9, 10},  // HL: HipX, HipY, Knee
            {12, 13, 14} // HR: HipX, HipY, Knee
        };
        const char* joint_names_short[] = {"HipX", "HipY", "Knee"};
        
        for (int leg = 0; leg < 4; leg++) {
            std::cout << "  " << leg_names[leg] << "腿:\n";
            
            float leg_vertical_error = 0.0;  // HipY和Knee的垂直方向误差总和
            
            for (int j = 0; j < 3; j++) {
                int joint_idx = leg_joint_indices[leg][j];
                float actual_pos = obs_snap.joint_pos(joint_idx);
                float pos_error = ra.goal_joint_pos(joint_idx) - actual_pos;
                
                // 判断运动方向
                std::string direction = "";
                if (std::abs(pos_error) < 0.01) {
                    direction = "保持";
                } else if (pos_error > 0) {
                    direction = "↑向上";
                } else {
                    direction = "↓向下";
                }
                
                // HipY和Knee主要控制垂直方向
                if (j == 1 || j == 2) {  // HipY或Knee
                    leg_vertical_error += pos_error;
                }
                
                std::cout << "    " << joint_names_short[j] << ": 误差=" 
                          << std::setprecision(3) << pos_error << " rad → " << direction << "\n";
            }
            
            // 腿的整体垂直方向
            std::string leg_direction = "";
            if (std::abs(leg_vertical_error) < 0.02) {
                leg_direction = "保持高度";
            } else if (leg_vertical_error > 0) {
                leg_direction = "↑向上抬起";
            } else {
                leg_direction = "↓向下压低";
            }
            std::cout << "    整体垂直方向: " << leg_direction 
                      << " (HipY+Knee误差=" << std::setprecision(3) << leg_vertical_error << " rad)\n";
            std::cout << "\n";
        }
        
        // 4. 关节运动趋势总结
        std::cout << "[关节运动趋势总结]\n";
        int moving_up = 0, moving_down = 0, keeping = 0;
        for (int n = 0; n < 12; n++) {
            const int i = leg_joint_indices_flat[n];
            float pos_error = ra.goal_joint_pos(i) - obs_snap.joint_pos(i);
            if (std::abs(pos_error) < 0.01) keeping++;
            else if (pos_error > 0) moving_up++;
            else moving_down++;
        }
        std::cout << "  ↑ 向上运动: " << moving_up << " 个关节\n";
        std::cout << "  ↓ 向下运动: " << moving_down << " 个关节\n";
        std::cout << "  ➡ 保持位置: " << keeping << " 个关节\n";
        
        std::cout << "====================================\n\n";
    }
    
    void PolicyRunner(){
        int observed_run_cnt = -1;
        double last_policy_time = -1.0;
        while (start_flag_.load()){
            if (ri_ptr_->IsRuntimeSafetyLatched()) {
                std::cout << "[RL monitor] runtime safety latched; stopping policy "
                             "inference and periodic diagnostics"
                          << std::endl;
                start_flag_.store(false);
                break;
            }
            const int state_run_cnt = state_run_cnt_.load();
            if(state_run_cnt >= 0 && state_run_cnt != observed_run_cnt){
                observed_run_cnt = state_run_cnt;
                const double interface_time = ri_ptr_->GetInterfaceTimeStamp();
                const bool first_policy = last_policy_time < 0.0;
                const bool period_elapsed =
                    (interface_time - last_policy_time) >=
                    static_cast<double>(robot_model::kPolicyPeriodSec) - 1e-6;
                if(!first_policy && !period_elapsed){
                    std::this_thread::sleep_for(std::chrono::microseconds(100));
                    continue;
                }
                last_policy_time = interface_time;

                timespec start_timestamp, end_timestamp;
                clock_gettime(CLOCK_MONOTONIC,&start_timestamp);
                RobotBasicState rbs_snap;
                {
                    std::lock_guard<std::mutex> lock(rbs_mutex_);
                    rbs_snap = rbs_;
                }
                auto ra = policy_ptr_->GetRobotAction(rbs_snap);

                for(int i=0;i<robot_model::kTotalDof;++i){
                    if (robot_model::kJoints[i].mode == robot_model::ActuatorMode::Velocity) {
                        continue;
                    }
                    const bool front_leg = i < 8;
                    ra.kp(i) *= front_leg ? robot_data_all->front_leg_kp_scale
                                           : robot_data_all->back_leg_kp_scale;
                    ra.kd(i) *= front_leg ? robot_data_all->front_leg_kd_scale
                                           : robot_data_all->back_leg_kd_scale;
                }
               
                MatXf res = ra.ConvertToMat();

                // StandUp maintains the loaded robot with a non-zero steady
                // position error. Starting RL from measured q would erase that
                // error and abruptly remove its gravity-support torque. Start
                // from the previous StandUp q_des instead, then blend to policy.
                if (rl_transition_cnt_ < kRlTransitionSteps) {
                    if (!transition_start_set_) {
                        transition_start_pos_ = rbs_snap.joint_pos;
                        transition_start_set_ = true;
                    }
                    const float alpha = static_cast<float>(rl_transition_cnt_ + 1)
                                        / static_cast<float>(kRlTransitionSteps);
                    for (int j = 0; j < robot_model::kTotalDof; ++j) {
                        if (robot_model::kJoints[j].mode == robot_model::ActuatorMode::Velocity) {
                            continue;
                        }
                        res(j, 1) = (1.0f - alpha) * transition_start_pos_(j)
                                  + alpha * res(j, 1);
                        if (transition_from_standup_goal_) {
                            // Preserve the validated absolute StandUp Kd=3 and
                            // reach the training Kd=2 smoothly over the same
                            // two-second transition.
                            const float kd_factor =
                                (1.0f - alpha) * robot_model::kStandLegKdScale + alpha;
                            res(j, 2) *= kd_factor;
                        }
                    }
                    ++rl_transition_cnt_;
                }

                // Apply hardware runtime floors after the entry interpolation.
                // Defaults are training-aligned Kd=2; explicit environment
                // overrides can raise one group during a supported A/B test.
                if (hardware_hip_kd_floor_ > 0.0f ||
                    hardware_knee_kd_floor_ > 0.0f) {
                    for (int j = 0; j < robot_model::kTotalDof; ++j) {
                        if (IsHipJoint(j)) {
                            res(j, 2) = std::max(res(j, 2),
                                                 hardware_hip_kd_floor_);
                        } else if (IsKneeJoint(j)) {
                            res(j, 2) = std::max(res(j, 2),
                                                 hardware_knee_kd_floor_);
                        }
                    }
                }

                // ====== RL 动作目标低通滤波 (EMA) ======
                // goal_filtered = alpha*raw + (1-alpha)*prev，alpha=1.0 时直通
                if (!goal_filter_init_) {
                    goal_filtered_.resize(robot_model::kTotalDof);
                    for (int j = 0; j < robot_model::kTotalDof; ++j) goal_filtered_(j) = res(j, 1);
                    goal_filter_init_ = true;
                } else {
                    const float a = goal_filter_alpha_;
                    for (int j = 0; j < robot_model::kTotalDof; ++j) {
                        if (robot_model::kJoints[j].mode == robot_model::ActuatorMode::Velocity) {
                            continue;
                        }
                        goal_filtered_(j) = a * res(j, 1) + (1.0f - a) * goal_filtered_(j);
                    }
                }
                for (int j = 0; j < robot_model::kTotalDof; ++j) {
                    if (robot_model::kJoints[j].mode == robot_model::ActuatorMode::PositionPD) {
                        res(j, 1) = goal_filtered_(j);
                    }
                }

                // Permanent post-policy target slew limit. This is applied
                // after entry blending and optional EMA, and therefore remains
                // active for the whole RL session.
                if (!last_commanded_leg_goal_valid_) {
                    last_commanded_leg_goal_ = rbs_snap.joint_pos;
                    last_commanded_leg_goal_valid_ = true;
                }
                const float max_goal_step =
                    leg_goal_slew_rate_radps_ * robot_model::kPolicyPeriodSec;
                int slew_limited_count = 0;
                for (int j = 0; j < robot_model::kTotalDof; ++j) {
                    if (robot_model::kJoints[j].mode !=
                        robot_model::ActuatorMode::PositionPD) {
                        continue;
                    }
                    const float requested_delta =
                        res(j, 1) - last_commanded_leg_goal_(j);
                    const float limited_delta = std::clamp(
                        requested_delta, -max_goal_step, max_goal_step);
                    if (limited_delta != requested_delta) {
                        ++slew_limited_count;
                    }
                    last_commanded_leg_goal_(j) += limited_delta;
                    res(j, 1) = last_commanded_leg_goal_(j);
                }

                PrintStabilityDiagnostics(res, rbs_snap, slew_limited_count);
#ifndef BUILD_SIMULATION
                if (torque_alignment_diag_enabled_) {
                    PrintTorqueAlignmentDiagnostics(res, rbs_snap);
                }
#endif

                // 与 SetJointCommand(res) 一致写入 SHM，避免 RoboPro/录包与真机 PD 输入不一致
                // [过渡诊断] 已关闭，需要时取消注释即可

                copy_joint_command_mat_to_shared_memory(res, robot_model::kTotalDof);

                ri_ptr_->SetJointCommand(res);

                // RL_DEBUG_CHAIN=1：核对 SetJointCommand 的矩阵与接口缓存一致（应 max|Δ|≈0）
                {
                    const char* chain = std::getenv("RL_DEBUG_CHAIN");
                    if (chain && chain[0] == '1' && chain[1] == '\0') {
                        static int chain_cnt = 0;
                        if ((chain_cnt++ % 25) == 0) {
                            MatXf jc = ri_ptr_->GetJointCommand();
                            float max_diff = 0.f;
                            const int nrow = std::min(robot_model::kTotalDof, std::min(static_cast<int>(res.rows()),
                                                                  static_cast<int>(jc.rows())));
                            for (int i = 0; i < nrow; ++i) {
                                for (int c = 0; c < 5; ++c) {
                                    max_diff = std::max(max_diff, std::abs(res(i, c) - jc(i, c)));
                                }
                            }
                            std::cout << "[RL链路对齐] res 与 ri_ptr_->GetJointCommand() max|Δ|=" << max_diff
                                      << " (未钳位前矩阵，应≈0)\n";
                        }
                    }
                }
                
                // 调试输出
                PrintPolicyDebugInfo(ra, rbs_snap);
                
                clock_gettime(CLOCK_MONOTONIC,&end_timestamp);
                policy_cost_time_.store(
                    (end_timestamp.tv_sec-start_timestamp.tv_sec)*1e3
                    +(end_timestamp.tv_nsec-start_timestamp.tv_nsec)/1e6);
                // std::cout << "cost_time:  " << policy_cost_time_ << " ms\n";
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

public:
    RLControlStateONNX(const RobotType& robot_type, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_type, state_name, data_ptr){
        rbs_.joint_pos = VecXf::Zero(robot_model::kTotalDof);
        rbs_.joint_vel = VecXf::Zero(robot_model::kTotalDof);
        rbs_.joint_tau = VecXf::Zero(robot_model::kTotalDof);
        
        // The legacy multi-page debug dump is opt-in. Normal deployment uses
        // the compact [Stability] line instead.
        const char* debug_env = std::getenv("RL_DEBUG_OUTPUT");
        if (debug_env && (std::string(debug_env) == "1" || std::string(debug_env) == "true")) {
            enable_debug_output_ = true;
            std::cout << "[RL debug] verbose policy dump enabled" << std::endl;
        } else {
            enable_debug_output_ = false;
        }
        debug_print_counter_ = 0;
        
        test_policy_ = std::make_shared<MydogTestPolicyRunnerONNX>("test_onnx");
        policy_ptr_ = test_policy_;
        if(!policy_ptr_){
            std::cerr << "[ERROR] Failed to initialize ONNX policy runner." << std::endl;
            exit(0);
        }  
        policy_ptr_->DisplayPolicyInfo();
        data_ptr_->policy_warmup_ptr = policy_ptr_;
        }
    ~RLControlStateONNX(){
        start_flag_.store(false);
        if(run_policy_thread_.joinable()){
            run_policy_thread_.join();
        }
    }

    virtual void OnEnter() {
        log_(INFO,"RLControlStateONNX entered" );
        state_run_cnt_.store(-1);
        start_flag_.store(true);
        VecXf current_joint_pos = ri_ptr_->GetJointPosition();
        
        // 调试输出：显示进入RL控制时的关节状态
        std::cout << "\n========== [RL控制进入] 初始关节位置 ==========\n";
        std::cout << "实际位置:\n";
        std::cout << "FL: HipX=" << current_joint_pos(0) << ", HipY=" << current_joint_pos(1)
                  << ", Knee=" << current_joint_pos(2) << ", Wheel=" << current_joint_pos(3) << "\n";
        std::cout << "FR: HipX=" << current_joint_pos(4) << ", HipY=" << current_joint_pos(5)
                  << ", Knee=" << current_joint_pos(6) << ", Wheel=" << current_joint_pos(7) << "\n";
        std::cout << "HL: HipX=" << current_joint_pos(8) << ", HipY=" << current_joint_pos(9)
                  << ", Knee=" << current_joint_pos(10) << ", Wheel=" << current_joint_pos(11) << "\n";
        std::cout << "HR: HipX=" << current_joint_pos(12) << ", HipY=" << current_joint_pos(13)
                  << ", Knee=" << current_joint_pos(14) << ", Wheel=" << current_joint_pos(15) << "\n";
        std::cout << "==========================================\n\n";
        smoothed_cmd_vel_ << 0, 0, 0;  // 切入RL时先站稳，用户用wasd主动控制
        cmd_vel_initialized_ = true;
        last_cmd_slew_update_ = std::chrono::steady_clock::now();

        rl_transition_cnt_ = 0;
        transition_start_pos_ = current_joint_pos;
        transition_from_standup_goal_ = false;
        if (data_ptr_->standup_goal_joint_pos.size() == robot_model::kTotalDof) {
            float worst_leg_error = 0.0f;
            for (int j = 0; j < robot_model::kTotalDof; ++j) {
                if (robot_model::kJoints[j].mode == robot_model::ActuatorMode::PositionPD) {
                    worst_leg_error = std::max(
                        worst_leg_error,
                        std::abs(current_joint_pos(j) -
                                 data_ptr_->standup_goal_joint_pos(j)));
                }
            }
            // This is the same pose bound required by StandUp before it allows
            // RL entry. A direct Idle->RL entry that is far from the stand pose
            // safely falls back to measured q instead.
            if (worst_leg_error <= 0.25f) {
                transition_start_pos_ = data_ptr_->standup_goal_joint_pos;
                transition_from_standup_goal_ = true;
            }
        }
        transition_start_set_ = true;
        goal_filter_init_ = false;
        last_commanded_leg_goal_ = transition_start_pos_;
        last_commanded_leg_goal_valid_ = true;
        hip_diag_counter_ = 0;
        last_hip_error_.fill(0.0f);
        last_hip_dq_.fill(0.0f);
        last_hip_q_des_.fill(0.0f);
        hip_error_crossings_.fill(0);
        hip_history_valid_.fill(false);
        hip_motion_active_.fill(false);
        previous_torque_command_valid_ = false;
        torque_alignment_counter_ = 0;

        std::cout << "[RL safety] permanent leg target slew limit="
                  << leg_goal_slew_rate_radps_ << " rad/s, entry anchor="
                  << (transition_from_standup_goal_ ? "StandUp q_des" : "measured q")
                  << std::endl;
        std::cout << "[RL safety] normalized command slew=(forward="
                  << robot_model::kCommandForwardSlewPerSec << ", lateral="
                  << robot_model::kCommandLateralSlewPerSec << ", yaw="
                  << robot_model::kCommandYawSlewPerSec << ") per second"
                  << std::endl;
#ifdef BUILD_SIMULATION
        std::cout << "[RL damping] simulation training gains: Hip/Knee Kd="
                  << robot_model::kTrainingLegKd
                  << std::endl;
#else
        std::cout << "[RL damping] hardware HipX/HipY Kd floor="
                  << hardware_hip_kd_floor_
                  << ", Knee Kd floor=" << hardware_knee_kd_floor_
                  << std::endl;
        if (torque_alignment_diag_enabled_) {
            std::cout << "[RL torque diagnostics] verbose alignment enabled at 5 Hz"
                      << std::endl;
        }
#endif
        std::cout << "[RL monitor] compact stability report="
                  << (stability_monitor_enabled_ ? "5 Hz" : "off")
                  << ", legacy policy I/O and torque reports are opt-in"
                  << std::endl;

        policy_ptr_->OnEnter();

        run_policy_thread_ = std::thread(std::bind(&RLControlStateONNX::PolicyRunner, this));
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::RLControlMode);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
    };

    virtual void OnExit() { 
        start_flag_.store(false);
        if(run_policy_thread_.joinable()){
            run_policy_thread_.join();
        }
        state_run_cnt_.store(-1);
    }

    virtual void Run() {
        UpdateRobotObservation();
        ds_ptr_->InsertScopeData(0, policy_cost_time_.load());
        state_run_cnt_.fetch_add(1);
    }

    virtual bool LoseControlJudge() {
        if (ri_ptr_->IsRuntimeSafetyLatched()) return true;
        if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping)) return true;
        return false;
    }

    bool PostureUnsafeCheck(){
        // Vec3f rpy = ri_ptr_->GetImuRpy();
        // if(fabs(rpy(0)) > 30./180*M_PI || fabs(rpy(1)) > 45./180*M_PI){
        //     std::cout << "posture value: " << 180./M_PI*rpy.transpose() << std::endl;
        //     return true;
        // }
        return false;
    }

    virtual StateName GetNextStateName() {


        // 支持直接从idle进入RL控制模式（跳过站立模式）
        if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping)) {
            return StateName::kJointDamping;
        }
        // 'x' 键触发: 从 RL 控制回到默认站立姿态
        if(uc_ptr_->GetUserCommand().sitdown_trigger) {
            return StateName::kSitDown;
        }
        return StateName::kRLControl;
    }
};

// 静态成员初始化
bool RLControlStateONNX::enable_debug_output_ = false;
int RLControlStateONNX::debug_print_counter_ = 0;


#endif  // RL_CONTROL_STATE_ONNX_HPP_
