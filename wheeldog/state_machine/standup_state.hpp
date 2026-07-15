/**
 * @file standup_state.hpp
 * @brief Wheeled-dog stand-up state. Legs use position PD; wheels smoothly
 * spread the front and rear contact patches, then stop.
 */
#ifndef STANDUP_STATE_HPP_
#define STANDUP_STATE_HPP_

#include "state_base.h"
#include "quardLeg_defines.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

class StandUpState : public StateBase {
private:
    VecXf init_joint_pos_;
    VecXf init_joint_vel_;
    VecXf current_joint_pos_;
    VecXf current_joint_vel_;
    VecXf goal_joint_pos_;
    VecXf kp_;
    VecXf kd_;
    MatXf joint_cmd_;
    RobotBasicState rbs_;

    float time_stamp_record_ = 0.0f;
    float run_time_ = 0.0f;
    float reset_duration_ = robot_model::kStandDuration;
    float last_print_time_ = -1.0f;
    float last_rl_entry_block_print_time_ = -10000.0f;
    float rl_entry_ready_since_ = -1.0f;

    bool HardwareRlEntryReady() {
#ifdef BUILD_SIMULATION
        return true;
#else
        constexpr float kMaxLegPositionErrorRad = 0.25f;
        constexpr float kMaxJointSpeedRadps = 0.50f;
        constexpr float kReadyDwellSec = 1.0f;
        int worst_position_index = -1;
        int worst_velocity_index = -1;
        float worst_position_error = 0.0f;
        float worst_velocity = 0.0f;

        if (current_joint_pos_.size() != robot_model::kTotalDof ||
            current_joint_vel_.size() != robot_model::kTotalDof) {
            return false;
        }
        for (int i = 0; i < robot_model::kTotalDof; ++i) {
            const float q = current_joint_pos_(i);
            const float dq = current_joint_vel_(i);
            if (!std::isfinite(q) || !std::isfinite(dq)) {
                worst_position_index = i;
                worst_position_error = INFINITY;
                break;
            }
            if (robot_model::kJoints[i].mode == robot_model::ActuatorMode::PositionPD) {
                const float error = std::abs(q - goal_joint_pos_(i));
                if (error > worst_position_error) {
                    worst_position_error = error;
                    worst_position_index = i;
                }
            }
            const float speed = std::abs(dq);
            if (speed > worst_velocity) {
                worst_velocity = speed;
                worst_velocity_index = i;
            }
        }

        const bool instant_ready =
            worst_position_error <= kMaxLegPositionErrorRad &&
            worst_velocity <= kMaxJointSpeedRadps;
        if (!instant_ready) {
            rl_entry_ready_since_ = -1.0f;
        }
        if (!instant_ready &&
            run_time_ - last_rl_entry_block_print_time_ >= 1.0f) {
            std::cerr << "[StandUp] RL entry blocked: ";
            if (worst_position_error > kMaxLegPositionErrorRad) {
                const float q = current_joint_pos_(worst_position_index);
                const float dq = current_joint_vel_(worst_position_index);
                const float q_des = joint_cmd_(worst_position_index, 1);
                const float dq_des = joint_cmd_(worst_position_index, 3);
                const float kp = joint_cmd_(worst_position_index, 0);
                const float kd = joint_cmd_(worst_position_index, 2);
                const float tau_ff = joint_cmd_(worst_position_index, 4);
                const float expected_pd_tau =
                    kp * (q_des - q) + kd * (dq_des - dq) + tau_ff;
                const float limited_pd_tau = std::clamp(
                    expected_pd_tau,
                    -robot_model::kHardwareBringupTorqueLimitNm,
                    robot_model::kHardwareBringupTorqueLimitNm);
                const float measured_tau =
                    (rbs_.joint_tau.size() == robot_model::kTotalDof)
                        ? rbs_.joint_tau(worst_position_index)
                        : 0.0f;
                std::cerr << robot_model::kJoints[worst_position_index].name
                          << " position error=" << worst_position_error
                          << " rad (limit=" << kMaxLegPositionErrorRad << ")"
                          << ", q=" << q
                          << ", q_des=" << q_des
                          << ", dq=" << dq
                          << ", PD_tau=" << expected_pd_tau
                          << " Nm -> hard-limit " << limited_pd_tau
                          << " Nm, feedback_tau=" << measured_tau << " Nm";
            }
            if (worst_velocity > kMaxJointSpeedRadps) {
                if (worst_position_error > kMaxLegPositionErrorRad) {
                    std::cerr << ", ";
                }
                std::cerr << robot_model::kJoints[worst_velocity_index].name
                          << " speed=" << worst_velocity
                          << " rad/s (limit=" << kMaxJointSpeedRadps << ")";
            }
            std::cerr << "; staying in StandUp" << std::endl;
            last_rl_entry_block_print_time_ = run_time_;
        }
        if (!instant_ready) {
            return false;
        }

        if (rl_entry_ready_since_ < 0.0f) {
            rl_entry_ready_since_ = run_time_;
        }
        const float ready_dwell = run_time_ - rl_entry_ready_since_;
        if (ready_dwell < kReadyDwellSec) {
            if (run_time_ - last_rl_entry_block_print_time_ >= 1.0f) {
                std::cerr << "[StandUp] RL entry waiting: pose/speed currently valid, "
                          << "stable for " << ready_dwell << "/"
                          << kReadyDwellSec << " s" << std::endl;
                last_rl_entry_block_print_time_ = run_time_;
            }
            return false;
        }
        return true;
#endif
    }

    // Fifth-order trajectory with zero acceleration at both ends. Unlike a
    // normalized smoothstep, it preserves the measured entry velocity and
    // therefore avoids a velocity-command jump when StandUp is entered.
    static float QuinticPos(float x0, float v0, float xf, float vf, float t, float T) {
        if (T <= 0.0f || t >= T) return xf;
        t = std::max(0.0f, t);
        const float T2 = T * T;
        const float T3 = T2 * T;
        const float T4 = T3 * T;
        const float T5 = T4 * T;
        const float c3 = (20.0f * (xf - x0) - (8.0f * vf + 12.0f * v0) * T) /
                         (2.0f * T3);
        const float c4 = (30.0f * (x0 - xf) + (14.0f * vf + 16.0f * v0) * T) /
                         (2.0f * T4);
        const float c5 = (12.0f * (xf - x0) - (6.0f * vf + 6.0f * v0) * T) /
                         (2.0f * T5);
        return x0 + v0 * t + c3 * t * t * t + c4 * t * t * t * t +
               c5 * t * t * t * t * t;
    }

    static float QuinticVel(float x0, float v0, float xf, float vf, float t, float T) {
        if (T <= 0.0f || t >= T) return vf;
        t = std::max(0.0f, t);
        const float T2 = T * T;
        const float T3 = T2 * T;
        const float T4 = T3 * T;
        const float T5 = T4 * T;
        const float c3 = (20.0f * (xf - x0) - (8.0f * vf + 12.0f * v0) * T) /
                         (2.0f * T3);
        const float c4 = (30.0f * (x0 - xf) + (14.0f * vf + 16.0f * v0) * T) /
                         (2.0f * T4);
        const float c5 = (12.0f * (xf - x0) - (6.0f * vf + 6.0f * v0) * T) /
                         (2.0f * T5);
        return v0 + 3.0f * c3 * t * t + 4.0f * c4 * t * t * t +
               5.0f * c5 * t * t * t * t;
    }

    void ReadRobotState() {
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
        rbs_.base_rpy = ri_ptr_->GetImuRpy();
        rbs_.base_rot_mat = RpyToRm(rbs_.base_rpy);
        rbs_.projected_gravity = RmToProjectedGravity(rbs_.base_rot_mat);
        rbs_.base_omega = ri_ptr_->GetImuOmega();
        rbs_.base_acc = ri_ptr_->GetImuAcc();
        rbs_.joint_pos = ri_ptr_->GetJointPosition();
        rbs_.joint_vel = ri_ptr_->GetJointVelocity();
        rbs_.joint_tau = ri_ptr_->GetJointTorque();
        rbs_.cmd_vel_normlized = Vec3f::Zero();
        current_joint_pos_ = rbs_.joint_pos;
        current_joint_vel_ = rbs_.joint_vel;
    }

    void ApplyGainScales(VecXf& kp_scaled, VecXf& kd_scaled) const {
        for (int i = 0; i < robot_model::kTotalDof; ++i) {
            if (robot_model::kJoints[i].mode == robot_model::ActuatorMode::Velocity) {
                kp_scaled(i) = 0.0f;
                // A zero wheel target in RS01 motion mode needs the same
                // training-aligned damping as RobotLab; Kd=0 would be a free
                // wheel rather than a flexible zero-speed command.
                kd_scaled(i) = robot_model::kJoints[i].kd_default;
                continue;
            }
            const bool front_leg = i < 8;
            const int joint_in_leg = i % robot_model::kDofPerWheelLeg;
            if (joint_in_leg < robot_model::kLegJointsPerLeg) {
                kp_scaled(i) = robot_data_all->kp_base[joint_in_leg] *
                               (front_leg ? robot_data_all->front_leg_kp_scale
                                          : robot_data_all->back_leg_kp_scale);
                kd_scaled(i) = robot_data_all->kd_base[joint_in_leg] *
                               (front_leg ? robot_data_all->front_leg_kd_scale
                                          : robot_data_all->back_leg_kd_scale) *
                               robot_model::kStandLegKdScale;
            }
        }
    }

public:
    StandUpState(const RobotType& robot_type, const std::string& state_name,
                 std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_type, state_name, data_ptr) {
        goal_joint_pos_ = VecXf::Zero(robot_model::kTotalDof);
        kp_ = VecXf::Zero(robot_model::kTotalDof);
        kd_ = VecXf::Zero(robot_model::kTotalDof);
        joint_cmd_ = MatXf::Zero(robot_model::kTotalDof, 5);

        for (int i = 0; i < robot_model::kTotalDof; ++i) {
            goal_joint_pos_(i) = robot_model::kJoints[i].pos_default_rad;
            kp_(i) = robot_model::kJoints[i].kp_default;
            kd_(i) = robot_model::kJoints[i].kd_default;
        }
        data_ptr_->standup_goal_joint_pos = goal_joint_pos_;
    }

    void OnEnter() override {
        log_(INFO, "StandUpState entered");
        ReadRobotState();
        init_joint_pos_ = current_joint_pos_;
        init_joint_vel_ = current_joint_vel_;
        time_stamp_record_ = run_time_;
        last_print_time_ = -1.0f;
        last_rl_entry_block_print_time_ = -10000.0f;
        rl_entry_ready_since_ = -1.0f;

        StateBase::msfb_.UpdateCurrentState(RobotMotionState::StandingUp);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
    }

    void OnExit() override {}

    void Run() override {
        ReadRobotState();
        copy_robo_state_to_shared_memory(rbs_, robot_model::kTotalDof);

        const float current_t = run_time_ - time_stamp_record_;
        if (current_t - last_print_time_ >= 1.0f) {
            std::cout << "[StandUp] wheeled dog standing... (" << std::fixed << std::setprecision(1)
                      << std::min(current_t, reset_duration_) << "/" << reset_duration_ << "s)"
                      << std::endl;
            last_print_time_ = current_t;
        }

        VecXf planning_joint_pos = current_joint_pos_;
        VecXf planning_joint_vel = VecXf::Zero(robot_model::kTotalDof);
        for (int i = 0; i < robot_model::kTotalDof; ++i) {
            if (robot_model::kJoints[i].mode == robot_model::ActuatorMode::Velocity) {
                planning_joint_pos(i) = current_joint_pos_(i);
                if (robot_model::kStandWheelTravelRad == 0.0f ||
                    robot_model::kStandWheelVelocityLimitRadps <= 0.0f) {
                    planning_joint_vel(i) = 0.0f;
                    continue;
                }
                const bool front_wheel = i < 2 * robot_model::kDofPerWheelLeg;
                const float wheel_travel = front_wheel
                    ? robot_model::kStandWheelTravelRad
                    : -robot_model::kStandWheelTravelRad;
                const float wheel_velocity = QuinticVel(
                    0.0f, init_joint_vel_(i), wheel_travel, 0.0f,
                    current_t, reset_duration_);
                planning_joint_vel(i) = std::clamp(
                    wheel_velocity,
                    -robot_model::kStandWheelVelocityLimitRadps,
                    robot_model::kStandWheelVelocityLimitRadps);
                continue;
            }
            planning_joint_pos(i) = QuinticPos(init_joint_pos_(i), init_joint_vel_(i),
                                               goal_joint_pos_(i), 0.0f, current_t, reset_duration_);
            planning_joint_vel(i) = QuinticVel(init_joint_pos_(i), init_joint_vel_(i),
                                               goal_joint_pos_(i), 0.0f, current_t, reset_duration_);
        }

        VecXf kp_scaled = kp_;
        VecXf kd_scaled = kd_;
        ApplyGainScales(kp_scaled, kd_scaled);

        joint_cmd_.col(0) = kp_scaled;
        joint_cmd_.col(1) = planning_joint_pos;
        joint_cmd_.col(2) = kd_scaled;
        joint_cmd_.col(3) = planning_joint_vel;
        joint_cmd_.col(4) = VecXf::Zero(robot_model::kTotalDof);

        copy_joint_command_mat_to_shared_memory(joint_cmd_, robot_model::kTotalDof);
        ri_ptr_->SetJointCommand(joint_cmd_);
    }

    bool LoseControlJudge() override {
        return uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping);
    }

    StateName GetNextStateName() override {
        if (run_time_ - time_stamp_record_ <= reset_duration_) {
            return StateName::kStandUp;
        }
        if (uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::RLControlMode)) {
            if (!HardwareRlEntryReady()) {
                return StateName::kStandUp;
            }
            return StateName::kRLControl;
        }
        if (uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::PACEChirpCollect)) {
            return StateName::kPACEChirpCollect;
        }
        if (uc_ptr_->GetUserCommand().sitdown_trigger) {
            return StateName::kSitDown;
        }
        return StateName::kStandUp;
    }
};

#endif
