/**
 * @file sitdown_state.hpp
 * @brief Wheeled-dog lie-down/sit-down state for simulation bring-up.
 */
#ifndef SITDOWN_STATE_HPP_
#define SITDOWN_STATE_HPP_

#include "state_base.h"
#include "quardLeg_defines.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

class SitDownState : public StateBase {
private:
    VecXf init_joint_pos_;
    VecXf init_joint_vel_;
    VecXf goal_joint_pos_;
    VecXf kp_;
    VecXf kd_;
    MatXf joint_cmd_;
    RobotBasicState rbs_;
    float time_stamp_record_ = 0.0f;
    float run_time_ = 0.0f;
    float duration_ = 4.0f;
    float last_print_time_ = -1.0f;
    bool completion_reported_ = false;

    static float CubicPos(float x0, float v0, float xf, float vf, float t, float T) {
        if (t >= T) return xf;
        const float a = (vf * T - 2 * xf + v0 * T + 2 * x0) / (T * T * T);
        const float b = (3 * xf - vf * T - 2 * v0 * T - 3 * x0) / (T * T);
        return a * t * t * t + b * t * t + v0 * t + x0;
    }

    static float CubicVel(float x0, float v0, float xf, float vf, float t, float T) {
        if (t >= T) return 0.0f;
        const float a = (vf * T - 2 * xf + v0 * T + 2 * x0) / (T * T * T);
        const float b = (3 * xf - vf * T - 2 * v0 * T - 3 * x0) / (T * T);
        return 3.0f * a * t * t + 2.0f * b * t + v0;
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
    }

public:
    SitDownState(const RobotType& robot_type, const std::string& state_name,
                 std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_type, state_name, data_ptr) {
        goal_joint_pos_ = VecXf::Zero(robot_model::kTotalDof);
        kp_ = VecXf::Zero(robot_model::kTotalDof);
        kd_ = VecXf::Zero(robot_model::kTotalDof);
        joint_cmd_ = MatXf::Zero(robot_model::kTotalDof, 5);

        for (int i = 0; i < robot_model::kTotalDof; ++i) {
            goal_joint_pos_(i) = robot_model::kLieDownPosition[i];
        }

        for (int i = 0; i < robot_model::kTotalDof; ++i) {
            if (robot_model::kJoints[i].mode == robot_model::ActuatorMode::PositionPD) {
                kp_(i) = robot_model::kJoints[i].kp_default;
                kd_(i) = robot_model::kJoints[i].kd_default;
            } else {
                kp_(i) = 0.0f;
                kd_(i) = robot_model::kJoints[i].kd_default;
            }
        }
    }

    void OnEnter() override {
        log_(INFO, "SitDownState entered");
        ReadRobotState();
        init_joint_pos_ = rbs_.joint_pos;
        init_joint_vel_ = rbs_.joint_vel;
        time_stamp_record_ = run_time_;
        last_print_time_ = -1.0f;
        completion_reported_ = false;
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::StandingUp);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
    }

    void OnExit() override {}

    void Run() override {
        ReadRobotState();
        copy_robo_state_to_shared_memory(rbs_, robot_model::kTotalDof);
        const float current_t = run_time_ - time_stamp_record_;
        if (current_t - last_print_time_ >= 1.0f) {
            std::cout << "[SitDown] folding legs... (" << std::fixed << std::setprecision(1)
                      << std::min(current_t, duration_) << "/" << duration_ << "s)"
                      << std::endl;
            last_print_time_ = current_t;
        }

        VecXf planning_joint_pos = rbs_.joint_pos;
        VecXf planning_joint_vel = VecXf::Zero(robot_model::kTotalDof);
        for (int i = 0; i < robot_model::kTotalDof; ++i) {
            if (robot_model::kJoints[i].mode == robot_model::ActuatorMode::Velocity) {
                planning_joint_pos(i) = rbs_.joint_pos(i);
                planning_joint_vel(i) = 0.0f;
                continue;
            }
            planning_joint_pos(i) = CubicPos(init_joint_pos_(i), init_joint_vel_(i),
                                             goal_joint_pos_(i), 0.0f, current_t, duration_);
            planning_joint_vel(i) = CubicVel(init_joint_pos_(i), init_joint_vel_(i),
                                             goal_joint_pos_(i), 0.0f, current_t, duration_);
        }

        joint_cmd_.col(0) = kp_;
        joint_cmd_.col(1) = planning_joint_pos;
        joint_cmd_.col(2) = kd_;
        joint_cmd_.col(3) = planning_joint_vel;
        joint_cmd_.col(4) = VecXf::Zero(robot_model::kTotalDof);
        copy_joint_command_mat_to_shared_memory(joint_cmd_, robot_model::kTotalDof);
        ri_ptr_->SetJointCommand(joint_cmd_);
    }

    bool LoseControlJudge() override {
        return uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping);
    }

    StateName GetNextStateName() override {
        if (run_time_ - time_stamp_record_ <= duration_) {
            return StateName::kSitDown;
        }
        if (!completion_reported_ && rbs_.joint_pos.size() == robot_model::kTotalDof) {
            int worst_index = -1;
            float worst_error = 0.0f;
            for (int i = 0; i < robot_model::kTotalDof; ++i) {
                if (robot_model::kJoints[i].mode !=
                    robot_model::ActuatorMode::PositionPD) {
                    continue;
                }
                const float error = std::abs(rbs_.joint_pos(i) - goal_joint_pos_(i));
                if (error > worst_error) {
                    worst_error = error;
                    worst_index = i;
                }
            }
            if (worst_index >= 0) {
                std::cout << "[SitDown] completed trajectory; worst final error: "
                          << robot_model::kJoints[worst_index].name
                          << " q=" << rbs_.joint_pos(worst_index)
                          << ", q_des=" << goal_joint_pos_(worst_index)
                          << ", error=" << worst_error << " rad" << std::endl;
            }
            completion_reported_ = true;
        }
        return StateName::kIdle;
    }
};

#endif
