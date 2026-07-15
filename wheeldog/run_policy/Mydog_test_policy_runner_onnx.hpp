/**
 * @file Mydog_test_policy_runner_onnx.hpp
 * @brief ONNX/mock policy adapter for the 16-DoF wheeled-dog controller.
 */
#ifndef Mydog_TEST_POLICY_RUNNER_ONNX_HPP_
#define Mydog_TEST_POLICY_RUNNER_ONNX_HPP_

#include "policy_runner_base.hpp"
#include "../utils/robot_lab_obs_alignment.hpp"
#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace types;

class MydogTestPolicyRunnerONNX : public PolicyRunnerBase {
private:
    std::string model_path_;

    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo memory_info_;

    std::vector<const char*> input_names_{"obs"};
    std::vector<const char*> output_names_{"actions"};

    static constexpr int obs_dim_ = robot_lab_obs::kObsTotal;
    static constexpr int act_dim_ = robot_model::kTotalDof;

    bool mock_policy_ = true;
    bool warned_bad_runtime_shape_ = false;
    VecXf current_obs_ = VecXf::Zero(obs_dim_);
    VecXf joint_pos_rl_ = VecXf::Zero(act_dim_);
    VecXf joint_vel_rl_ = VecXf::Zero(act_dim_);
    VecXf last_action_ = VecXf::Zero(act_dim_);
    VecXf action_ = VecXf::Zero(act_dim_);
    VecXf robot_action_buffer_ = VecXf::Zero(act_dim_);
    VecXf wheel_model_feedback_ = VecXf::Zero(robot_model::kWheelDof);
    VecXf wheel_model_target_ = VecXf::Zero(robot_model::kWheelDof);
    Vec3f gravity_direction_ = Vec3f(0.0f, 0.0f, -1.0f);
    Vec3f last_logged_cmd_vel_ = Vec3f::Zero();
    bool cmd_log_initialized_ = false;
    // The compact stability monitor in RLControlState is enabled by default.
    // Keep the older full observation/wheel dumps opt-in so terminal I/O does
    // not perturb or obscure a real-machine stability test.
    bool verbose_policy_io_ = []() {
        const char* value = std::getenv("RL_VERBOSE_POLICY_IO");
        return value && (value[0] == '1' || value[0] == 't' || value[0] == 'T' ||
                         value[0] == 'y' || value[0] == 'Y');
    }();

    RobotAction ra_;

    static constexpr float WheelInterfaceToUrdfSign(int wheel_index) {
#ifdef BUILD_SIMULATION
        // MuJoCo joint coordinates already are the URDF coordinates.
        (void)wheel_index;
        return 1.0f;
#else
        return (wheel_index >= 0 && wheel_index < robot_model::kWheelDof)
                   ? robot_model::kWheelUrdfHardwareVelocitySigns[wheel_index]
                   : 1.0f;
#endif
    }

    static constexpr float WheelVirtualTargetLimitRadps() {
#ifdef BUILD_SIMULATION
        return robot_model::kTrainingWheelVelocityLimitRadps;
#else
        return robot_model::kHardwareWheelMotionVirtualVelocityLimitRadps;
#endif
    }

    static bool FileExists(const std::string& path) {
        std::ifstream f(path);
        return f.good();
    }

    bool RequireOnnx() const {
        const char* required = std::getenv("WHEELDOG_REQUIRE_ONNX");
        return required && required[0] == '1' && required[1] == '\0';
    }

    static std::string FormatShape(const std::vector<int64_t>& shape) {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < shape.size(); ++i) {
            if (i > 0) oss << ", ";
            oss << shape[i];
        }
        oss << "]";
        return oss.str();
    }

    template <typename ShapeInfo>
    std::vector<int64_t> GetShapeOrEmpty(const ShapeInfo& info) const {
        try {
            return info.GetShape();
        } catch (const std::exception& e) {
            std::cout << "[ONNX INIT] Could not read static tensor shape: "
                      << e.what() << "\n";
            return {};
        }
    }

    bool ShapeLastDimMatches(const std::vector<int64_t>& shape, int expected) const {
        if (shape.empty()) return false;
        const int64_t last_dim = shape.back();
        return last_dim < 0 || last_dim == expected;
    }

    static std::string ResolveModelPath() {
        const char* env_path = std::getenv("WHEELDOG_POLICY_PATH");
        if (env_path && env_path[0] != '\0') {
            return std::string(env_path);
        }

        const std::string root = GetAbsPath();
        const std::vector<std::string> candidates = {
            root + "/policy/ppo/policy.onnx",
            root + "/ppo/policy.onnx",
        };
        for (const auto& path : candidates) {
            if (FileExists(path)) {
                return path;
            }
        }
        return candidates.front();
    }

    void TryLoadOnnx() {
        if (!FileExists(model_path_)) {
            std::cout << "[ONNX INIT] " << model_path_
                      << " not found, using zero-action mock policy for simulation bring-up.\n";
            if (RequireOnnx()) {
                throw std::runtime_error("WHEELDOG_REQUIRE_ONNX=1 but policy.onnx is missing");
            }
            return;
        }

        try {
            session_options_.SetIntraOpNumThreads(1);
            session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
            session_ = std::make_unique<Ort::Session>(env_, model_path_.c_str(), session_options_);

            const auto input_info = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo();
            const auto output_info = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo();
            const auto input_shape = GetShapeOrEmpty(input_info);
            const auto output_shape = GetShapeOrEmpty(output_info);
            const bool input_static_ok = ShapeLastDimMatches(input_shape, obs_dim_);
            const bool output_static_ok = ShapeLastDimMatches(output_shape, act_dim_);
            std::cout << "[ONNX INIT] static shape: input=" << FormatShape(input_shape)
                      << " output=" << FormatShape(output_shape)
                      << " expected obs=" << obs_dim_
                      << " action=" << act_dim_ << "\n";
            if (!input_static_ok || !output_static_ok) {
                std::cout << "[ONNX INIT] Static shape check did not confirm the model; "
                          << "continuing and validating tensor sizes at runtime.\n";
            }
            mock_policy_ = false;
            std::cout << "[ONNX INIT] Model loaded: " << model_path_ << "\n";
        } catch (const std::exception& e) {
            session_.reset();
            mock_policy_ = true;
            std::cout << "[ONNX INIT] Failed to load policy: " << e.what()
                      << ". Using zero-action mock policy.\n";
            if (RequireOnnx()) {
                throw;
            }
        }
    }

    void BuildObservation(const RobotBasicState& ro) {
        const Vec3f base_omega = ro.base_omega * robot_lab_obs::kScaleBaseAngVel;
        const Vec3f projected_gravity = ro.base_rot_mat.inverse() * gravity_direction_;
        const Vec3f cmd_vel = ro.cmd_vel_normlized;

        joint_pos_rl_.setZero();
        joint_vel_rl_.setZero();
        wheel_model_feedback_.setZero();
        int wheel_feedback_index = 0;
        for (int i = 0; i < act_dim_; ++i) {
            const auto& joint = robot_model::kJoints[i];
            const int policy_idx = joint.policy_index;
            if (joint.mode == robot_model::ActuatorMode::Velocity) {
                joint_pos_rl_(policy_idx) = 0.0f;
                const float model_velocity =
                    WheelInterfaceToUrdfSign(wheel_feedback_index) *
                    ro.joint_vel(i);
                joint_vel_rl_(policy_idx) =
                    model_velocity * robot_lab_obs::kScaleJointVel;
                if (wheel_feedback_index < robot_model::kWheelDof) {
                    wheel_model_feedback_(wheel_feedback_index++) =
                        model_velocity;
                }
            } else {
                joint_pos_rl_(policy_idx) = ro.joint_pos(i) - joint.pos_default_rad;
                joint_vel_rl_(policy_idx) =
                    ro.joint_vel(i) * robot_lab_obs::kScaleJointVel;
            }
        }

        current_obs_.setZero();
        current_obs_ << base_omega,
                        projected_gravity,
                        cmd_vel,
                        joint_pos_rl_,
                        joint_vel_rl_,
                        last_action_;

        const bool cmd_changed = !cmd_log_initialized_ ||
                                 (cmd_vel - last_logged_cmd_vel_).cwiseAbs().maxCoeff() > 0.01f;
        if (verbose_policy_io_ && (cmd_changed || run_cnt_ % 50 == 0)) {
            std::cout << "[PolicyObs] obs_dim=" << obs_dim_
                      << " cmd=(" << cmd_vel.transpose() << ")"
                      << " omega_radps=(" << ro.base_omega.transpose() << ")"
                      << " gravity=(" << projected_gravity.transpose() << ")"
                      << std::endl;
            last_logged_cmd_vel_ = cmd_vel;
            cmd_log_initialized_ = true;
        }
    }

    void RunPolicyInferenceUpdateLastAction(const RobotBasicState& ro) {
        BuildObservation(ro);
        action_.setZero();

        if (!mock_policy_ && session_) {
            try {
                std::array<int64_t, 2> input_shape{1, obs_dim_};
                Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                    memory_info_, current_obs_.data(), current_obs_.size(),
                    input_shape.data(), input_shape.size());

                auto output_tensors = session_->Run(Ort::RunOptions{nullptr},
                                                    input_names_.data(), &input_tensor, 1,
                                                    output_names_.data(), 1);
                auto output_info = output_tensors[0].GetTensorTypeAndShapeInfo();
                const size_t output_count = output_info.GetElementCount();
                if (output_count == static_cast<size_t>(act_dim_)) {
                    float* action_data = output_tensors[0].GetTensorMutableData<float>();
                    action_ = Eigen::Map<VecXf>(action_data, act_dim_);
                } else {
                    if (!warned_bad_runtime_shape_) {
                        std::cout << "[ONNX RUN] output element count " << output_count
                                  << " != " << act_dim_ << ", using zeros.\n";
                        warned_bad_runtime_shape_ = true;
                    }
                    if (RequireOnnx()) {
                        throw std::runtime_error("policy.onnx runtime output shape mismatch");
                    }
                }
            } catch (const std::exception& e) {
                if (!warned_bad_runtime_shape_) {
                    std::cout << "[ONNX RUN] inference failed: " << e.what()
                              << ". Using zero-action mock policy.\n";
                    warned_bad_runtime_shape_ = true;
                }
                session_.reset();
                mock_policy_ = true;
                if (RequireOnnx()) {
                    throw;
                }
            }
        }

        for (int i = 0; i < act_dim_; ++i) {
            action_(i) = std::clamp(action_(i),
                                    -robot_model::kPolicyActionClip,
                                    robot_model::kPolicyActionClip);
        }
        last_action_ = action_;
        ++run_cnt_;
    }

public:
    explicit MydogTestPolicyRunnerONNX(std::string policy_name)
        : PolicyRunnerBase(policy_name),
          model_path_(ResolveModelPath()),
          env_(ORT_LOGGING_LEVEL_WARNING, "WheeledDogONNXPolicy"),
          session_options_(),
          session_(nullptr),
          memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {
        ra_.goal_joint_pos = VecXf::Zero(act_dim_);
        ra_.goal_joint_vel = VecXf::Zero(act_dim_);
        ra_.wheel_vel_actions = VecXf::Zero(robot_model::kWheelDof);
        ra_.tau_ff = VecXf::Zero(act_dim_);
        ra_.kp = VecXf::Zero(act_dim_);
        ra_.kd = VecXf::Zero(act_dim_);
        for (int i = 0; i < act_dim_; ++i) {
            ra_.goal_joint_pos(i) = robot_model::kJoints[i].pos_default_rad;
            ra_.kp(i) = robot_model::kJoints[i].kp_default;
            ra_.kd(i) = robot_model::kJoints[i].kd_default;
        }

        TryLoadOnnx();
        decimation_ = robot_model::kPolicyDecimation;
    }

    void DisplayPolicyInfo() override {
        std::cout << "Policy runner: " << policy_name_ << "\n";
        std::cout << "path: " << model_path_ << "\n";
        std::cout << "mode: " << (mock_policy_ ? "mock-zero" : "onnx") << "\n";
        std::cout << "obs_dim: " << obs_dim_ << ", action_dim: " << act_dim_
                  << ", leg_split: " << robot_model::kSplitLegActionDof
                  << ", policy_period_s: " << robot_model::kPolicyPeriodSec
                  << " (legacy decimation=" << decimation_ << ")\n";
        std::cout << "wheel_frame: "
#ifdef BUILD_SIMULATION
                  << "MuJoCo/URDF identity"
#else
                  << "real hardware <-> URDF signs=[FL "
                  << WheelInterfaceToUrdfSign(0) << ", FR "
                  << WheelInterfaceToUrdfSign(1) << ", HL "
                  << WheelInterfaceToUrdfSign(2) << ", HR "
                  << WheelInterfaceToUrdfSign(3) << "]"
#endif
                  << " (applied to both velocity observation and action)\n";
    }

    void OnEnter() override {
        log_(INFO, "MydogTestPolicyRunnerONNX entered");
        run_cnt_ = 0;
        current_obs_.setZero();
        // A fresh RL session is equivalent to an Isaac Lab episode boundary:
        // last_action starts at zero. Without this reset, re-entering RL would
        // feed the previous session's final action into the first observation.
        last_action_.setZero();
        action_.setZero();
        cmd_log_initialized_ = false;
        std::cout << "[PolicyRunner] entered: " << policy_name_ << std::endl;
    }

    void WarmupObservationOnly(const RobotBasicState& ro) override {
        RunPolicyInferenceUpdateLastAction(ro);
    }

    RobotAction GetRobotAction(const RobotBasicState& ro) override {
        RunPolicyInferenceUpdateLastAction(ro);

        robot_action_buffer_.setZero();
        ra_.goal_joint_pos = VecXf::Zero(act_dim_);
        ra_.goal_joint_vel = VecXf::Zero(act_dim_);
        ra_.wheel_vel_actions = VecXf::Zero(robot_model::kWheelDof);
        ra_.tau_ff = VecXf::Zero(act_dim_);
        wheel_model_target_.setZero();

        int wheel_out = 0;
        for (int i = 0; i < act_dim_; ++i) {
            const auto& joint = robot_model::kJoints[i];
            const float raw = action_(joint.policy_index);
            const float scaled = raw * joint.action_scale;
            ra_.kp(i) = joint.kp_default;
            ra_.kd(i) = joint.kd_default;

            if (joint.mode == robot_model::ActuatorMode::Velocity) {
                const float wheel_target_limit = std::min(
                    joint.velocity_limit_radps,
                    WheelVirtualTargetLimitRadps());
                const float model_wheel_vel = std::clamp(
                    scaled,
                    -wheel_target_limit,
                    wheel_target_limit);
                const float hardware_wheel_vel =
                    WheelInterfaceToUrdfSign(wheel_out) * model_wheel_vel;
                ra_.goal_joint_pos(i) = ro.joint_pos(i);
                ra_.goal_joint_vel(i) = hardware_wheel_vel;
                if (wheel_out < robot_model::kWheelDof) {
                    wheel_model_target_(wheel_out) = model_wheel_vel;
                    ra_.wheel_vel_actions(wheel_out++) = hardware_wheel_vel;
                }
            } else {
                const float q_des = std::clamp(joint.pos_default_rad + scaled,
                                               joint.urdf_lower_rad,
                                               joint.urdf_upper_rad);
                ra_.goal_joint_pos(i) = q_des;
                ra_.goal_joint_vel(i) = 0.0f;
            }
        }

        if (verbose_policy_io_ && run_cnt_ % 50 == 0) {
            VecXf hardware_wheel_target = ra_.wheel_vel_actions;
            for (int i = 0; i < hardware_wheel_target.size(); ++i) {
                hardware_wheel_target(i) = std::clamp(
                    hardware_wheel_target(i),
                    -WheelVirtualTargetLimitRadps(),
                    WheelVirtualTargetLimitRadps());
            }
            std::cout << "[PolicyWheel] URDF feedback=("
                      << wheel_model_feedback_.transpose()
                      << "), URDF/policy virtual target=("
                      << wheel_model_target_.transpose()
                      << "), hardware virtual target after frame sign and +/-"
                      << WheelVirtualTargetLimitRadps()
                      << " rad/s protocol clamp=("
                      << hardware_wheel_target.transpose()
#ifdef BUILD_SIMULATION
                      << ")"
#else
                      << "), measured-speed guard=disabled"
#endif
                      << std::endl;
        }
        return ra_;
    }
};

#endif  // Mydog_TEST_POLICY_RUNNER_ONNX_HPP_
