/**
 * @file pace_chirp_collect_state.hpp
 * @brief PACE chirp data collection state — plays chirp trajectory through existing
 *        SetJointCommand hardware chain, records native log for offline conversion
 *        to PACE/fit.py compatible chirp_data.pt.
 *
 * Design: PACE unchanged. Hardware chain unchanged. This state is a special "policy"
 * that outputs q_des; everything downstream (SetJointCommand -> actuator backend)
 * is the same as StandUpState/RLControlState.
 */

#ifndef PACE_CHIRP_COLLECT_STATE_HPP_
#define PACE_CHIRP_COLLECT_STATE_HPP_

#include "state_base.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <chrono>
#include <ctime>
#include <filesystem>
#include "quardLeg_defines.h"
#include "robot_model_config.hpp"

// ==========================================================================
//  Configurable parameters — adjust before real-robot run, not at runtime
// ==========================================================================

// --- chirp trajectory ---
static constexpr float kChirpMinFrequencyHz     = 0.1f;
static constexpr float kChirpMaxFrequencyHz     = 10.0f;
static constexpr float kChirpDurationS          = 20.0f;
static constexpr float kChirpAmplitudeRad       = 0.15f;   // conservative default
static constexpr float kChirpCommandUpdateHz    = 50.0f;   // q_des update rate
static constexpr float kPaceSampleRateHz        = 400.0f;  // for metadata only

// --- phase durations ---
static constexpr float kHoldBeforeRampInS       = 2.0f;
static constexpr float kRampInDurationS         = 2.0f;
static constexpr float kRampOutDurationS        = 2.0f;
static constexpr float kHoldAfterRampOutS       = 1.0f;

// --- HipX self-check ---
static constexpr float kHipxZeroTargetRad       = 0.0f;
static constexpr float kHipxZeroToleranceRad    = 0.20f;
static constexpr float kHipxZeroRampDurationS   = 3.0f;
static constexpr float kHipxZeroMaxTimeoutS     = 10.0f;

// --- safety: kp/kd limits ---
static constexpr float kChirpSafeKpMax          = 220.0f;
static constexpr float kChirpSafeKdMax          = 10.0f;

// --- safety: joint limits ---
static constexpr float kKneeSafetyMarginRad     = 0.10f;   // extra margin from 0.0
static constexpr float kJointSafetyMarginRad    = 0.08f;   // general margin from URDF limits

// ==========================================================================
//  Joint order definitions
// ==========================================================================

// Deployment order: leg-grouped FL→FR→HL→HR, each (HipX, HipY, Knee)
static const char* kDeployJointNames[12] = {
    "FL_HipX", "FL_HipY", "FL_Knee",
    "FR_HipX", "FR_HipY", "FR_Knee",
    "HL_HipX", "HL_HipY", "HL_Knee",
    "HR_HipX", "HR_HipY", "HR_Knee"
};

// PACE / Isaac order: type-grouped (all HipX → all HipY → all Knee)
static const char* kPaceJointNames[12] = {
    "FL_HipX", "FR_HipX", "HL_HipX", "HR_HipX",
    "FL_HipY", "FR_HipY", "HL_HipY", "HR_HipY",
    "FL_Knee", "FR_Knee", "HL_Knee", "HR_Knee"
};

// Pre-computed mapping tables
// deploy_to_pace[di] = pi: deployment index di → PACE/Isaac index pi
// pace_to_deploy[pi] = di: PACE/Isaac index pi → deployment index di
inline void BuildJointOrderMaps(int deploy_to_pace[12], int pace_to_deploy[12]) {
    for (int di = 0; di < 12; ++di) {
        for (int pi = 0; pi < 12; ++pi) {
            if (std::string(kDeployJointNames[di]) == std::string(kPaceJointNames[pi])) {
                deploy_to_pace[di] = pi;
                pace_to_deploy[pi] = di;
                break;
            }
        }
    }
}

inline bool VerifyJointOrderMaps(const int deploy_to_pace[12], const int pace_to_deploy[12]) {
    // Expected: Isaac HipX 0,1,2,3 → Deploy 0,3,6,9
    //           Isaac HipY 4,5,6,7 → Deploy 1,4,7,10
    //           Isaac Knee 8,9,10,11 → Deploy 2,5,8,11
    int expected_p2d[12] = {0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11};
    for (int i = 0; i < 12; ++i) {
        if (pace_to_deploy[i] != expected_p2d[i]) return false;
    }
    // Verify round-trip
    for (int di = 0; di < 12; ++di) {
        if (pace_to_deploy[deploy_to_pace[di]] != di) return false;
    }
    return true;
}

// ==========================================================================
//  Enum for chirp state phases
// ==========================================================================

enum class ChirpPhase {
    Enter,
    HipxSelfCheck,
    TrajectorySafetyCheck,
    HoldBefore,
    RampIn,
    ChirpExcitation,
    RampOut,
    HoldAfter,
    SaveLog,
    Exit,
    Error,
};

static const char* ChirpPhaseName(ChirpPhase p) {
    switch (p) {
        case ChirpPhase::Enter:                  return "Enter";
        case ChirpPhase::HipxSelfCheck:          return "HipxSelfCheck";
        case ChirpPhase::TrajectorySafetyCheck:  return "TrajectorySafetyCheck";
        case ChirpPhase::HoldBefore:             return "HoldBefore";
        case ChirpPhase::RampIn:                 return "RampIn";
        case ChirpPhase::ChirpExcitation:        return "ChirpExcitation";
        case ChirpPhase::RampOut:                return "RampOut";
        case ChirpPhase::HoldAfter:              return "HoldAfter";
        case ChirpPhase::SaveLog:                return "SaveLog";
        case ChirpPhase::Exit:                   return "Exit";
        case ChirpPhase::Error:                  return "Error";
    }
    return "Unknown";
}

// ==========================================================================
//  PACEChirpCollectState
// ==========================================================================

class PACEChirpCollectState : public StateBase {
private:
    // --- joint order mapping ---
    int deploy_to_pace_[12];
    int pace_to_deploy_[12];

    // --- state variables ---
    ChirpPhase phase_;
    double enter_time_;
    double phase_start_time_;
    double last_command_update_time_;
    bool is_command_update_;

    // --- joint data ---
    VecXf current_joint_pos_;
    VecXf current_joint_vel_;
    VecXf current_joint_tau_;
    VecXf init_joint_pos_;

    // --- q_center (deployment order) ---
    VecXf q_center_deploy_;           // hold-phase target center (HipX=0)
    VecXf q_center_pace_;
    VecXf q_chirp_center_deploy_;     // chirp oscillation center (HipX=measured)
    VecXf q_chirp_center_pace_;

    // --- HipX self-check ---
    VecXf hipx_goal_deploy_;
    VecXf hipx_ramp_start_;
    bool hipx_self_check_passed_;
    std::string hipx_check_msg_;

    // --- trajectory safety check ---
    bool trajectory_safety_passed_;
    std::string trajectory_safety_msg_;

    // --- kp/kd ---
    VecXf kp_;
    VecXf kd_;
    bool kp_kd_safe_;
    std::string kp_kd_msg_;
    bool kp_kd_printed_;

    // --- command ---
    MatXf joint_cmd_;

    // --- chirp trajectory (pre-computed, PACE order) ---
    std::vector<VecXf> chirp_trajectory_pace_;   // full 400Hz theoretical chirp
    int chirp_total_steps_;
    int chirp_current_step_;

    // --- ramp-in / ramp-out trajectory storage ---
    VecXf ramp_in_start_deploy_;
    VecXf ramp_out_start_deploy_;

    // --- held target ---
    VecXf held_target_deploy_;
    VecXf held_target_pace_;
    double held_target_age_;

    // --- CSV logger ---
    std::string log_dir_;
    std::string csv_path_;
    std::string metadata_path_;
    std::ofstream csv_file_;
    bool csv_header_written_;

    // --- frequency statistics ---
    std::vector<double> state_tick_intervals_;
    double last_state_tick_time_;

    // --- error handling ---
    bool error_occurred_;
    std::string error_message_;

    // ======================================================================
    //  Utility: cubic spline
    // ======================================================================
    float CubicSplinePos(float x0, float v0, float xf, float vf, float t, float T) const {
        if (t >= T) return xf;
        if (t <= 0) return x0;
        float a = (vf * T - 2.0f * xf + v0 * T + 2.0f * x0) / (T * T * T);
        float b = (3.0f * xf - vf * T - 2.0f * v0 * T - 3.0f * x0) / (T * T);
        float c = v0;
        float d = x0;
        return a * t * t * t + b * t * t + c * t + d;
    }

    // ======================================================================
    //  Utility: linear ramp
    // ======================================================================
    float LinearRamp(float x0, float xf, float t, float T) const {
        if (t >= T) return xf;
        if (t <= 0) return x0;
        return x0 + (xf - x0) * t / T;
    }

    // ======================================================================
    //  Read robot state (same pattern as StandUpState)
    // ======================================================================
    void ReadRobotState() {
        current_joint_pos_ = ri_ptr_->GetJointPosition();
        current_joint_vel_ = ri_ptr_->GetJointVelocity();
        current_joint_tau_ = ri_ptr_->GetJointTorque();
    }

    // ======================================================================
    //  Read kp/kd from shared memory (same scaling as StandUpState)
    // ======================================================================
    void ReadKpKdFromShm() {
        kp_ = VecXf(12);
        kd_ = VecXf(12);

        if (!robot_data_all) {
            kp_.setZero();
            kd_.setZero();
            return;
        }

        // Base kp/kd per joint type (HipX, HipY, Knee) from shared memory
        float kp_base[3] = {
            robot_data_all->kp_base[0],
            robot_data_all->kp_base[1],
            robot_data_all->kp_base[2]
        };
        float kd_base[3] = {
            robot_data_all->kd_base[0],
            robot_data_all->kd_base[1],
            robot_data_all->kd_base[2]
        };

        float front_kp_scale = robot_data_all->front_leg_kp_scale;
        float back_kp_scale  = robot_data_all->back_leg_kp_scale;
        float front_kd_scale = robot_data_all->front_leg_kd_scale;
        float back_kd_scale  = robot_data_all->back_leg_kd_scale;

        // Apply scaling per leg (same logic as StandUpState)
        for (int i = 0; i < 12; ++i) {
            int motor_idx = i % 3;  // 0=HipX, 1=HipY, 2=Knee
            bool is_front = (i < 6);
            float kp_s = is_front ? front_kp_scale : back_kp_scale;
            float kd_s = is_front ? front_kd_scale : back_kd_scale;
            kp_(i) = kp_base[motor_idx] * kp_s;
            kd_(i) = kd_base[motor_idx] * kd_s;
        }
    }

    // ======================================================================
    //  Check kp/kd against safety limits
    // ======================================================================
    bool CheckKpKdSafety() {
        kp_kd_safe_ = true;
        std::ostringstream oss;
        oss << "kp/kd safety check:";
        for (int i = 0; i < 12; ++i) {
            if (kp_(i) > kChirpSafeKpMax) {
                kp_kd_safe_ = false;
                oss << " [" << i << "]" << kDeployJointNames[i]
                    << " kp=" << kp_(i) << " > max=" << kChirpSafeKpMax;
            }
            if (kd_(i) > kChirpSafeKdMax) {
                kp_kd_safe_ = false;
                oss << " [" << i << "]" << kDeployJointNames[i]
                    << " kd=" << kd_(i) << " > max=" << kChirpSafeKdMax;
            }
        }
        if (kp_kd_safe_) {
            oss << " ALL PASSED (max kp=" << kp_.maxCoeff()
                << ", max kd=" << kd_.maxCoeff() << ")";
        }
        kp_kd_msg_ = oss.str();
        return kp_kd_safe_;
    }

    // ======================================================================
    //  Print kp/kd table
    // ======================================================================
    void PrintKpKdTable() {
        std::cout << "\n========== [PACE Chirp] kp/kd Configuration ==========\n";
        std::cout << "Source: shared memory kp_base/kd_base + front/back leg scale\n";
        if (robot_data_all) {
            std::cout << "  kp_base: [" << robot_data_all->kp_base[0]
                      << ", " << robot_data_all->kp_base[1]
                      << ", " << robot_data_all->kp_base[2] << "]\n";
            std::cout << "  kd_base: [" << robot_data_all->kd_base[0]
                      << ", " << robot_data_all->kd_base[1]
                      << ", " << robot_data_all->kd_base[2] << "]\n";
            std::cout << "  front_kp_scale=" << robot_data_all->front_leg_kp_scale
                      << " back_kp_scale=" << robot_data_all->back_leg_kp_scale << "\n";
            std::cout << "  front_kd_scale=" << robot_data_all->front_leg_kd_scale
                      << " back_kd_scale=" << robot_data_all->back_leg_kd_scale << "\n";
        }
        std::cout << "Safety limits: kp_max=" << kChirpSafeKpMax
                  << ", kd_max=" << kChirpSafeKdMax << "\n\n";
        std::cout << std::fixed << std::setprecision(1);
        std::cout << " idx  joint         kp       kd\n";
        std::cout << " ---  ------------  -------  -------\n";
        for (int i = 0; i < 12; ++i) {
            std::cout << "  " << std::setw(2) << i << "  "
                      << std::setw(12) << std::left << kDeployJointNames[i] << std::right
                      << "  " << std::setw(7) << kp_(i)
                      << "  " << std::setw(7) << kd_(i) << "\n";
        }
        std::cout << "============================================================\n\n";
    }

    // ======================================================================
    //  Write command (same pattern as StandUpState)
    // ======================================================================
    void WriteCommand(const VecXf& goal_pos_deploy) {
        joint_cmd_.col(0) = kp_;
        joint_cmd_.col(1) = goal_pos_deploy;
        joint_cmd_.col(2) = kd_;
        joint_cmd_.col(3) = VecXf::Zero(12);  // goal_vel = 0
        joint_cmd_.col(4) = VecXf::Zero(12);  // tau_ff = 0

        // Write shared memory (same pattern as StandUpState)
        for (int i = 0; i < DOF_COUNT; ++i) {
            robot_data_all->joint_cmd[i].kp = kp_(i);
            robot_data_all->joint_cmd[i].kd = kd_(i);
            robot_data_all->joint_cmd[i].goal_joint_pos = goal_pos_deploy(i);
            robot_data_all->joint_cmd[i].goal_joint_vel = 0.0f;
            robot_data_all->joint_cmd[i].torque_feedforward = 0.0f;
        }

        // Send through existing hardware chain
        ri_ptr_->SetJointCommand(joint_cmd_);
    }

    // ======================================================================
    //  Generate chirp trajectory (PACE order, 400Hz theoretical)
    // ======================================================================
    void GenerateChirpTrajectory() {
        int num_steps = static_cast<int>(kChirpDurationS * kPaceSampleRateHz);
        chirp_total_steps_ = num_steps;
        chirp_trajectory_pace_.resize(num_steps);

        float f0 = kChirpMinFrequencyHz;
        float f1 = kChirpMaxFrequencyHz;
        float T  = kChirpDurationS;
        float dt = 1.0f / kPaceSampleRateHz;

        // Compute per-joint amplitude (PACE order) — restrict HipX amplitude
        VecXf amplitude_pace(12);
        for (int pi = 0; pi < 12; ++pi) {
            std::string name(kPaceJointNames[pi]);
            if (name.find("HipX") != std::string::npos) {
                amplitude_pace(pi) = std::min(kChirpAmplitudeRad, 0.10f);  // tighter for HipX
            } else {
                amplitude_pace(pi) = kChirpAmplitudeRad;
            }
        }

        for (int step = 0; step < num_steps; ++step) {
            float t = step * dt;
            float phase = 2.0f * M_PI * (f0 * t + ((f1 - f0) / (2.0f * T)) * t * t);
            float chirp_val = std::sin(phase);

            VecXf pos_pace(12);
            for (int pi = 0; pi < 12; ++pi) {
                pos_pace(pi) = q_chirp_center_pace_(pi) + chirp_val * amplitude_pace(pi);
            }
            chirp_trajectory_pace_[step] = pos_pace;
        }
    }

    // ======================================================================
    //  Check if a trajectory point is within joint limits (deploy order)
    // ======================================================================
    bool IsWithinJointLimits(const VecXf& pos_deploy, std::string& msg) const {
        std::ostringstream oss;
        bool ok = true;
        for (int i = 0; i < 12; ++i) {
            const int control_index = (i / 3) * 4 + (i % 3);
            const auto& joint = robot_model::kJoints[control_index];
            float margin = kJointSafetyMarginRad;
            // Extra margin for Knee (avoid full extension near 0.0)
            if (i % 3 == 2) {
                margin = std::max(margin, kKneeSafetyMarginRad);
            }
            float lo = joint.urdf_lower_rad + margin;
            float hi = joint.urdf_upper_rad - margin;
            if (pos_deploy(i) < lo || pos_deploy(i) > hi) {
                ok = false;
                oss << " [" << i << "]" << kDeployJointNames[i]
                    << "=" << pos_deploy(i) << " limit=[" << lo << "," << hi << "];";
            }
        }
        if (!ok) msg = oss.str();
        return ok;
    }

    // ======================================================================
    //  Full trajectory safety check
    // ======================================================================
    bool RunTrajectorySafetyCheck() {
        trajectory_safety_passed_ = true;
        std::ostringstream oss;

        // Check chirp trajectory: sample every Nth point at chirp_command_update rate
        int stride = static_cast<int>(kPaceSampleRateHz / kChirpCommandUpdateHz);
        int chirp_steps = static_cast<int>(chirp_trajectory_pace_.size());

        // Check hold, ramp, and all chirp command-update points
        // Build list of all "held target" points (deploy order) that will be commanded
        std::vector<VecXf> check_points;

        // Hold before: q_center
        check_points.push_back(q_center_deploy_);

        // Ramp-in: start (q_center) and end (first chirp point)
        if (chirp_steps > 0) {
            VecXf first_chirp_pace = chirp_trajectory_pace_[0];
            VecXf first_chirp_deploy(12);
            for (int pi = 0; pi < 12; ++pi) first_chirp_deploy(pace_to_deploy_[pi]) = first_chirp_pace(pi);
            check_points.push_back(first_chirp_deploy);
        }

        // Chirp: check at command-update rate
        for (int cs = 0; cs < chirp_steps; cs += stride) {
            VecXf cp = chirp_trajectory_pace_[cs];
            VecXf cd(12);
            for (int pi = 0; pi < 12; ++pi) cd(pace_to_deploy_[pi]) = cp(pi);
            check_points.push_back(cd);
        }

        // Ramp-out: last chirp point → q_center
        if (chirp_steps > 0) {
            VecXf last_chirp_pace = chirp_trajectory_pace_[chirp_steps - 1];
            VecXf last_chirp_deploy(12);
            for (int pi = 0; pi < 12; ++pi) last_chirp_deploy(pace_to_deploy_[pi]) = last_chirp_pace(pi);
            // (already added via stride loop if divisible; add explicitly for safety)
            check_points.push_back(last_chirp_deploy);
        }
        check_points.push_back(q_center_deploy_);  // hold after

        // Check all points
        int violations = 0;
        for (size_t ci = 0; ci < check_points.size(); ++ci) {
            std::string pt_msg;
            if (!IsWithinJointLimits(check_points[ci], pt_msg)) {
                ++violations;
                if (violations <= 5) {  // limit output
                    oss << "  point " << ci << ": " << pt_msg << "\n";
                }
            }
        }
        if (violations > 5) oss << "  ... and " << (violations - 5) << " more violations\n";

        if (violations > 0) {
            trajectory_safety_passed_ = false;
            trajectory_safety_msg_ = oss.str();
        } else {
            trajectory_safety_msg_ = "ALL " + std::to_string(check_points.size()) + " points within limits";
        }

        return trajectory_safety_passed_;
    }

    // ======================================================================
    //  Convert deploy → pace
    // ======================================================================
    VecXf DeployToPace(const VecXf& deploy) const {
        VecXf pace(12);
        for (int di = 0; di < 12; ++di) pace(deploy_to_pace_[di]) = deploy(di);
        return pace;
    }

    VecXf PaceToDeploy(const VecXf& pace) const {
        VecXf deploy(12);
        for (int pi = 0; pi < 12; ++pi) deploy(pace_to_deploy_[pi]) = pace(pi);
        return deploy;
    }

    // ======================================================================
    //  CSV logging
    // ======================================================================
    void OpenCsvLog() {
        // Create log directory
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream dir_oss;
        dir_oss << GetAbsPath() << "/../logs/pace_chirp/"
                << std::put_time(&tm, "%Y%m%d_%H%M%S");
        log_dir_ = dir_oss.str();

        std::error_code ec;
        std::filesystem::create_directories(log_dir_, ec);
        if(ec){
            std::cerr << "[PACE] Failed to create log directory " << log_dir_
                      << ": " << ec.message() << std::endl;
            return;
        }

        csv_path_ = log_dir_ + "/native_log.csv";
        metadata_path_ = log_dir_ + "/metadata.json";

        csv_file_.open(csv_path_);
        if(!csv_file_.is_open()){
            std::cerr << "[PACE] Failed to open CSV log " << csv_path_ << std::endl;
        }
        csv_header_written_ = false;
    }

    void WriteCsvHeader() {
        if (!csv_file_.is_open()) return;
        csv_file_ << "timestamp,phase,is_command_update,last_command_update_time,held_target_age";
        // Raw chirp (pace order)
        // des_dof (pace order)
        // dof_pos (pace order, measured)
        // dof_vel (pace order)
        // dof_tau (pace order)
        // kp (deploy order)
        // kd (deploy order)
        // tau_ff (deploy order)
        // des_dof_deploy (deploy order, as actually sent)
        // dof_pos_deploy (deploy order, as measured)
        // q_center (deploy order)
        for (int i = 0; i < 12; ++i)
            csv_file_ << ",raw_chirp_pos_" << i;
        for (int i = 0; i < 12; ++i)
            csv_file_ << ",des_dof_pos_" << i;
        for (int i = 0; i < 12; ++i)
            csv_file_ << ",dof_pos_" << i;
        for (int i = 0; i < 12; ++i)
            csv_file_ << ",dof_vel_" << i;
        for (int i = 0; i < 12; ++i)
            csv_file_ << ",dof_tau_" << i;
        for (int i = 0; i < 12; ++i)
            csv_file_ << ",kp_" << i;
        for (int i = 0; i < 12; ++i)
            csv_file_ << ",kd_" << i;
        for (int i = 0; i < 12; ++i)
            csv_file_ << ",tau_ff_cmd_" << i;
        for (int i = 0; i < 12; ++i)
            csv_file_ << ",des_dof_deploy_" << i;
        for (int i = 0; i < 12; ++i)
            csv_file_ << ",dof_pos_deploy_" << i;
        for (int i = 0; i < 12; ++i)
            csv_file_ << ",q_center_deploy_" << i;
        for (int i = 0; i < 12; ++i)
            csv_file_ << ",q_chirp_center_deploy_" << i;
        csv_file_ << ",error_flag,error_message\n";
        csv_header_written_ = true;
    }

    void WriteCsvRow(double timestamp, const VecXf& raw_chirp_pace,
                     const VecXf& des_pace, const VecXf& dof_pace,
                     const VecXf& dof_vel_pace, const VecXf& dof_tau_pace,
                     const VecXf& des_deploy, const VecXf& dof_deploy) {
        if (!csv_file_.is_open()) return;
        if (!csv_header_written_) WriteCsvHeader();

        csv_file_ << std::fixed << std::setprecision(6);
        csv_file_ << timestamp << ","
                  << ChirpPhaseName(phase_) << ","
                  << (is_command_update_ ? 1 : 0) << ","
                  << last_command_update_time_ << ","
                  << held_target_age_;

        auto write12 = [&](const VecXf& v) {
            for (int i = 0; i < 12; ++i)
                csv_file_ << "," << v(i);
        };

        write12(raw_chirp_pace);
        write12(des_pace);
        write12(dof_pace);
        write12(dof_vel_pace);
        write12(dof_tau_pace);
        write12(kp_);
        write12(kd_);
        write12(VecXf::Zero(12));  // tau_ff_cmd (always zero)
        write12(des_deploy);
        write12(dof_deploy);
        write12(q_center_deploy_);
        write12(q_chirp_center_deploy_);

        csv_file_ << "," << (error_occurred_ ? 1 : 0)
                  << ",\"" << error_message_ << "\"\n";
    }

    void CloseCsvLog() {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

    // ======================================================================
    //  Metadata JSON writer
    // ======================================================================
    void WriteMetadataJson() {
        std::ofstream mf(metadata_path_);
        if (!mf.is_open()) return;

        // Compute frequency stats
        double mean_interval = 0, min_interval = 0, max_interval = 0;
        if (!state_tick_intervals_.empty()) {
            double sum = 0;
            min_interval = state_tick_intervals_[0];
            max_interval = state_tick_intervals_[0];
            for (auto v : state_tick_intervals_) {
                sum += v;
                if (v < min_interval) min_interval = v;
                if (v > max_interval) max_interval = v;
            }
            mean_interval = sum / state_tick_intervals_.size();
        }

        mf << "{\n";
        mf << "  \"robot\": \"mydog\",\n";
        mf << "  \"mode\": \"PACEChirpCollectState\",\n";
        mf << "  \"joint_order_deploy\": [\n";
        for (int i = 0; i < 12; ++i)
            mf << "    \"" << kDeployJointNames[i] << "\"" << (i < 11 ? "," : "") << "\n";
        mf << "  ],\n";
        mf << "  \"joint_order_pace\": [\n";
        for (int i = 0; i < 12; ++i)
            mf << "    \"" << kPaceJointNames[i] << "\"" << (i < 11 ? "," : "") << "\n";
        mf << "  ],\n";
        mf << "  \"deploy_to_pace_map\": [";
        for (int i = 0; i < 12; ++i)
            mf << deploy_to_pace_[i] << (i < 11 ? ", " : "");
        mf << "],\n";
        mf << "  \"low_level_frequency_hz\": 2000,\n";
        mf << "  \"chirp_command_update_frequency_hz\": " << kChirpCommandUpdateHz << ",\n";
        mf << "  \"pace_sample_frequency_hz\": " << kPaceSampleRateHz << ",\n";
        mf << "  \"state_tick_frequency_measured\": {\n";
        mf << "    \"mean_hz\": " << (mean_interval > 0 ? 1.0 / mean_interval : 0) << ",\n";
        mf << "    \"min_hz\": " << (max_interval > 0 ? 1.0 / max_interval : 0) << ",\n";
        mf << "    \"max_hz\": " << (min_interval > 0 ? 1.0 / min_interval : 0) << ",\n";
        mf << "    \"num_samples\": " << state_tick_intervals_.size() << "\n";
        mf << "  },\n";
        mf << "  \"kp_source\": \"shared_memory_kp_base_kd_base_with_front_back_scale\",\n";
        mf << "  \"kd_source\": \"shared_memory_kp_base_kd_base_with_front_back_scale\",\n";
        mf << "  \"kp_values\": [";
        for (int i = 0; i < 12; ++i)
            mf << kp_(i) << (i < 11 ? ", " : "");
        mf << "],\n";
        mf << "  \"kd_values\": [";
        for (int i = 0; i < 12; ++i)
            mf << kd_(i) << (i < 11 ? ", " : "");
        mf << "],\n";
        mf << "  \"chirp_safe_kp_max\": " << kChirpSafeKpMax << ",\n";
        mf << "  \"chirp_safe_kd_max\": " << kChirpSafeKdMax << ",\n";
        mf << "  \"kp_kd_safety_check_passed\": " << (kp_kd_safe_ ? "true" : "false") << ",\n";
        mf << "  \"state_level_tau_ff_enabled\": false,\n";
        mf << "  \"hardware_gravity_compensation_enabled\": false,\n";
        mf << "  \"friction_compensation_enabled\": false,\n";
        mf << "  \"tau_ff_mode\": \"zero\",\n";
        mf << "  \"chirp_params\": {\n";
        mf << "    \"min_frequency_hz\": " << kChirpMinFrequencyHz << ",\n";
        mf << "    \"max_frequency_hz\": " << kChirpMaxFrequencyHz << ",\n";
        mf << "    \"duration_s\": " << kChirpDurationS << ",\n";
        mf << "    \"amplitude_rad\": " << kChirpAmplitudeRad << ",\n";
        mf << "    \"command_update_hz\": " << kChirpCommandUpdateHz << "\n";
        mf << "  },\n";
        mf << "  \"q_center_source\": \"measured_after_hipx_self_check\",\n";
        mf << "  \"q_center_deploy_order\": [";
        for (int i = 0; i < 12; ++i)
            mf << q_center_deploy_(i) << (i < 11 ? ", " : "");
        mf << "],\n";
        mf << "  \"q_center_pace_order\": [";
        for (int i = 0; i < 12; ++i)
            mf << q_center_pace_(i) << (i < 11 ? ", " : "");
        mf << "],\n";
        mf << "  \"q_chirp_center_deploy_order\": [";
        for (int i = 0; i < 12; ++i)
            mf << q_chirp_center_deploy_(i) << (i < 11 ? ", " : "");
        mf << "],\n";
        mf << "  \"q_chirp_center_pace_order\": [";
        for (int i = 0; i < 12; ++i)
            mf << q_chirp_center_pace_(i) << (i < 11 ? ", " : "");
        mf << "],\n";
        mf << "  \"hipx_q_center_mode\": \"hold_phases_target_0_chirp_uses_measured\",\n";
        mf << "  \"hipx_self_check_result\": \"" << hipx_check_msg_ << "\",\n";
        mf << "  \"full_trajectory_safety_check_result\": \"" << trajectory_safety_msg_ << "\",\n";
        mf << "  \"clamp_occurred\": false,\n";
        mf << "  \"target_resampling_method\": \"zero_order_hold\",\n";
        mf << "  \"feedback_resampling_method\": \"linear_interpolation\",\n";
        mf << "  \"error_occurred\": " << (error_occurred_ ? "true" : "false") << ",\n";
        if (error_occurred_) {
            mf << "  \"error_message\": \"" << error_message_ << "\",\n";
        }
        mf << "  \"notes\": \"PACE unchanged; hardware chain unchanged; converted log generated from native log by convert_native_to_pace.py\"\n";
        mf << "}\n";
        mf.close();
    }

    // ======================================================================
    //  Safety: go to error phase
    // ======================================================================
    void EnterError(const std::string& msg) {
        error_occurred_ = true;
        error_message_ = msg;
        phase_ = ChirpPhase::Error;
        std::cerr << "\n[PACE Chirp] ERROR: " << msg << "\n";
        std::cerr << "[PACE Chirp] Entering Error phase — writing log and returning to Idle\n\n";
    }

    void GoToIdle() {
        // Write current kp/kd=0 to disable PD before exiting
        VecXf zero_kp = VecXf::Zero(12);
        VecXf zero_kd = VecXf::Zero(12);
        joint_cmd_.col(0) = zero_kp;
        joint_cmd_.col(1) = current_joint_pos_;
        joint_cmd_.col(2) = zero_kd;
        joint_cmd_.col(3) = VecXf::Zero(12);
        joint_cmd_.col(4) = VecXf::Zero(12);
        for (int i = 0; i < DOF_COUNT; ++i) {
            robot_data_all->joint_cmd[i].kp = 0.0f;
            robot_data_all->joint_cmd[i].kd = 0.0f;
            robot_data_all->joint_cmd[i].goal_joint_pos = current_joint_pos_(i);
            robot_data_all->joint_cmd[i].goal_joint_vel = 0.0f;
            robot_data_all->joint_cmd[i].torque_feedforward = 0.0f;
        }
        ri_ptr_->SetJointCommand(joint_cmd_);
    }

public:
    PACEChirpCollectState(const RobotType& robot_type, const std::string& state_name,
                          std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_type, state_name, data_ptr) {

        BuildJointOrderMaps(deploy_to_pace_, pace_to_deploy_);

        joint_cmd_ = MatXf::Zero(12, 5);
        current_joint_pos_ = VecXf(12);
        current_joint_vel_ = VecXf(12);
        current_joint_tau_ = VecXf(12);
        init_joint_pos_ = VecXf(12);
        q_center_deploy_ = VecXf(12);
        q_center_pace_ = VecXf(12);
        q_chirp_center_deploy_ = VecXf(12);
        q_chirp_center_pace_ = VecXf(12);
        hipx_goal_deploy_ = VecXf(12);
        hipx_ramp_start_ = VecXf(12);
        held_target_deploy_ = VecXf(12);
        held_target_pace_ = VecXf(12);
        ramp_in_start_deploy_ = VecXf(12);
        ramp_out_start_deploy_ = VecXf(12);
        kp_ = VecXf::Zero(12);
        kd_ = VecXf::Zero(12);

        phase_ = ChirpPhase::Enter;
        enter_time_ = 0;
        phase_start_time_ = 0;
        last_command_update_time_ = 0;
        is_command_update_ = false;
        held_target_age_ = 0;
        chirp_total_steps_ = 0;
        chirp_current_step_ = 0;
        hipx_self_check_passed_ = false;
        trajectory_safety_passed_ = false;
        kp_kd_safe_ = false;
        kp_kd_printed_ = false;
        error_occurred_ = false;
        last_state_tick_time_ = 0;
        csv_header_written_ = false;
    }

    ~PACEChirpCollectState() {
        CloseCsvLog();
    }

    // ======================================================================
    //  OnEnter
    // ======================================================================
    virtual void OnEnter() override {
        log_(INFO, "PACEChirpCollectState entered");

        // Verify joint order maps
        if (!VerifyJointOrderMaps(deploy_to_pace_, pace_to_deploy_)) {
            std::cerr << "[PACE Chirp] FATAL: Joint order mapping verification FAILED\n";
            EnterError("Joint order mapping verification failed");
            // Mark for immediate exit — LoseControlJudge won't catch this,
            // so we transition to Error phase and handle in Run
            return;
        }

        // Initialize
        ReadRobotState();
        init_joint_pos_ = current_joint_pos_;

        // Reset state
        phase_ = ChirpPhase::Enter;
        enter_time_ = ri_ptr_->GetInterfaceTimeStamp();
        phase_start_time_ = enter_time_;
        last_command_update_time_ = 0;
        is_command_update_ = false;
        held_target_age_ = 0;
        error_occurred_ = false;
        error_message_.clear();
        kp_kd_printed_ = false;
        hipx_self_check_passed_ = false;
        trajectory_safety_passed_ = false;
        state_tick_intervals_.clear();
        last_state_tick_time_ = 0;

        // Read kp/kd from shared memory
        ReadKpKdFromShm();
        PrintKpKdTable();

        // Check kp/kd safety
        if (!CheckKpKdSafety()) {
            std::cerr << "[PACE Chirp] FATAL: " << kp_kd_msg_ << "\n";
            EnterError("kp/kd exceeds safety limits: " + kp_kd_msg_);
            return;
        }

        // Print joint order mapping table
        std::cout << "\n========== [PACE Chirp] Joint Order Mapping ==========\n";
        std::cout << " deploy_idx  deploy_name       →  pace_idx  pace_name\n";
        std::cout << " ----------  ----------------     --------  ----------------\n";
        for (int di = 0; di < 12; ++di) {
            int pi = deploy_to_pace_[di];
            std::cout << " " << std::setw(9) << di << "  " << std::setw(16) << std::left
                      << kDeployJointNames[di] << std::right
                      << "  →  " << std::setw(7) << pi << "  " << kPaceJointNames[pi] << "\n";
        }
        std::cout << "========================================================\n\n";

        // Open CSV log
        OpenCsvLog();
        std::cout << "[PACE Chirp] Log directory: " << log_dir_ << "\n";

        // Update motion state feedback
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::PACEChirpCollect);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);

        // Record state tick interval baseline
        last_state_tick_time_ = ri_ptr_->GetInterfaceTimeStamp();

        // Seed initial held target = current position
        held_target_deploy_ = current_joint_pos_;
        held_target_pace_ = DeployToPace(held_target_deploy_);

        // Transition to HipX self-check
        phase_ = ChirpPhase::HipxSelfCheck;
        phase_start_time_ = ri_ptr_->GetInterfaceTimeStamp();
        last_command_update_time_ = phase_start_time_;
    }

    // ======================================================================
    //  Run
    // ======================================================================
    virtual void Run() override {
        double run_time = ri_ptr_->GetInterfaceTimeStamp();

        // Track state tick frequency
        if (last_state_tick_time_ > 0) {
            double interval = run_time - last_state_tick_time_;
            if (interval > 0 && interval < 1.0) {
                state_tick_intervals_.push_back(interval);
            }
        }
        last_state_tick_time_ = run_time;

        // Read current state
        ReadRobotState();

        // Write robot state to shared memory
        RobotBasicState rbs;
        rbs.joint_pos = current_joint_pos_;
        rbs.joint_vel = current_joint_vel_;
        rbs.joint_tau = current_joint_tau_;
        rbs.base_rpy = ri_ptr_->GetImuRpy();
        rbs.base_acc = ri_ptr_->GetImuAcc();
        rbs.base_omega = ri_ptr_->GetImuOmega();
        rbs.base_rot_mat = RpyToRm(rbs.base_rpy);
        rbs.projected_gravity = RmToProjectedGravity(rbs.base_rot_mat);
        copy_robo_state_to_shared_memory(rbs, 12);

        // Check for NaN/inf
        for (int i = 0; i < 12; ++i) {
            if (!std::isfinite(current_joint_pos_(i)) ||
                !std::isfinite(current_joint_vel_(i)) ||
                !std::isfinite(current_joint_tau_(i))) {
                EnterError("NaN or Inf detected in joint state at index " + std::to_string(i));
                return;
            }
        }

        // Command update logic: update held target at chirp_command_update rate
        double time_since_last_update = run_time - last_command_update_time_;
        double chirp_update_interval = 1.0 / kChirpCommandUpdateHz;
        is_command_update_ = (time_since_last_update >= chirp_update_interval ||
                              phase_ == ChirpPhase::Enter ||
                              phase_ == ChirpPhase::Error);

        if (is_command_update_) {
            last_command_update_time_ = run_time;
            held_target_age_ = 0;
        } else {
            held_target_age_ = time_since_last_update;
        }

        // Execute current phase
        double phase_elapsed = run_time - phase_start_time_;

        switch (phase_) {
            case ChirpPhase::Enter:
                // Should not reach here — OnEnter transitions to HipxSelfCheck
                RunHipxSelfCheck(run_time, phase_elapsed);
                break;

            case ChirpPhase::HipxSelfCheck:
                RunHipxSelfCheck(run_time, phase_elapsed);
                break;

            case ChirpPhase::TrajectorySafetyCheck:
                RunTrajectorySafetyCheckPhase(run_time, phase_elapsed);
                break;

            case ChirpPhase::HoldBefore:
                RunHoldBefore(run_time, phase_elapsed);
                break;

            case ChirpPhase::RampIn:
                RunRampIn(run_time, phase_elapsed);
                break;

            case ChirpPhase::ChirpExcitation:
                RunChirpExcitation(run_time, phase_elapsed);
                break;

            case ChirpPhase::RampOut:
                RunRampOut(run_time, phase_elapsed);
                break;

            case ChirpPhase::HoldAfter:
                RunHoldAfter(run_time, phase_elapsed);
                break;

            case ChirpPhase::SaveLog:
                RunSaveLog();
                break;

            case ChirpPhase::Error:
                RunErrorPhase();
                break;

            case ChirpPhase::Exit:
                break;
        }

        // Always write the current held target through SetJointCommand
        // (same pattern as StandUpState — write every state tick)
        if (phase_ != ChirpPhase::Error && phase_ != ChirpPhase::Exit && phase_ != ChirpPhase::SaveLog) {
            WriteCommand(held_target_deploy_);
        }

        // Write CSV row every state tick
        VecXf raw_chirp_pace = held_target_pace_;  // in chirp/motion phases, this is the theoretical
        VecXf dof_pos_pace = DeployToPace(current_joint_pos_);
        VecXf dof_vel_pace = DeployToPace(current_joint_vel_);
        VecXf dof_tau_pace = DeployToPace(current_joint_tau_);

        WriteCsvRow(run_time, raw_chirp_pace, held_target_pace_,
                    dof_pos_pace, dof_vel_pace, dof_tau_pace,
                    held_target_deploy_, current_joint_pos_);
    }

    // ======================================================================
    //  Phase: HipX Self-Check
    // ======================================================================
    void RunHipxSelfCheck(double run_time, double phase_elapsed) {
        // HipX indices in deployment order: 0 (FL), 3 (FR), 6 (HL), 9 (HR)
        static const int hipx_indices[4] = {0, 3, 6, 9};

        // Compute HipX goal: ramp from current to 0
        if (phase_elapsed <= kHipxZeroRampDurationS) {
            for (int j = 0; j < 4; ++j) {
                int i = hipx_indices[j];
                held_target_deploy_(i) = LinearRamp(
                    init_joint_pos_(i), kHipxZeroTargetRad,
                    phase_elapsed, kHipxZeroRampDurationS);
            }
            // Hold other joints at initial position
            for (int i = 0; i < 12; ++i) {
                if (i != hipx_indices[0] && i != hipx_indices[1] &&
                    i != hipx_indices[2] && i != hipx_indices[3]) {
                    held_target_deploy_(i) = init_joint_pos_(i);
                }
            }
        } else {
            // Hold HipX at 0
            for (int j = 0; j < 4; ++j) {
                held_target_deploy_(hipx_indices[j]) = kHipxZeroTargetRad;
            }
        }

        held_target_pace_ = DeployToPace(held_target_deploy_);

        // Check if HipX has reached target
        bool all_hipx_ok = true;
        std::ostringstream oss;
        for (int j = 0; j < 4; ++j) {
            int i = hipx_indices[j];
            float err = std::abs(current_joint_pos_(i) - kHipxZeroTargetRad);
            if (err > kHipxZeroToleranceRad) {
                all_hipx_ok = false;
                oss << " " << kDeployJointNames[i] << " err=" << err << "rad;";
            }
        }

        // Check timeout
        if (phase_elapsed > kHipxZeroMaxTimeoutS && !all_hipx_ok) {
            hipx_self_check_passed_ = false;
            hipx_check_msg_ = "TIMEOUT after " + std::to_string(kHipxZeroMaxTimeoutS) +
                              "s: " + oss.str();
            EnterError("HipX self-check failed: " + hipx_check_msg_);
            return;
        }

        // Check success
        if (all_hipx_ok && phase_elapsed >= kHipxZeroRampDurationS) {
            hipx_self_check_passed_ = true;
            hipx_check_msg_ = "PASSED — all 4 HipX within " +
                              std::to_string(kHipxZeroToleranceRad) + " rad of 0.0";

            std::cout << "[PACE Chirp] HipX self-check PASSED\n";
            for (int j = 0; j < 4; ++j) {
                int i = hipx_indices[j];
                std::cout << "  " << kDeployJointNames[i] << " = "
                          << current_joint_pos_(i) << " rad\n";
            }

            // q_center for hold phases (HoldBefore/RampOut/HoldAfter): HipX=0
            q_center_deploy_ = current_joint_pos_;
            for (int j = 0; j < 4; ++j)
                q_center_deploy_(hipx_indices[j]) = 0.0f;
            q_center_pace_ = DeployToPace(q_center_deploy_);

            // q_chirp_center for chirp oscillation: HipX uses actual measured position
            q_chirp_center_deploy_ = current_joint_pos_;
            q_chirp_center_pace_ = DeployToPace(q_chirp_center_deploy_);

            std::cout << "[PACE Chirp] q_center (hold phases, HipX=0) [deploy order]:\n";
            for (int i = 0; i < 12; ++i) {
                std::cout << "  [" << i << "] " << kDeployJointNames[i]
                          << " = " << q_center_deploy_(i) << " rad\n";
            }
            std::cout << "[PACE Chirp] q_chirp_center (chirp center, HipX=measured) [deploy order]:\n";
            for (int i = 0; i < 12; ++i) {
                std::cout << "  [" << i << "] " << kDeployJointNames[i]
                          << " = " << q_chirp_center_deploy_(i) << " rad\n";
            }

            // Generate chirp trajectory
            GenerateChirpTrajectory();

            // Transition to trajectory safety check
            phase_ = ChirpPhase::TrajectorySafetyCheck;
            phase_start_time_ = run_time;
        }

        // Print progress
        static double last_hipx_print = -1;
        if (run_time - last_hipx_print >= 1.0) {
            std::cout << "[PACE Chirp] HipX self-check: t=" << phase_elapsed
                      << "s";
            for (int j = 0; j < 4; ++j) {
                int i = hipx_indices[j];
                std::cout << " " << kDeployJointNames[i] << "="
                          << current_joint_pos_(i);
            }
            std::cout << "\n";
            last_hipx_print = run_time;
        }
    }

    // ======================================================================
    //  Phase: Trajectory Safety Check
    // ======================================================================
    void RunTrajectorySafetyCheckPhase(double run_time, double) {
        if (!RunTrajectorySafetyCheck()) {
            EnterError("Trajectory safety check FAILED:\n" + trajectory_safety_msg_);
            return;
        }

        std::cout << "[PACE Chirp] Trajectory safety check PASSED — "
                  << trajectory_safety_msg_ << "\n";

        // Transition to HoldBefore
        ramp_in_start_deploy_ = held_target_deploy_;
        phase_ = ChirpPhase::HoldBefore;
        phase_start_time_ = run_time;
    }

    // ======================================================================
    //  Phase: Hold Before
    // ======================================================================
    void RunHoldBefore(double run_time, double phase_elapsed) {
        // Hold q_center
        held_target_deploy_ = q_center_deploy_;
        held_target_pace_ = q_center_pace_;

        if (phase_elapsed >= kHoldBeforeRampInS) {
            ramp_in_start_deploy_ = held_target_deploy_;
            phase_ = ChirpPhase::RampIn;
            phase_start_time_ = run_time;
            chirp_current_step_ = 0;
        }
    }

    // ======================================================================
    //  Phase: Ramp In
    // ======================================================================
    void RunRampIn(double run_time, double phase_elapsed) {
        if (chirp_trajectory_pace_.empty()) {
            EnterError("Chirp trajectory is empty");
            return;
        }

        VecXf first_chirp_pace = chirp_trajectory_pace_[0];
        VecXf first_chirp_deploy = PaceToDeploy(first_chirp_pace);

        if (is_command_update_) {
            float t = std::min(static_cast<float>(phase_elapsed), kRampInDurationS);
            for (int i = 0; i < 12; ++i) {
                held_target_deploy_(i) = CubicSplinePos(
                    ramp_in_start_deploy_(i), 0.0f,
                    first_chirp_deploy(i), 0.0f,
                    t, kRampInDurationS);
            }
            held_target_pace_ = DeployToPace(held_target_deploy_);
        }

        if (phase_elapsed >= kRampInDurationS) {
            phase_ = ChirpPhase::ChirpExcitation;
            phase_start_time_ = run_time;
            chirp_current_step_ = 0;
        }
    }

    // ======================================================================
    //  Phase: Chirp Excitation
    // ======================================================================
    void RunChirpExcitation(double run_time, double phase_elapsed) {
        if (chirp_trajectory_pace_.empty()) {
            EnterError("Chirp trajectory is empty");
            return;
        }

        // Chirp duration reached → ramp out
        if (phase_elapsed >= kChirpDurationS) {
            ramp_out_start_deploy_ = held_target_deploy_;
            phase_ = ChirpPhase::RampOut;
            phase_start_time_ = run_time;
            return;
        }

        // Determine which chirp command-update step corresponds to current time
        int target_step = static_cast<int>(phase_elapsed * kChirpCommandUpdateHz);

        if (is_command_update_ && target_step != chirp_current_step_) {
            // Get 400Hz chirp point closest to this command update
            int chirp_idx = target_step * static_cast<int>(kPaceSampleRateHz / kChirpCommandUpdateHz);
            if (chirp_idx >= chirp_total_steps_) chirp_idx = chirp_total_steps_ - 1;

            VecXf chirp_target_pace = chirp_trajectory_pace_[chirp_idx];
            held_target_pace_ = chirp_target_pace;
            held_target_deploy_ = PaceToDeploy(chirp_target_pace);
            chirp_current_step_ = target_step;
        }

        // Progress print
        static double last_chirp_print = -1;
        if (run_time - last_chirp_print >= 2.0) {
            int total_updates = static_cast<int>(kChirpDurationS * kChirpCommandUpdateHz);
            std::cout << "[PACE Chirp] Excitation: update " << chirp_current_step_
                      << "/" << total_updates
                      << " (t=" << std::fixed << std::setprecision(1) << phase_elapsed
                      << "s/" << kChirpDurationS << "s)\n";
            last_chirp_print = run_time;
        }
    }

    // ======================================================================
    //  Phase: Ramp Out
    // ======================================================================
    void RunRampOut(double run_time, double phase_elapsed) {
        if (is_command_update_) {
            float t = std::min(static_cast<float>(phase_elapsed), kRampOutDurationS);
            for (int i = 0; i < 12; ++i) {
                held_target_deploy_(i) = CubicSplinePos(
                    ramp_out_start_deploy_(i), 0.0f,
                    q_center_deploy_(i), 0.0f,
                    t, kRampOutDurationS);
            }
            held_target_pace_ = DeployToPace(held_target_deploy_);
        }

        if (phase_elapsed >= kRampOutDurationS) {
            phase_ = ChirpPhase::HoldAfter;
            phase_start_time_ = run_time;
        }
    }

    // ======================================================================
    //  Phase: Hold After
    // ======================================================================
    void RunHoldAfter(double run_time, double phase_elapsed) {
        held_target_deploy_ = q_center_deploy_;
        held_target_pace_ = q_center_pace_;

        if (phase_elapsed >= kHoldAfterRampOutS) {
            phase_ = ChirpPhase::SaveLog;
            phase_start_time_ = run_time;
        }
    }

    // ======================================================================
    //  Phase: Save Log
    // ======================================================================
    void RunSaveLog() {
        std::cout << "[PACE Chirp] Saving native log...\n";
        CloseCsvLog();
        WriteMetadataJson();
        std::cout << "[PACE Chirp] Native log saved to: " << csv_path_ << "\n";
        std::cout << "[PACE Chirp] Metadata saved to: " << metadata_path_ << "\n";

        // Disable PD before exiting
        GoToIdle();

        phase_ = ChirpPhase::Exit;
    }

    // ======================================================================
    //  Phase: Error
    // ======================================================================
    void RunErrorPhase() {
        // Disable PD
        GoToIdle();

        // Save log even on error
        if (csv_file_.is_open()) {
            CloseCsvLog();
            WriteMetadataJson();
            std::cout << "[PACE Chirp] Error log saved to: " << log_dir_ << "\n";
        }

        phase_ = ChirpPhase::Exit;
    }

    // ======================================================================
    //  OnExit
    // ======================================================================
    virtual void OnExit() override {
        GoToIdle();

        // Save log if not already saved
        if (csv_file_.is_open()) {
            CloseCsvLog();
            WriteMetadataJson();
        }

        std::cout << "[PACE Chirp] State exit complete. Data saved to: " << log_dir_ << "\n";
        log_(INFO, "PACEChirpCollectState exited");
    }

    // ======================================================================
    //  LoseControlJudge — emergency stop path must remain valid
    // ======================================================================
    virtual bool LoseControlJudge() override {
        if (uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping)) {
            std::cout << "[PACE Chirp] EMERGENCY: JointDamping requested, exiting chirp\n";
            return true;
        }
        return false;
    }

    // ======================================================================
    //  GetNextStateName
    // ======================================================================
    virtual StateName GetNextStateName() override {
        // Exit or Error phase → go to Idle
        if (phase_ == ChirpPhase::Exit || phase_ == ChirpPhase::Error) {
            return StateName::kIdle;
        }
        // Stay in current state
        return StateName::kPACEChirpCollect;
    }
};

#endif  // PACE_CHIRP_COLLECT_STATE_HPP_
