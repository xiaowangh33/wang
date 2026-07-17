/**
 * @file idle_state.hpp
 * @brief robot need to confirm sensor input while in idle state
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef IDLE_STATE_HPP_
#define IDLE_STATE_HPP_

#include <cstdlib>

#include "state_base.h"
#include "quardLeg_defines.h"


class IdleState : public StateBase{
private:
    bool joint_normal_flag_ = false, imu_normal_flag_ = false;
    bool first_enter_flag_ = true;
    VecXf joint_pos_, joint_vel_, joint_tau_;
    Vec3f rpy_, acc_, omg_;
    double enter_state_time_ = -10000.;
    double last_rl_entry_block_log_time_ = -10000.;
    double last_sensor_warning_time_ = -10000.;
    RobotBasicState rbs_;

    double last_print_time_ = -10000.;
    bool verbose_idle_output_ = []() {
        const char* value = std::getenv("WHEELDOG_IDLE_DEBUG");
        return value && (value[0] == '1' || value[0] == 't' || value[0] == 'T' ||
                         value[0] == 'y' || value[0] == 'Y');
    }();
    void GetProprioceptiveData(){
        joint_pos_ = ri_ptr_->GetJointPosition();
        joint_vel_ = ri_ptr_->GetJointVelocity();
        joint_tau_ = ri_ptr_->GetJointTorque();
        rpy_ = ri_ptr_->GetImuRpy();
        acc_ = ri_ptr_->GetImuAcc();
        omg_ = ri_ptr_->GetImuOmega();


        rbs_.base_rpy     = ri_ptr_->GetImuRpy();
        rbs_.base_rot_mat = RpyToRm(rbs_.base_rpy);
        rbs_.projected_gravity = RmToProjectedGravity(rbs_.base_rot_mat);
        rbs_.base_omega   = ri_ptr_->GetImuOmega();
        rbs_.base_acc     = ri_ptr_->GetImuAcc();
        rbs_.joint_pos    = ri_ptr_->GetJointPosition();
        rbs_.joint_vel    = ri_ptr_->GetJointVelocity();
        rbs_.joint_tau    = ri_ptr_->GetJointTorque();
        copy_robo_state_to_shared_memory(rbs_, robot_model::kTotalDof);
    }

    bool JointDataNormalCheck(){
        if(joint_pos_.size() != robot_model::kTotalDof ||
           joint_vel_.size() != robot_model::kTotalDof ||
           joint_tau_.size() != robot_model::kTotalDof){
            return false;
        }
        for(int i=0;i<robot_model::kTotalDof;++i){
            if(!std::isfinite(joint_pos_(i)) ||
               !std::isfinite(joint_vel_(i)) ||
               !std::isfinite(joint_tau_(i))){
                return false;
            }
#ifndef BUILD_SIMULATION
            const auto& joint = robot_model::kJoints[i];
            if(joint.mode == robot_model::ActuatorMode::PositionPD &&
               (joint_pos_(i) < joint.urdf_lower_rad - 0.2f ||
                joint_pos_(i) > joint.urdf_upper_rad + 0.2f)){
                return false;
            }
            if(std::fabs(joint_vel_(i)) > joint.velocity_limit_radps + 0.5f){
                return false;
            }
#endif
        }
        return true;
    }
        
    bool ImuDataNormalCheck(){
        for(int i=0;i<3;++i){
            if(!std::isfinite(rpy_(i))){
                return false;
            } 
            if(!std::isfinite(omg_(i))){
                return false;
            }
            if(!std::isfinite(acc_(i))){
                return false;
            }
        }
#ifndef BUILD_SIMULATION
        for(int i=0;i<3;++i){
            if(fabs(rpy_(i)) > M_PI){
                return false;
            }
            if(fabs(omg_(i)) > M_PI){
                return false;
            }
        }
        // RobotInterface 约定加速度单位为 m/s²；静止时模长应接近 gravity。
        if(acc_.norm() < 0.1*gravity || acc_.norm() > 3.0*gravity){
            return false;
        }
#endif
        return true;
    }

    void DisplayProprioceptiveInfo(){
        std::cout << "version-1217-2.1.1";
        std::cout << "Joint Data: \n";
        std::cout << "pos: " << joint_pos_.transpose() << std::endl;
        std::cout << "vel: " << joint_vel_.transpose() << std::endl;
        std::cout << "tau: " << joint_tau_.transpose() << std::endl;
        std::cout << "Imu Data: \n";
        std::cout << "rpy: " << rpy_.transpose() << std::endl;
        std::cout << "acc: " << acc_.transpose() << std::endl;
        std::cout << "omg: " << omg_.transpose() << std::endl;
    }

    void DisplayAxisValue(){
        auto cmd = uc_ptr_->GetUserCommand();
        std::cout << "User Command Input: \n";
        std::cout << "axis value:  " << cmd.forward_vel_scale << " " 
                                     << cmd.side_vel_scale << " "
                                     << cmd.turnning_vel_scale << std::endl;
        std::cout << "target mode: " << cmd.target_mode << std::endl;
    }


public:
    IdleState(const RobotType& robot_type, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_type, state_name, data_ptr){
        }
    ~IdleState(){}

    virtual void OnEnter() {
        log_(INFO,"IdleState entered" );

        for(int i=0;i<robot_model::kTotalDof;++i){
            robot_data_all->joint_cmd[i].kp=0; // kp
            robot_data_all->joint_cmd[i].goal_joint_pos=robot_data_all->robot_data.JointPosition[i]; // goal joint pos
            robot_data_all->joint_cmd[i].kd=0; // kd
            robot_data_all->joint_cmd[i].goal_joint_vel=0; // goal joint vel
            robot_data_all->joint_cmd[i].torque_feedforward=0; // tau feedforward
        }

        StateBase::msfb_.UpdateCurrentState(RobotMotionState::WaitingForStand);
        std::cout << "Waiting for stand up..." << std::endl;
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
        enter_state_time_ = ri_ptr_->GetInterfaceTimeStamp();
    };
    virtual void OnExit() {
        first_enter_flag_ = false;
    }
    virtual void Run() {
        GetProprioceptiveData();
        joint_normal_flag_ = JointDataNormalCheck();
        imu_normal_flag_ = ImuDataNormalCheck();
        const double now = ri_ptr_->GetInterfaceTimeStamp();
        if (verbose_idle_output_ && now - last_print_time_ >= 3.0) {
            DisplayProprioceptiveInfo();
            DisplayAxisValue();
            last_print_time_ = now;
        }
        MatXf cmd = MatXf::Zero(robot_model::kTotalDof, 5);
        for(int i=0;i<robot_model::kTotalDof;++i){
            cmd(i, 0) = robot_data_all->joint_cmd[i].kp; // kp
            cmd(i, 1) = robot_data_all->joint_cmd[i].goal_joint_pos; // goal joint pos
            cmd(i, 2) = robot_data_all->joint_cmd[i].kd; // kd
            cmd(i, 3) = robot_data_all->joint_cmd[i].goal_joint_vel; // goal joint vel
            cmd(i, 4) = robot_data_all->joint_cmd[i].torque_feedforward; // tau feedforward
        }
        ri_ptr_->SetJointCommand(cmd);
    }

    virtual bool LoseControlJudge() {
        return false;
    }

    virtual StateName GetNextStateName() {
        // std::cout << "Current target_mode = " << uc_ptr_->GetUserCommand().target_mode << std::endl;

        if(first_enter_flag_ && ri_ptr_->GetInterfaceTimeStamp() - enter_state_time_ < 0.5){
            return StateName::kIdle;
        }

#ifndef BUILD_SIMULATION
        if(!joint_normal_flag_ || !imu_normal_flag_) {
            const double now = ri_ptr_->GetInterfaceTimeStamp();
            if(now - last_sensor_warning_time_ >= 3.0){
                std::cout << "Sensor data not ready: joint=" << joint_normal_flag_
                          << " imu=" << imu_normal_flag_ << std::endl;
                last_sensor_warning_time_ = now;
            }
            return StateName::kIdle;
        }
#endif
            
        if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::RLControlMode)) {
#ifndef BUILD_SIMULATION
            const double now = ri_ptr_->GetInterfaceTimeStamp();
            if(now - last_rl_entry_block_log_time_ >= 3.0){
                std::cerr << "[Idle] RL entry blocked on hardware: press z first; "
                          << "StandUp must verify every joint before c" << std::endl;
                last_rl_entry_block_log_time_ = now;
            }
            return StateName::kIdle;
#else
            return StateName::kRLControl;
#endif
        }

        if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::PACEChirpCollect)) {
            return StateName::kPACEChirpCollect;
        }

        if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::StandingUp)) return StateName::kStandUp;

        return StateName::kIdle;
    }


    
};




#endif
