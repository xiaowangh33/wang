/**
 * @file joint_damping_state.hpp
 * @brief joint passive control state
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef JOINT_DAMPING_STATE_HPP_
#define JOINT_DAMPING_STATE_HPP_

#include "state_base.h"
#include "quardLeg_defines.h"

class JointDampingState : public StateBase{
private:
    float time_record_ = 0.0f, run_time_ = 0.0f;
    MatXf joint_cmd_;
    RobotBasicState rbs_;

public:
    JointDampingState(const RobotType& robot_type, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_type, state_name, data_ptr){
            joint_cmd_ = MatXf::Zero(robot_model::kTotalDof, 5);
        }
    ~JointDampingState(){}

    virtual void OnEnter() {
        log_(INFO,"JointDampingState entered" );
        time_record_ = ri_ptr_->GetInterfaceTimeStamp();
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();


        rbs_.base_rpy     = ri_ptr_->GetImuRpy();
        rbs_.base_rot_mat = RpyToRm(rbs_.base_rpy);
        rbs_.projected_gravity = RmToProjectedGravity(rbs_.base_rot_mat);
        rbs_.base_omega   = ri_ptr_->GetImuOmega();
        rbs_.base_acc     = ri_ptr_->GetImuAcc();
        rbs_.joint_pos    = ri_ptr_->GetJointPosition();
        rbs_.joint_vel    = ri_ptr_->GetJointVelocity();
        rbs_.joint_tau    = ri_ptr_->GetJointTorque();
        copy_robo_state_to_shared_memory(rbs_, robot_model::kTotalDof);
        for(int i=0;i<robot_model::kTotalDof;++i){
            robot_data_all->joint_cmd[i].kp=0; // kp
            robot_data_all->joint_cmd[i].goal_joint_pos=robot_data_all->robot_data.JointPosition[i]; // goal joint pos
            robot_data_all->joint_cmd[i].kd=
                robot_model::kJoints[i].kd_default;
            robot_data_all->joint_cmd[i].goal_joint_vel=0; // goal joint vel
            robot_data_all->joint_cmd[i].torque_feedforward=0; // tau feedforward
        }
    };
    virtual void OnExit() {

    }
    virtual void Run() {
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();

        rbs_.base_rpy     = ri_ptr_->GetImuRpy();
        rbs_.base_rot_mat = RpyToRm(rbs_.base_rpy);
        rbs_.projected_gravity = RmToProjectedGravity(rbs_.base_rot_mat);
        rbs_.base_omega   = ri_ptr_->GetImuOmega();
        rbs_.base_acc     = ri_ptr_->GetImuAcc();
        rbs_.joint_pos    = ri_ptr_->GetJointPosition();
        rbs_.joint_vel    = ri_ptr_->GetJointVelocity();
        rbs_.joint_tau    = ri_ptr_->GetJointTorque();
        copy_robo_state_to_shared_memory(rbs_, robot_model::kTotalDof);

        // for(int i=0;i<12;++i){
        //     robot_data_all->joint_cmd[i].kp=0; // kp
        //     robot_data_all->joint_cmd[i].goal_joint_pos=robot_data_all->robot_data.JointPosition[i]; // goal joint pos
        //     robot_data_all->joint_cmd[i].kd=100; // kd
        //     robot_data_all->joint_cmd[i].goal_joint_vel=0; // goal joint vel
        //     robot_data_all->joint_cmd[i].torque_feedforward=0; // tau feedforward
        // }


        for(int i=0;i<robot_model::kTotalDof;++i){
            joint_cmd_(i, 0) = robot_data_all->joint_cmd[i].kp; // kp
            joint_cmd_(i, 1) = robot_data_all->joint_cmd[i].goal_joint_pos; // goal joint pos
            joint_cmd_(i, 2) = robot_data_all->joint_cmd[i].kd; // kd
            joint_cmd_(i, 3) = robot_data_all->joint_cmd[i].goal_joint_vel; // goal joint vel
            joint_cmd_(i, 4) = robot_data_all->joint_cmd[i].torque_feedforward; // tau feedforward
        }

        ri_ptr_->SetJointCommand(joint_cmd_);


    }
    virtual bool LoseControlJudge() {
        return false;
    }
    virtual StateName GetNextStateName() {
        if(run_time_ - time_record_ < 3.){
            return StateName::kJointDamping;
        }
        return StateName::kIdle;
    }
};





#endif
