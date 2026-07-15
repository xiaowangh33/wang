#include "quardLeg_defines.h"

share_mem *sbf_robot_state;
ShmRobotDataAll *robot_data_all;

void copy_robo_state_to_shared_memory(RobotBasicState &robot_state,int dof)
{
    if (!robot_data_all) {
        return;
    }
    ShmRobotData *p = &(robot_data_all->robot_data);
    int n = dof > DOF_COUNT ? DOF_COUNT : dof;
    if (n < 0) {
        n = 0;
    }
    for(int i=0;i<n;i++){
        p->JointPosition[i] = robot_state.joint_pos[i];
        p->JointVelocity[i] = robot_state.joint_vel[i];
        p->JointTorque[i] = robot_state.joint_tau[i];
    }
    for(int i=0;i<3;i++){
        p->ImuRpy[i] = robot_state.base_rpy[i];
        p->ImuAcc[i] = robot_state.base_acc[i];  
    }
        for(int i=0;i<3;i++){
            p->ImuOmega[i] = robot_state.base_omega[i];  
        }   
        
}
void copy_robot_action_to_shared_memory(RobotAction &robot_action,int dof)
{
    if (!robot_data_all) {
        return;
    }
    ShmJointCmd *p = robot_data_all->joint_cmd;
    int n = dof > DOF_COUNT ? DOF_COUNT : dof;
    if (n < 0) {
        n = 0;
    }
    for(int i=0;i<n;i++){
        p[i].goal_joint_pos = robot_action.goal_joint_pos[i];
        p[i].goal_joint_vel = robot_action.goal_joint_vel[i];
        p[i].kp = robot_action.kp[i];
        p[i].kd = robot_action.kd[i];
        p[i].torque_feedforward = robot_action.tau_ff[i];
    }
}

void copy_joint_command_mat_to_shared_memory(const MatXf& joint_cmd_mat, int dof)
{
    if (!robot_data_all) {
        return;
    }
    if (joint_cmd_mat.cols() < 5) {
        return;
    }
    ShmJointCmd *p = robot_data_all->joint_cmd;
    int n = dof > DOF_COUNT ? DOF_COUNT : dof;
    if (n < 0) {
        n = 0;
    }
    const int rows = static_cast<int>(joint_cmd_mat.rows());
    if (rows < n) {
        n = rows;
    }
    for (int i = 0; i < n; ++i) {
        p[i].kp = joint_cmd_mat(i, 0);
        p[i].goal_joint_pos = joint_cmd_mat(i, 1);
        p[i].kd = joint_cmd_mat(i, 2);
        p[i].goal_joint_vel = joint_cmd_mat(i, 3);
        p[i].torque_feedforward = joint_cmd_mat(i, 4);
    }
}

void init_RobotDataAll()
{
    if (!robot_data_all) {
        return;
    }
    robot_data_all->kp_base[0] = robot_model::kJoints[0].kp_default;  // HipX
    robot_data_all->kp_base[1] = robot_model::kJoints[1].kp_default;  // HipY
    robot_data_all->kp_base[2] = robot_model::kJoints[2].kp_default;  // Knee
    robot_data_all->kd_base[0] = robot_model::kJoints[0].kd_default;  // HipX
    robot_data_all->kd_base[1] = robot_model::kJoints[1].kd_default;  // HipY
    robot_data_all->kd_base[2] = robot_model::kJoints[2].kd_default;  // Knee

    for (int i = 0; i < DOF_COUNT && i < robot_model::kTotalDof; ++i) {
        const auto& joint = robot_model::kJoints[i];
        robot_data_all->actuator_mode[i] = static_cast<uint8_t>(joint.mode);
        robot_data_all->joint_cmd[i].kp = joint.kp_default;
        robot_data_all->joint_cmd[i].goal_joint_pos = joint.pos_default_rad;
        robot_data_all->joint_cmd[i].kd = joint.kd_default;
        robot_data_all->joint_cmd[i].goal_joint_vel = 0.0f;
        robot_data_all->joint_cmd[i].torque_feedforward = 0.0f;
    }

    robot_data_all->front_leg_kp_scale = 1.0f;
    robot_data_all->back_leg_kp_scale = 1.0f;
    robot_data_all->front_leg_kd_scale = 1.0f;
    robot_data_all->back_leg_kd_scale = 1.0f;
}
