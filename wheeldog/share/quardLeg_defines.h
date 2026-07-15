#pragma once
#ifndef DARM_DEFINES_H
#define DARM_DEFINES_H


#include "sys/types.h"
#include <stdint.h>
#include <stdio.h>  
#include <sys/sem.h>  
#include <stdlib.h>  
#include "robot_state_shm.h"
#include "robo_core.h"
#include "common_types.h"

using namespace ROBO_CORE;
using namespace types;
#define PROGRAM_ROOT_PATH    "PROGRAM_ROOT_PATH" //系统的变量名称




extern share_mem *sbf_robot_state;
extern ShmRobotDataAll *robot_data_all;
void init_RobotDataAll();
void copy_robo_state_to_shared_memory(RobotBasicState &robo_state,int dof);
void copy_robot_action_to_shared_memory(RobotAction &robot_action,int dof);
/** 与 SetJointCommand 的 5 列矩阵一致：[kp, goal_pos, kd, goal_vel, tau_ff]，供 SHM 与真机指令对齐 */
void copy_joint_command_mat_to_shared_memory(const MatXf& joint_cmd_mat, int dof);
#endif
