#pragma once
#ifndef _ROBOT_STATE_SHM_H_
#define _ROBOT_STATE_SHM_H_

#include <stdint.h>
#include <stddef.h>

/****共享内存结构体类型为： ShmRobotDataAll****/
// ====== variables / constants ======
constexpr int32_t DOF_COUNT = 16;

#pragma pack(push, 1)

typedef struct __attribute__((__packed__)) ShmJointCmd {
    float kp;
    float goal_joint_pos;
    float kd;
    float goal_joint_vel;
    float torque_feedforward;
} ShmJointCmd;

typedef struct __attribute__((__packed__)) ShmRobotData {
    float JointPosition[DOF_COUNT];
    float JointVelocity[DOF_COUNT];
    float JointTorque[DOF_COUNT];
    float ImuRpy[3];
    float ImuAcc[3];
    float ImuOmega[3];
} ShmRobotData;

typedef struct __attribute__((__packed__)) ShmRobotDataAll {
    struct ShmRobotData robot_data;
    struct ShmJointCmd joint_cmd[DOF_COUNT];
    uint8_t actuator_mode[DOF_COUNT];
    float kp_base[3];
    float kd_base[3];
    float front_leg_kp_scale;
    float back_leg_kp_scale;
    float front_leg_kd_scale;
    float back_leg_kd_scale;
} ShmRobotDataAll;

#pragma pack(pop)

#endif // _ROBOT_STATE_SHM_H_
