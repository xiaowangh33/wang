#include "control_parameters.h"
#include "robot_model_config.hpp"

void ControlParameters::GenerateMydogParameters(){
    body_len_x_ = robot_model::kBodyLenX;
    body_len_y_ = robot_model::kBodyLenY;
    hip_len_ = robot_model::kHipLen;
    thigh_len_ = robot_model::kThighLen;
    shank_len_ = robot_model::kShankLen;

    pre_height_ = robot_model::kPreHeight;
    stand_height_ = robot_model::kStandHeight;
    
    swing_leg_kp_ << 65.0, 65.0, 65.0;
    swing_leg_kd_ << 1.0, 1.0, 1.0;

    stand_duration_ = robot_model::kStandDuration;
}
