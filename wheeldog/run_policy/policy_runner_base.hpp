/**
 * @file policy_runner_base.hpp
 * @brief policy runner base class
 * @author mazunwang
 * @version 1.0
 * @date 2024-06-06
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#ifndef POLICY_RUNNER_BASE_HPP_
#define POLICY_RUNNER_BASE_HPP_


#include "common_types.h"

using namespace types;

class PolicyRunnerBase{
public:
    PolicyRunnerBase(std::string policy_name)
        : policy_name_(policy_name), decimation_(1), run_cnt_(0) {}
    virtual ~PolicyRunnerBase(){}
    /**
     * @brief use to display policy detail
     */
    virtual void DisplayPolicyInfo() = 0;

    /**
     * @brief Get the robot action object by run your policy
     * @return RobotAction 
     */
    virtual RobotAction GetRobotAction(const RobotBasicState&) = 0;

    /**
     * @brief execute function when first entering policy runner
     */
    virtual void OnEnter() = 0;

    /**
     * @brief 仅按当前机器人状态做观测构建与策略推理，更新内部 last_action 等状态，不生成下发关节命令。
     * 用于站立保持阶段预热，避免切入 RL 时首帧 last_action/观测突变。
     */
    virtual void WarmupObservationOnly(const RobotBasicState& ro) { (void)ro; }

    /**
     * @brief Set the decimation
     * @param  d decimation
     */
    virtual void SetDecimation(int d){
        decimation_ = d;
    }

    const std::string policy_name_;
    int decimation_;
    int run_cnt_;
};






#endif