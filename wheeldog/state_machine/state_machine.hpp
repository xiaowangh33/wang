/**
 * @file state_machine.hpp
 * @brief for robot to switch control state by user command input
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef STATE_MACHINE_HPP_
#define STATE_MACHINE_HPP_

#include <cstdlib>  // std::getenv
#include "state_base.h"
#include "idle_state.hpp"
#include "standup_state.hpp"
#include "joint_damping_state.hpp"
#include "sitdown_state.hpp"

// #ifdef USE_ONNX
//     #include "rl_control_state_onnx.hpp"
// #else   
//     #include "rl_control_state.hpp"
// #endif

#include "rl_control_state_onnx.hpp"

#include "skydroid_gamepad_interface.hpp"
#include "retroid_gamepad_interface.hpp"
#include "keyboard_interface.hpp"
#ifdef USE_RAISIM
    #include "simulation/jueying_raisim_simulation.hpp"
#endif
#ifdef USE_MJCPP
    #include "simulation/mujoco_interface.hpp"
#endif

// 硬件接口：只在非仿真模式下包含
#ifndef BUILD_SIMULATION
    #include "hardware/wheeled_dog_hardware_interface.hpp"
#endif

#include "data_streaming.hpp"

class StateMachine{
private:
    std::shared_ptr<StateBase> current_controller_;
    std::shared_ptr<StateBase> idle_controller_;
    std::shared_ptr<StateBase> standup_controller_;
    std::shared_ptr<StateBase> rl_controller_;
    std::shared_ptr<StateBase> joint_damping_controller_;
    std::shared_ptr<StateBase> sitdown_controller_;

    StateName current_state_name_, next_state_name_;

    std::shared_ptr<UserCommandInterface> uc_ptr_;
    std::shared_ptr<RobotInterface> ri_ptr_;
    std::shared_ptr<ControlParameters> cp_ptr_;

    std::shared_ptr<DataStreaming> ds_ptr_;

    static std::string GetEnvOrDefault(const char* name, const char* fallback){
        const char* value = std::getenv(name);
        return (value && value[0] != '\0') ? std::string(value) : std::string(fallback);
    }

    static int GetEnvIntOrDefault(const char* name, int fallback){
        const char* value = std::getenv(name);
        if(!value || value[0] == '\0'){
            return fallback;
        }
        char* end = nullptr;
        const long parsed = std::strtol(value, &end, 10);
        if(end == value || *end != '\0' || parsed <= 0 || parsed > 4000000){
            std::cerr << "Ignoring invalid " << name << "=" << value << std::endl;
            return fallback;
        }
        return static_cast<int>(parsed);
    }

    void GetDataStreaming(){
        if(!ri_ptr_) return;
        VecXf pos = ri_ptr_->GetJointPosition();
        VecXf vel = ri_ptr_->GetJointVelocity();
        VecXf tau = ri_ptr_->GetJointTorque();
        Vec3f rpy = ri_ptr_->GetImuRpy();
        Vec3f acc = ri_ptr_->GetImuAcc();
        Vec3f omg = ri_ptr_->GetImuOmega();
        MatXf jc = ri_ptr_->GetJointCommand();

        ds_ptr_->InsertInterfaceTime(ri_ptr_->GetInterfaceTimeStamp());
        ds_ptr_->InsertJointData("q", pos);
        ds_ptr_->InsertJointData("dq", vel);
        ds_ptr_->InsertJointData("tau", tau);
        ds_ptr_->InsertJointData("q_cmd", jc.col(1));
        ds_ptr_->InsertJointData("tau_ff", jc.col(4));

        ds_ptr_->InsertImuData("rpy", rpy);
        ds_ptr_->InsertImuData("acc", acc);
        ds_ptr_->InsertImuData("omg", omg);

        if(!uc_ptr_) return;
        auto cmd = uc_ptr_->GetUserCommand();
        ds_ptr_->InsertCommandData("target_mode", float(cmd.target_mode));

        ds_ptr_->InsertStateData("current_state", StateBase::msfb_.current_state);
       
        ds_ptr_->SendData();
    }

    std::shared_ptr<StateBase> GetNextStatePtr(StateName state_name){
        switch(state_name){
            case StateName::kInvalid:{
                return nullptr;
            }
            case StateName::kIdle:{
                return idle_controller_;
            }
            case StateName::kStandUp:{
                return standup_controller_;
            }
            case StateName::kRLControl:{
                return rl_controller_;
            }
            case StateName::kJointDamping:{
                return joint_damping_controller_;
            }
            case StateName::kPACEChirpCollect:{
                return nullptr;
            }
            case StateName::kSitDown:{
                return sitdown_controller_;
            }
            default:{
                std::cerr << "error state name" << std::endl;
            }
        }
        return nullptr;
    }
public:
    StateMachine(RobotType robot_type){
        const std::string activation_key = "~/raisim/activation.raisim";
        std::string urdf_path = "";
        std::string mjcf_path = "";
        // 使用键盘接口（仿真和实际硬件都支持）
        uc_ptr_ = std::make_shared<KeyboardInterface>();
        // 如果需要使用游戏手柄，可以取消注释下面一行并注释上面一行
        // uc_ptr_ = std::make_shared<RetroidGamepadInterface>(12121);
        if(robot_type == RobotType::Mydog){
            urdf_path = GetAbsPath()+"/wheeldog_description/mywheeldog/urdf/wheel_legged_dog.urdf";
            // 通过环境变量 MUJOCO_SCENE 预留地形切换入口；当前默认使用新轮足狗平地 XML。
            {
                const char* scene = std::getenv("MUJOCO_SCENE");
                if (scene && std::string(scene) == "stairs") {
                    mjcf_path = GetAbsPath()+"/description/wheel_legged_dog_mujoco.xml";
                } else {
                    mjcf_path = GetAbsPath()+"/description/wheel_legged_dog_mujoco.xml";
                }
            }
            #ifdef USE_RAISIM
                ri_ptr_ = std::make_shared<JueyingRaisimSimulation>(activation_key, urdf_path, "Mydog_sim");

            #elif defined(USE_MJCPP)
                ri_ptr_ = std::make_shared<MujocoInterface>("WheeledDog", mjcf_path, robot_model::kTotalDof);
                std::cout << "Using MujocoInterface CPP " << std::endl;
                std::cout << "mjcf_path: " << mjcf_path << std::endl;
            #else
                ri_ptr_ = std::make_shared<WheeledDogHardwareInterface>("WheeledDogHardware");
            #endif
            cp_ptr_ = std::make_shared<ControlParameters>(robot_type);
        }else{
            std::cerr << "error" << std::endl;
        }

        std::shared_ptr<ControllerData> data_ptr = std::make_shared<ControllerData>();
        data_ptr->ri_ptr = ri_ptr_;
        data_ptr->uc_ptr = uc_ptr_;
        data_ptr->cp_ptr = cp_ptr_;
        ds_ptr_ = std::make_shared<DataStreaming>(false, false);
        data_ptr->ds_ptr = ds_ptr_;

        idle_controller_ = std::make_shared<IdleState>(robot_type, "idle_state", data_ptr);
        standup_controller_ = std::make_shared<StandUpState>(robot_type, "standup_state", data_ptr);

        // 测试ONNX，后续需要改成参数控制
        // rl_controller_ = std::make_shared<RLControlState>(robot_type, "rl_control", data_ptr);
        // #ifdef USE_ONNX
        //     rl_controller_ = std::make_shared<RLControlStateONNX>(robot_type, "rl_control", data_ptr);
        // #else
        //     rl_controller_ = std::make_shared<RLControlState>(robot_type, "rl_control", data_ptr);
        // #endif
        rl_controller_ = std::make_shared<RLControlStateONNX>(robot_type, "rl_control", data_ptr);
        


        joint_damping_controller_ = std::make_shared<JointDampingState>(robot_type, "joint_damping", data_ptr);

        sitdown_controller_ = std::make_shared<SitDownState>(robot_type, "sitdown_state", data_ptr);

        current_controller_ = idle_controller_;
        current_state_name_ = kIdle;
        next_state_name_ = kIdle;
   
        // std::cout << "Controller will be enabled in 3 seconds!!!" << std::endl;
        // std::this_thread::sleep_for(std::chrono::seconds(3)); //for safety 

        ri_ptr_->Start();
        std::cout << "Robot interface started" << std::endl;
        uc_ptr_->Start();
        
        current_controller_->OnEnter();  
    }
    ~StateMachine(){}

    void Run(){
        int cnt = 0;
        static double time_record = 0;
        while(true && ROBO_CORE::check_process_ok()){
            if(ri_ptr_->GetInterfaceTimeStamp()!= time_record){
                time_record = ri_ptr_->GetInterfaceTimeStamp();
                current_controller_ -> Run();
                
                if(current_controller_->LoseControlJudge()) next_state_name_ = StateName::kJointDamping;
                else next_state_name_ = current_controller_ -> GetNextStateName();
                
                if(next_state_name_ != current_state_name_){
                    auto next_controller = GetNextStatePtr(next_state_name_);
                    if(!next_controller){
                        std::cerr << "Ignoring transition to invalid state "
                                  << static_cast<int>(next_state_name_) << std::endl;
                        next_state_name_ = current_state_name_;
                    }else{
                        current_controller_ -> OnExit();
                        std::cout << current_controller_ -> state_name_ << " ------------> ";
                        current_controller_ = next_controller;
                        std::cout << current_controller_ -> state_name_ << std::endl;
                        current_controller_ ->OnEnter();
                        current_state_name_ = next_state_name_;
                    }
                }
                ++cnt;
                this->GetDataStreaming();
            }
            std::this_thread::sleep_for(std::chrono::microseconds(500));

            ROBO_CORE::tick_process_heart_beats();
        }

        log_(INFO,"StateMachine is stopping...\n");

        current_controller_->OnExit();
        uc_ptr_->Stop();
        log_(INFO,"UserCommandInterface is stopped\n");
        ri_ptr_->Stop();
        log_(INFO,"RobotInterface is stopped\n");
    }

};

#endif
