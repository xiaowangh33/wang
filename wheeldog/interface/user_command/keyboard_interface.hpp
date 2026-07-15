#ifndef KEYBOARD_INTERFACE_HPP_
#define KEYBOARD_INTERFACE_HPP_

#include "user_command_interface.h"
#include "custom_types.h"
#include <cstdio>
#include <functional>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <mutex>
#include <chrono>
#include <thread>
#include <atomic>

#define AXIS_STEP 0.15
// 新逻辑：方向键/控制键按下后，直接给固定满速并保持
static constexpr float kDirectFullSpeed = 1.0f;  // 直接使用策略训练的满速范围
// 最后一次按键后延迟归零时间 (ms)：0=立刻归零
// 终端字符输入没有 key-up 事件，因此默认保留 500ms；可用 RL_KEYBOARD_HOLD_MS 调节。
inline double GetCommandHoldMs() {
    static const double value = []() -> double {
        const char* env = std::getenv("RL_KEYBOARD_HOLD_MS");
        if (env) { double v = std::strtod(env, nullptr); if (v >= 0.0) return v; }
        return 500.0;  // 默认松开后 500ms 归零
    }();
    return value;
}
static constexpr bool kEnableCommandTimeout = true;

using namespace interface;
using namespace types;

class KeyboardInterface : public UserCommandInterface
{
private:
    UserCommand usr_cmd_;
    std::atomic<bool> start_thread_flag_{false};
    std::thread kb_thread_;
    std::mutex mtx_;  //  保护 usr_cmd_ 和 msfb_
    
    void ClipNumber(float &num, float low, float up){
        if(low > up) std::cerr << "error clip" << std::endl;
        if(num < low) num = low;
        if(num > up) num = up;
    }

    double GetCurrentTimeStamp(){
        static timespec startup_timestamp;
        timespec now_timestamp;
        if (startup_timestamp.tv_sec + startup_timestamp.tv_nsec == 0) {
            clock_gettime(CLOCK_MONOTONIC,&startup_timestamp);
        }
        clock_gettime(CLOCK_MONOTONIC,&now_timestamp);
        return (now_timestamp.tv_sec-startup_timestamp.tv_sec)*1e3 
            + (now_timestamp.tv_nsec-startup_timestamp.tv_nsec)/1e6;
    }

public:
    KeyboardInterface(){
        std::cout << "Using Keyboard Command Interface" << std::endl;
    }
    ~KeyboardInterface(){
        Stop();
    }

    virtual void Start(){
        if(kb_thread_.joinable()){
            return;
        }
        start_thread_flag_.store(true);
        kb_thread_ = std::thread(std::bind(&KeyboardInterface::Run, this));
    }
    virtual void Stop(){
        start_thread_flag_.store(false);
        if(kb_thread_.joinable() && kb_thread_.get_id() != std::this_thread::get_id()){
            kb_thread_.join();
        }
    }
    virtual UserCommand GetUserCommand() override {
        std::lock_guard<std::mutex> lock(mtx_);
        return usr_cmd_;
    }

    virtual void SetMotionStateFeedback(const MotionStateFeedback& msfb) override {
        std::lock_guard<std::mutex> lock(mtx_);
        msfb_ = msfb;
    }


    void Run(){
        struct termios oldt, newt;
        const bool terminal_configured = (tcgetattr(STDIN_FILENO, &oldt) == 0);
        if(terminal_configured){
            newt = oldt;
            newt.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        }
        int old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if(old_flags >= 0){
            fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK);
        }

        char input;
        double forward_time_record = GetCurrentTimeStamp();
        double side_time_record = GetCurrentTimeStamp();
        double turnning_time_record = GetCurrentTimeStamp();
        std::cout << "==========================================\n";
        std::cout << "  键盘控制已启动\n";
        std::cout << "==========================================\n";
        std::cout << "控制键说明：\n";
        std::cout << "  r - 关节阻尼模式\n";
        std::cout << "  z - 站立模式（在等待状态时）\n";
        std::cout << "  c - RL控制模式\n";
        std::cout << "  p - PACE chirp数据采集模式（在等待状态时）\n";
        std::cout << "  w/s - 前进/后退（归一化命令±1.0，默认保持500ms）\n";
        std::cout << "  a/d - 左/右移动（归一化命令±1.0，默认保持500ms）\n";
        std::cout << "  q/e - 左/右转向（归一化命令±1.0，默认保持500ms）\n";
        std::cout << "  space - 清零速度命令\n";
        std::cout << "==========================================\n";
        std::cout << "等待键盘输入...\n";
        fflush(stdout);
        
        while (start_thread_flag_.load()) {
            double current_time = GetCurrentTimeStamp();
            bool got_input = (read(STDIN_FILENO, &input, 1) > 0);
            if(got_input){
                double current_time = GetCurrentTimeStamp();
                std::lock_guard<std::mutex> lock(mtx_);  // 修改 usr_cmd_ 和读取 msfb_

                // 每次有按键输入时清除一次性触发器（避免残留触发）
                usr_cmd_.sitdown_trigger = false;

                std::cout << "\n[键盘] 收到输入: '" << input << "' (ASCII: " << (int)input << ")\n";
                fflush(stdout);
                if(input == 'r'){
                    usr_cmd_.target_mode = int(RobotMotionState::JointDamping);
                    std::cout << "[键盘] 切换到关节阻尼模式\n";
                    fflush(stdout);
                }
                // usr_cmd_.down_shift = false;
                // usr_cmd_.up_shift = false;
                switch(msfb_.current_state) {
                    case RobotMotionState::WaitingForStand:
                        if(input=='z'){
                            usr_cmd_.target_mode = int(RobotMotionState::StandingUp);
                            std::cout << "[键盘] 切换到站立模式\n";
                            fflush(stdout);
                        }
                        // 支持直接从idle进入RL控制模式（按'c'键）
                        if(input=='c'){
                            usr_cmd_.target_mode = int(RobotMotionState::RLControlMode);
                            std::cout << "[键盘] 切换到RL控制模式\n";
                            fflush(stdout);
                        }
                        // 支持从idle进入PACE chirp数据采集模式（按'p'键）
                        if(input=='p'){
                            usr_cmd_.target_mode = int(RobotMotionState::PACEChirpCollect);
                            std::cout << "[键盘] 切换到PACE chirp数据采集模式\n";
                            fflush(stdout);
                        }
                    break;
                    case RobotMotionState::StandingUp:
                        if(input=='c'){
                            usr_cmd_.target_mode = int(RobotMotionState::RLControlMode);
                            std::cout << "[键盘] 切换到RL控制模式\n";
                            fflush(stdout);
                        }
                        if(input=='p'){
                            usr_cmd_.target_mode = int(RobotMotionState::PACEChirpCollect);
                            std::cout << "[键盘] 切换到PACE chirp数据采集模式\n";
                            fflush(stdout);
                        }
                        // 'x' 在 Standing 状态触发 SitDown（回 0 位）
                        if(input=='x'){
                            usr_cmd_.target_mode = -1;   // 清除残留 target_mode，防止回 idle 后重新站起
                            usr_cmd_.sitdown_trigger = true;
                            std::cout << "[键盘] 触发 SitDown: 回 0 位\n";
                            fflush(stdout);
                        }
                    break;
                    case RobotMotionState::RLControlMode:
                        // 旧逻辑（保留）：步进增减
                        // if(input=='w') {
                        //     usr_cmd_.forward_vel_scale+=AXIS_STEP;
                        //     forward_time_record = current_time;
                        // }  
                        // else if(input=='s') {
                        //     usr_cmd_.forward_vel_scale-=AXIS_STEP;
                        //     forward_time_record = current_time;
                        // }
                        //
                        // if(input=='a') {
                        //     usr_cmd_.side_vel_scale-=AXIS_STEP;
                        //     side_time_record = current_time;
                        // }
                        // else if(input=='d') {
                        //     usr_cmd_.side_vel_scale+=AXIS_STEP;
                        //     side_time_record = current_time;
                        // }
                        //
                        // if(input=='q') {
                        //     usr_cmd_.turnning_vel_scale-=AXIS_STEP;
                        //     turnning_time_record = current_time;
                        // }
                        // else if(input=='e') {
                        //     usr_cmd_.turnning_vel_scale+=AXIS_STEP;
                        //     turnning_time_record = current_time;
                        // }

                        // 新逻辑：按键后直接给固定满速并保持
                        if(input=='w') {
                            usr_cmd_.forward_vel_scale = kDirectFullSpeed;
                            forward_time_record = current_time;
                        }  
                        else if(input=='s') {
                            usr_cmd_.forward_vel_scale = -kDirectFullSpeed;
                            forward_time_record = current_time;
                        }

                        if(input=='a') {
                            usr_cmd_.side_vel_scale = -kDirectFullSpeed;
                            side_time_record = current_time;
                        }
                        else if(input=='d') {
                            usr_cmd_.side_vel_scale = kDirectFullSpeed;
                            side_time_record = current_time;
                        }
                        
                        if(input=='q') {
                            usr_cmd_.turnning_vel_scale = -kDirectFullSpeed;
                            turnning_time_record = current_time;
                        }
                        else if(input=='e') {
                            usr_cmd_.turnning_vel_scale = kDirectFullSpeed;
                            turnning_time_record = current_time;
                        }

                        if(input=='w' || input=='s' || input=='a' ||
                           input=='d' || input=='q' || input=='e') {
                            std::cout << "[键盘] 当前速度命令: forward="
                                      << usr_cmd_.forward_vel_scale
                                      << ", side=" << usr_cmd_.side_vel_scale
                                      << ", turn=" << usr_cmd_.turnning_vel_scale
                                      << "\n";
                            fflush(stdout);
                        }

                        // 按 'x' 从 RL 控制回到 0 位
                        if(input=='x'){
                            usr_cmd_.target_mode = -1;   // 清除残留 target_mode，防止回 idle 后重新站起
                            usr_cmd_.sitdown_trigger = true;
                            std::cout << "[键盘] 触发 SitDown: 回 0 位\n";
                            fflush(stdout);
                        }

                        // 新逻辑：手动清零（急停）
                        if(input==' '){
                            usr_cmd_.forward_vel_scale = 0.f;
                            usr_cmd_.side_vel_scale = 0.f;
                            usr_cmd_.turnning_vel_scale = 0.f;
                            forward_time_record = current_time;
                            side_time_record = current_time;
                            turnning_time_record = current_time;
                            std::cout << "[键盘] 速度命令已清零\n";
                            fflush(stdout);
                        }
                    break;
                    default:
                        break;
                }
            }
            // 旧逻辑（保留）：超时回零仅在有按键输入时才会执行（阻塞读下会失效）
            // if(current_time - forward_time_record > 300.) usr_cmd_.forward_vel_scale = 0;
            // if(current_time - side_time_record > 300.) usr_cmd_.side_vel_scale = 0;
            // if(current_time - turnning_time_record > 300.) usr_cmd_.turnning_vel_scale = 0;

            // 新逻辑：每个周期都执行超时回零与限幅，保证松手后能回零
            {
                std::lock_guard<std::mutex> lock(mtx_);
                // 旧逻辑（保留）：
                // if(current_time - forward_time_record > 300.) usr_cmd_.forward_vel_scale = 0;
                // if(current_time - side_time_record > 300.) usr_cmd_.side_vel_scale = 0;
                // if(current_time - turnning_time_record > 300.) usr_cmd_.turnning_vel_scale = 0;
                // 旧逻辑（保留）：自动回零
                // if(current_time - forward_time_record > kCommandHoldMs) usr_cmd_.forward_vel_scale = 0;
                // if(current_time - side_time_record > kCommandHoldMs) usr_cmd_.side_vel_scale = 0;
                // if(current_time - turnning_time_record > kCommandHoldMs) usr_cmd_.turnning_vel_scale = 0;
                if(kEnableCommandTimeout){
                    const double hold_ms = GetCommandHoldMs();
                    if(current_time - forward_time_record > hold_ms) usr_cmd_.forward_vel_scale = 0;
                    if(current_time - side_time_record > hold_ms) usr_cmd_.side_vel_scale = 0;
                    if(current_time - turnning_time_record > hold_ms) usr_cmd_.turnning_vel_scale = 0;
                }

                ClipNumber(usr_cmd_.forward_vel_scale, -1., 1.);
                ClipNumber(usr_cmd_.side_vel_scale, -1., 1.);
                ClipNumber(usr_cmd_.turnning_vel_scale, -1., 1.);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if(old_flags >= 0){
            fcntl(STDIN_FILENO, F_SETFL, old_flags);
        }
        if(terminal_configured){
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        }
    }

};




#endif




// #ifndef KEYBOARD_INTERFACE_HPP_
// #define KEYBOARD_INTERFACE_HPP_

// #include "user_command_interface.h"
// #include "custom_types.h"
// #include <cstdio>
// #include <functional>
// #include <termios.h>

// #include <fcntl.h>      // shm_open
// #include <sys/mman.h>   // mmap
// #include <unistd.h>     // close
// #include <cstring>
// #include <string>
// #include <iostream>


// #define AXIS_STEP 0.1

// using namespace interface;
// using namespace types;

// class ShmKeyboardReader {
//     public:
//         ShmKeyboardReader(const std::string& name, size_t size = 256)
//             : shm_name_(name), shm_size_(size) {
    
//             std::string full_name = "/" + shm_name_;
//             shm_fd_ = shm_open(full_name.c_str(), O_RDONLY, 0666);
//             if (shm_fd_ < 0) {
//                 perror("shm_open failed");
//                 return;
//             }
    
//             shm_ptr_ = mmap(nullptr, shm_size_, PROT_READ, MAP_SHARED, shm_fd_, 0);
//             if (shm_ptr_ == MAP_FAILED) {
//                 perror("mmap failed");
//                 shm_ptr_ = nullptr;
//             }
    
//             std::cout << "Connected to SHM: " << shm_name_ << std::endl;
//         }
    
//         ~ShmKeyboardReader() {
//             if (shm_ptr_) munmap(shm_ptr_, shm_size_);
//             if (shm_fd_ >= 0) close(shm_fd_);
//         }
    
//         bool ReadChar(char &out_char) {
//             if (!shm_ptr_) return false;
    
//             char buf[256];
//             std::memcpy(buf, shm_ptr_, shm_size_);
//             buf[shm_size_ - 1] = '\0';
    
//             std::string msg(buf);
//             if (msg.empty() || msg == last_msg_)
//                 return false;
    
//             last_msg_ = msg;
    
//             // msg 形如 "w:down"
//             out_char = msg[0];
//             return true;
//         }
    
//     private:
//         std::string shm_name_;
//         size_t shm_size_;
//         int shm_fd_ = -1;
//         void* shm_ptr_ = nullptr;
//         std::string last_msg_;
//     };











// class KeyboardInterface : public UserCommandInterface
// {
// private:
//     ShmKeyboardReader shm_reader_{"ros_keyboard_shm"};
//     UserCommand usr_cmd_;
//     // MotionStateFeedback msfb_;
//     bool start_thread_flag_;
//     std::thread kb_thread_;
//     // std::mutex mtx_;  //  保护 usr_cmd_ 和 msfb_
    
//     void ClipNumber(float &num, float low, float up){
//         if(low > up) std::cerr << "error clip" << std::endl;
//         if(num < low) num = low;
//         if(num > up) num = up;
//     }

//     double GetCurrentTimeStamp(){
//         static timespec startup_timestamp;
//         timespec now_timestamp;
//         if (startup_timestamp.tv_sec + startup_timestamp.tv_nsec == 0) {
//             clock_gettime(CLOCK_MONOTONIC,&startup_timestamp);
//         }
//         clock_gettime(CLOCK_MONOTONIC,&now_timestamp);
//         return (now_timestamp.tv_sec-startup_timestamp.tv_sec)*1e3 
//             + (now_timestamp.tv_nsec-startup_timestamp.tv_nsec)/1e6;
//     }

// public:
//     KeyboardInterface(){
//         std::memset(&usr_cmd_, 0, sizeof(usr_cmd_));
//         std::cout << "Using Keyboard Command Interface" << std::endl;
//     }
//     ~KeyboardInterface(){}

//     virtual void Start(){
//         start_thread_flag_ = true;
//         kb_thread_ = std::thread(std::bind(&KeyboardInterface::Run, this));
//     }
//     virtual void Stop(){
//         start_thread_flag_ = false;
//     }
//     virtual UserCommand GetUserCommand() override {
//         // std::lock_guard<std::mutex> lock(mtx_);
//         return usr_cmd_;
//     }

//     virtual void SetMotionStateFeedback(const MotionStateFeedback& msfb){
//         // std::lock_guard<std::mutex> lock(mtx_);
//         msfb_ = msfb;
//     }


//     void Run(){
//         struct termios oldt, newt;
//         tcgetattr(STDIN_FILENO, &oldt);
//         newt = oldt;
//         newt.c_lflag &= ~(ICANON | ECHO);
//         tcsetattr(STDIN_FILENO, TCSANOW, &newt);

//         char input;
//         double forward_time_record = GetCurrentTimeStamp();
//         double side_time_record = GetCurrentTimeStamp();
//         double turnning_time_record = GetCurrentTimeStamp();
//         std::cout << "Start Keyboard Listening" << std::endl;
//         while (start_thread_flag_) {
//             // std::cout << "time: " << current_time << " " << forward_time_record << std::endl;
//             std::cout << "[ROS2msgs] Running..." << std::endl;
//             char input;
//             bool got_input = shm_reader_.ReadChar(input);
//             if(got_input){
//                 double current_time = GetCurrentTimeStamp();
//                 // std::lock_guard<std::mutex> lock(mtx_);  // 修改 usr_cmd_ 和读取 msfb_

//                 std::cout << "input: " << input << std::endl;
//                 if(input == 'r'){
//                     usr_cmd_.target_mode = int(RobotMotionState::JointDamping);
//                 }
//                 // usr_cmd_.down_shift = false;
//                 // usr_cmd_.up_shift = false;
//                 switch(msfb_.current_state) {
//                     case RobotMotionState::WaitingForStand:
//                         if(input=='z'){
//                             usr_cmd_.target_mode = int(RobotMotionState::StandingUp);
//                         }
//                     break;
//                     case RobotMotionState::StandingUp:
//                         if(input=='c'){
//                             usr_cmd_.target_mode = int(RobotMotionState::RLControlMode);
//                         }
//                     break;
//                     case RobotMotionState::RLControlMode:
//                         if(input=='w') {
//                             usr_cmd_.forward_vel_scale+=AXIS_STEP;
//                             forward_time_record = current_time;
//                         }  
//                         else if(input=='s') {
//                             usr_cmd_.forward_vel_scale-=AXIS_STEP;
//                             forward_time_record = current_time;
//                         }

//                         if(input=='a') {
//                             usr_cmd_.side_vel_scale+=AXIS_STEP;
//                             side_time_record = current_time;
//                         }
//                         else if(input=='d') {
//                             usr_cmd_.side_vel_scale-=AXIS_STEP;
//                             side_time_record = current_time;
//                         }
                        
//                         if(input=='q') {
//                             usr_cmd_.turnning_vel_scale+=AXIS_STEP;
//                             turnning_time_record = current_time;
//                         }
//                         else if(input=='e') {
//                             usr_cmd_.turnning_vel_scale-=AXIS_STEP;
//                             turnning_time_record = current_time;
//                         }


//                         if(current_time - forward_time_record > 300.) usr_cmd_.forward_vel_scale = 0;
//                         if(current_time - side_time_record > 300.) usr_cmd_.side_vel_scale = 0;
//                         if(current_time - turnning_time_record > 300.) usr_cmd_.turnning_vel_scale = 0;

//                         ClipNumber(usr_cmd_.forward_vel_scale, -1., 1.);
//                         ClipNumber(usr_cmd_.side_vel_scale, -1., 1.);
//                         ClipNumber(usr_cmd_.turnning_vel_scale, -1., 1.);
//                     break;
//                     default:
//                         break;
//                 }
//             }
//             // std::this_thread::sleep_for(std::chrono::microseconds(100));
//         }
//         tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
//     }

// };




//#endif
