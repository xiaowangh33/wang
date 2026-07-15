/**
 * @file mujoco_interface.hpp
 * @brief simulation in mujoco
 * @author Bo (Percy) Peng
 * @version 1.0
 * @date 2025-08-010
 * @copyright Copyright (c) 2025 DeepRobotics
 */


#ifndef MUJOCO_INTERFACE_HPP_
#define MUJOCO_INTERFACE_HPP_

#include "robot_interface.h"
#include "robot_model_config.hpp"
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <string>
#include <thread>
#include <iostream>
#include <cstring>
#include <random>
#include <atomic>
#include <mutex>
#include <algorithm>
#include <stdexcept>

namespace interface {

class MujocoInterface : public RobotInterface {
private:
    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;

    std::string xml_path_;
    mjvScene scene_;
    mjrContext context_;
    mjvCamera camera_;
    mjvOption opt_;
    GLFWwindow* window_ = nullptr;

    std::thread sim_thread_;
    mutable std::mutex state_mutex_;
    mutable std::mutex command_mutex_;
    int imu_acc_sensor_adr_ = -1;
    int imu_gyro_sensor_adr_ = -1;

    Vec3d omega_body_, rpy_, acc_;

    VecXd joint_pos_, joint_vel_, joint_tau_;


    double dt_ = 0.001;
    double run_time_ = 0.0;
    int run_cnt_ = 0;
    VecXd tau_ff_;

    int render_interval_ = 10;

    // Float mode: base stays in place, all joints movable (for chirp debugging in sim)
    bool float_mode_ = false;
    double init_base_qpos_[7] = {};   // x, y, z, qw, qx, qy, qz
    double init_base_qvel_[6] = {};   // vx, vy, vz, wx, wy, wz
    double float_base_z_ = 1.0;       // float height (meters)

    std::default_random_engine dre_;
    std::normal_distribution<> gyro_nd_{0, 0.0001}, rpy_nd_{0, 0.005}, acc_nd_{0, 0.0};

public:
    MujocoInterface(const std::string& robot_name,
                const std::string& xml_path,
                int dof_num = robot_model::kTotalDof)
        : RobotInterface(robot_name, dof_num), xml_path_(xml_path) {

        joint_pos_ = VecXd::Zero(dof_num_);
        joint_tau_ = VecXd::Zero(dof_num_);
        joint_vel_ = VecXd::Zero(dof_num_);
        joint_cmd_ = MatXf::Zero(dof_num_, 5);

        mjv_defaultCamera(&camera_);
        mjv_defaultOption(&opt_);
        mjv_defaultScene(&scene_);
        mjr_defaultContext(&context_);

        std::cout << "[MuJoCoInterface] Loading model: " << xml_path_ << std::endl;
        char error[1000] = "";
        model_ = mj_loadXML(xml_path_.c_str(), 0, error, 1000);
        if (!model_) {
            std::cerr << "[ERROR] Failed to load MuJoCo model: " << error << std::endl;
            exit(1);
        }
        data_ = mj_makeData(model_);
        if (!data_) {
            mj_deleteModel(model_);
            model_ = nullptr;
            throw std::runtime_error("Failed to allocate MuJoCo data");
        }
        dt_ = model_->opt.timestep;
        if (dt_ <= 0.0) {
            dt_ = robot_model::kSimulationDt;
            model_->opt.timestep = dt_;
        }
        if (model_->nq < 7 + dof_num_ || model_->nv < 6 + dof_num_) {
            mj_deleteData(data_);
            mj_deleteModel(model_);
            data_ = nullptr;
            model_ = nullptr;
            throw std::runtime_error(
                "MuJoCo model dimensions do not match the wheeled-dog free-base interface");
        }

        data_->qpos[2] = robot_model::kLieDownBaseHeight;
        for (int i = 0; i < dof_num_ && i < robot_model::kTotalDof; ++i) {
            data_->qpos[7 + i] = robot_model::kLieDownPosition[i];
        }
        mj_forward(model_, data_);

        const auto resolve_vec3_sensor = [this](const char* name) {
            const int sensor_id = mj_name2id(model_, mjOBJ_SENSOR, name);
            if (sensor_id < 0 || model_->sensor_dim[sensor_id] != 3) {
                throw std::runtime_error(
                    std::string("MuJoCo model requires a 3D sensor named '") + name + "'");
            }
            const int address = model_->sensor_adr[sensor_id];
            if (address < 0 || address + 3 > model_->nsensordata) {
                throw std::runtime_error(
                    std::string("MuJoCo sensor address is invalid for '") + name + "'");
            }
            return address;
        };
        try {
            imu_acc_sensor_adr_ = resolve_vec3_sensor("imu_acc");
            imu_gyro_sensor_adr_ = resolve_vec3_sensor("imu_gyro");
        } catch (...) {
            mj_deleteData(data_);
            mj_deleteModel(model_);
            data_ = nullptr;
            model_ = nullptr;
            throw;
        }

        // Float mode: check env var MUJOCO_FLOAT_MODE
        {
            const char* fm = std::getenv("MUJOCO_FLOAT_MODE");
            if (fm && (fm[0] == '1' || fm[0] == 'y' || fm[0] == 'Y')) {
                float_mode_ = true;
                // Read float height from env var (default 1.0m)
                {
                    const char* fh = std::getenv("MUJOCO_FLOAT_HEIGHT");
                    if (fh) float_base_z_ = std::atof(fh);
                }
                // Save initial base pose & velocity (after first forward kinematics)
                mj_forward(model_, data_);
                std::memcpy(init_base_qpos_, data_->qpos, 7 * sizeof(double));
                std::memcpy(init_base_qvel_, data_->qvel, 6 * sizeof(double));
                // Override base Z to float height
                init_base_qpos_[2] = float_base_z_;
                std::cout << "\n"
                          << "╔════════════════════════════════════════════════════╗\n"
                          << "║  MUJOCO FLOAT MODE ENABLED                        ║\n"
                          << "║  Base fixed at z=" << std::fixed << std::setprecision(1) << float_base_z_ << " m, all joints movable.     ║\n"
                          << "║  For chirp debugging only — not for dynamics.     ║\n"
                          << "╚════════════════════════════════════════════════════╝\n"
                          << std::endl;
            }
        }

        std::cout << "[MuJoCoInterface] Model loaded successfully. nq=" << model_->nq
                  << ", nv=" << model_->nv << ", nu=" << model_->nu
                  << ", timestep=" << dt_ << std::endl;
        std::cout << "[MuJoCoInterface] IMU frame: imu_site is identity-aligned with TORSO"
                  << " (+X forward, +Y left, +Z up)" << std::endl;

        // 某些模型（如 URDF 自动转换的 MJCF）可能没有 "base_link" body。
        // 若跟踪目标不存在，使用自由相机避免 mjv_updateCamera 直接报错退出。
        int base_id = mj_name2id(model_, mjOBJ_BODY, "base_link");
        if (base_id < 0) {
            base_id = mj_name2id(model_, mjOBJ_BODY, "TORSO");
        }
        if (base_id >= 0 && base_id < model_->nbody) {
            camera_.type = mjCAMERA_TRACKING;
            camera_.trackbodyid = base_id;
        } else {
            camera_.type = mjCAMERA_FREE;
            camera_.trackbodyid = 0;
            std::cout << "[MuJoCoInterface] WARNING: base body not found, fallback to free camera."
                      << std::endl;
        }
        // camera_.lookat[0] = 0.0;
        // camera_.lookat[1] = 0.0;
        // camera_.lookat[2] = 1.0;
        camera_.distance = 4.0;
        camera_.azimuth = 90.0;
        camera_.elevation = -30.0;
    }

    ~MujocoInterface() {
        Stop();
        mj_deleteData(data_);
        mj_deleteModel(model_);
        mjv_freeScene(&scene_);
        mjr_freeContext(&context_);
        if (window_) glfwDestroyWindow(window_);
        glfwTerminate();
    }

    virtual double GetInterfaceTimeStamp() override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return run_time_;
    }

    virtual VecXf GetJointPosition() override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return joint_pos_.cast<float>();
    }
    virtual VecXf GetJointVelocity() override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return joint_vel_.cast<float>();
    }
    virtual VecXf GetJointTorque() override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return joint_tau_.cast<float>();
    }
    virtual Vec3f GetImuRpy() override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return rpy_.cast<float>();
    }
    virtual Vec3f GetImuAcc() override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return acc_.cast<float>();
    }
    virtual Vec3f GetImuOmega() override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return omega_body_.cast<float>();
    }
    virtual VecXf GetContactForce() override { return VecXf::Zero(4); }

    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) override {
        std::lock_guard<std::mutex> lock(command_mutex_);
        joint_cmd_ = input;
    }

    virtual MatXf GetJointCommand() override {
        std::lock_guard<std::mutex> lock(command_mutex_);
        return joint_cmd_;
    }

    virtual void Start() override {
        if (sim_thread_.joinable()) {
            return;
        }
        start_flag_ = true;
        sim_thread_ = std::thread(std::bind(&MujocoInterface::Run, this));
    }

    virtual void Stop() override {
        start_flag_ = false;
        if (sim_thread_.joinable() && sim_thread_.get_id() != std::this_thread::get_id()) {
            sim_thread_.join();
        }
    }

private:
    void Run() {


        if (!glfwInit()) {
            std::cerr << "[ERROR] Could not initialize GLFW" << std::endl;
            start_flag_ = false;
            return;
        }

        window_ = glfwCreateWindow(1200, 900, "MuJoCo Simulation", NULL, NULL);
        if (!window_) {
            std::cerr << "[ERROR] Could not create GLFW window" << std::endl;
            glfwTerminate();
            start_flag_ = false;
            return;
        }

        glfwMakeContextCurrent(window_);
        mjv_makeScene(model_, &scene_, 2000);
        mjr_makeContext(model_, &context_, mjFONTSCALE_150);

        glfwMakeContextCurrent(window_);

        glfwSwapInterval(1);

        mj_forward(model_, data_);

        while (start_flag_ && !glfwWindowShouldClose(window_))  {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                run_time_ = data_->time;
                UpdateImu();
                UpdateJointState();
                ApplyControl();
            }

            mj_step(model_, data_);

            // Float mode: reset base to initial pose/zero velocity after each step
            if (float_mode_) {
                std::memcpy(data_->qpos, init_base_qpos_, 7 * sizeof(double));
                std::memcpy(data_->qvel, init_base_qvel_, 6 * sizeof(double));
                mj_forward(model_, data_);
            }

            if (run_cnt_ % render_interval_ == 0) {
                Render();
            }        
            // std::cout << "Rendered frame " << run_cnt_ << std::endl;

            ++run_cnt_;
            std::this_thread::sleep_for(std::chrono::microseconds(int(dt_ * 1e6)));
        }
        start_flag_ = false;
    }

    void UpdateImu() {

        double* q = data_->qpos + 3;
        double qw = q[0], qx = q[1], qy = q[2], qz = q[3];

        double roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
        const double pitch_arg = std::clamp(2 * (qw * qy - qz * qx), -1.0, 1.0);
        double pitch = asin(pitch_arg);
        double yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

        rpy_ << roll + rpy_nd_(dre_), pitch + rpy_nd_(dre_), yaw + rpy_nd_(dre_);

        acc_ = Eigen::Map<Vec3d>(data_->sensordata + imu_acc_sensor_adr_);
        omega_body_ = Eigen::Map<Vec3d>(data_->sensordata + imu_gyro_sensor_adr_);

        // std::cout << "[IMU] RPY: " << rpy_.transpose()
        //       << " | Omega: " << omega_body_.transpose()
        //       << " | Acc: " << acc_.transpose() << std::endl;
        
        
    }

    void UpdateJointState() {
        joint_pos_ = Eigen::Map<VecXd>(data_->qpos + 7, dof_num_);
        joint_vel_ = Eigen::Map<VecXd>(data_->qvel + 6, dof_num_);
        if (model_->nu >= dof_num_) {
            joint_tau_ = Eigen::Map<VecXd>(data_->actuator_force, dof_num_);
        } else {
            // 无 actuator 时，回退读取 qfrc_applied 作为“当前施加关节力矩”
            joint_tau_ = Eigen::Map<VecXd>(data_->qfrc_applied + 6, dof_num_);
        }

        // std::cout << "[JointState] pos[0:3]: " << joint_pos_.head(3).transpose()
        //       << " | vel[0:3]: " << joint_vel_.head(3).transpose()
        //       << " | tau[0:3]: " << joint_tau_.head(3).transpose() << std::endl;
        
    }

    void ApplyControl() {
        MatXf joint_cmd;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            joint_cmd = joint_cmd_;
        }
        auto kp = joint_cmd.col(0).cast<double>();
        auto q_des = joint_cmd.col(1).cast<double>();
        auto kd = joint_cmd.col(2).cast<double>();
        auto dq_des = joint_cmd.col(3).cast<double>();
        auto tau_ff = joint_cmd.col(4).cast<double>();

        VecXd tau_out = VecXd::Zero(dof_num_);
        VecXd ctrl_out = VecXd::Zero(dof_num_);
        for (int i = 0; i < dof_num_; ++i) {
            if (actuator_modes_[i] == robot_model::ActuatorMode::Velocity) {
                ctrl_out(i) = dq_des(i);
                continue;
            }
            const double tau = kp(i) * (q_des(i) - joint_pos_(i))
                             + kd(i) * (dq_des(i) - joint_vel_(i))
                             + tau_ff(i);
            const double effort_limit = (i < robot_model::kTotalDof)
                ? static_cast<double>(robot_model::kJoints[i].effort_limit_nm)
                : 1000.0;
            tau_out(i) = std::clamp(tau, -effort_limit, effort_limit);
            ctrl_out(i) = tau_out(i);
        }

        if (model_->nu >= dof_num_) {
            Eigen::Map<VecXd>(data_->ctrl, dof_num_) = ctrl_out;
        } else {
            // 无 actuator 的 URDF 转换模型：直接写入广义外力
            Eigen::Map<VecXd>(data_->qfrc_applied + 6, dof_num_) = tau_out;
        }
    }

    void Render() {
        mjv_updateScene(model_, data_, &opt_, nullptr, &camera_, mjCAT_ALL, &scene_);
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
        mjr_render(viewport, &scene_, &context_);
        glfwSwapBuffers(window_);
        glfwPollEvents();
    }
};

}  // namespace interface

#endif  // MUJOCO_INTERFACE_HPP_
