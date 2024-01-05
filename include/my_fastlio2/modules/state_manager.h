/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-05 16:59:52
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-05 18:02:46
 */
#pragma once
#include <Eigen/Dense>
#include "modules/base_module.h"
#include "data_types/imu.h"

namespace IESKFSlam {

    class StateManager : private BaseModule {
        public:
            struct State18 {
                Eigen::Quaterniond quaterniond; // 旋转
                Eigen::Vector3d position;       // 位移
                Eigen::Vector3d velocity;       // 速度
                Eigen::Vector3d bg;             // 角速度偏移
                Eigen::Vector3d ba;             // 加速度偏移
                Eigen::Vector3d gravity;        // 位移
                State18() {
                    quaterniond = Eigen::Quaterniond::Identity();
                    position    = Eigen::Vector3d::Zero();
                    velocity    = Eigen::Vector3d::Zero();
                    bg          = Eigen::Vector3d::Zero();
                    ba          = Eigen::Vector3d::Zero();
                    gravity     = Eigen::Vector3d::Zero();
                }
            };
        private:
            State18 system_state;
        public:
            StateManager(const std::string & config_path, const std::string & prefix);
            ~StateManager();
            void predict(const IMU & imu, double dt);
            bool update();
            const State18 & get_state();
            void set_state(const State18 & state_in);
    };

} // namespace IESKFSlam