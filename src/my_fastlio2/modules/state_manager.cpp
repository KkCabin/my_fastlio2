/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-05 17:50:15
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-05 20:29:49
 */
#include "modules/state_manager.h"

namespace IESKFSlam {

    StateManager::StateManager(const std::string & config_path, const std::string & prefix)
            : BaseModule(config_path, prefix, "StateManager") {}
    StateManager::~StateManager() {}

    void StateManager::predict(const IMU & imu, double dt) {

    }

    bool StateManager::update() {

    }

    const StateManager::State18 & StateManager::get_state() {
        return system_state;
    }

    void StateManager::set_state(const StateManager::State18 & state_in) {
        system_state = state_in;
    }

} // namespace IESKFSlam