/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-04 11:22:43
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-04 20:07:52
 */
#pragma once
#include <Eigen/Dense>
#include <data_types/timestamp.h>

namespace IESKFSlam {

    struct Pose {
        TimeStamp timestamp;
        Eigen::Quaterniond quaterniond;
        Eigen::Vector3d position;
    };

} // namespace IESKFSlam