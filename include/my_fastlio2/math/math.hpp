/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-05 18:23:43
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-05 18:27:24
 */
#pragma once
#include <Eigen/Dense>

namespace IESKFSlam {

    Eigen::Matrix4d compositeTransform(const Eigen::Quaterniond & q, const Eigen::Vector3d & t) {
        Eigen::Matrix4d res;
        res.setIdentity();
        res.block<3, 3>(0, 0) = q.toRotationMatrix();
        res.block<3, 1>(0, 3) = t;
        return res;
    }

} // namespace IESKFSlam