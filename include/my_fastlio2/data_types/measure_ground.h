/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-05 18:47:04
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-05 18:59:05
 */
#pragma once
#include <deque>
#include "data_types/imu.h"
#include "data_types/pointcloud.h"

namespace IESKFSlam {

    struct MeasureGroup
    {
        double lidar_begin_time;
        double lidar_end_time;
        PointCloud cloud;         // 为什么不用ptr, 因为定义中有 PCLPointCloudPtr cloud_ptr;
        std::deque<IMU> imus;
    };

} // namespace IESKFSlam