/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-04 10:04:40
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-04 11:49:14
 */
#pragma once
#include "data_types/point.h"
#include "data_types/timestamp.h"

namespace IESKFSlam {

    using PCLPointCloud = pcl::PointCloud<Point>;
    using PCLPointCloudPtr = PCLPointCloud::Ptr;
    struct PointCloud {
        using Ptr = std::shared_ptr<PointCloud>;
        TimeStamp timestamp;
        PCLPointCloudPtr cloud_ptr;
        PointCloud() {
            cloud_ptr = pcl::make_shared<PCLPointCloud>();
        }
    };

} // namespace IESKFSlam