/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-04 16:14:14
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-04 21:40:35
 */
#pragma once
#include <deque>
#include "modules/base_module.h"
#include "data_types/imu.h"
#include "data_types/base_type.h"
#include "data_types/pose.h"

namespace IESKFSlam {

    class FrontEnd : private BaseModule {
        public:
            using Ptr = std::shared_ptr<FrontEnd>;
        private:
            std::deque<IMU> imu_deque;
            std::deque<PointCloud> pointcloud_deque;
            std::deque<Pose> pose_deque;
            PCLPointCloud current_pointcloud;
        public:
            FrontEnd(const std::string & config_path, const std::string & prefix);
            ~FrontEnd();
            // 向前端添加数据
            void addImu(const IMU & imu);
            void addPointCloud(const PointCloud & pointcloud);
            void addPose(const Pose & pose);
            // 跟踪
            bool track();
            // 当前帧点云读取
            const PCLPointCloud & readCurrentPointCloud();
    };

} // namespace IESKFSlam