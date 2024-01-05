/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-04 16:14:14
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-05 20:28:12
 */
#pragma once
#include <deque>
#include "data_types/imu.h"
#include "data_types/base_type.h"
#include "data_types/pose.h"
#include "data_types/measure_ground.h"
#include "modules/base_module.h"
#include "modules/state_manager.h"
#include "modules/map_manager.h"


namespace IESKFSlam {

    class FrontEnd : private BaseModule {
        public:
            using Ptr = std::shared_ptr<FrontEnd>;
        private:
            std::deque<IMU> imu_deque;
            std::deque<PointCloud> pointcloud_deque;
            PCLPointCloud current_pointcloud;
            std::shared_ptr<StateManager> state_manage_ptr;
            std::shared_ptr<MapManager> map_manage_ptr;
            bool imu_inited = false;
            double imu_scale = 1.0;
        public:
            FrontEnd(const std::string & config_path, const std::string & prefix);
            ~FrontEnd();
            // 向前端添加数据
            void addImu(const IMU & imu);
            void addPointCloud(const PointCloud & pointcloud);

            // 跟踪
            bool track();
            // 当前帧点云读取
            const PCLPointCloud & readCurrentPointCloud();
            // 数据同步
            bool syncMeasureGroup(MeasureGroup & mg);
            // 状态初始化
            void initState(MeasureGroup & mg);
    };

} // namespace IESKFSlam