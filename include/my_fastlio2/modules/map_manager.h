/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-05 16:59:45
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-05 20:29:28
 */
#pragma once
#include <Eigen/Dense>
#include "modules/base_module.h"
#include "data_types/pointcloud.h"

namespace IESKFSlam {

    class MapManager : private BaseModule {
        private:
            PCLPointCloudPtr local_map_ptr;
        public:
            MapManager(const std::string & config_path, const std::string & prefix);
            ~MapManager();
            void reset();
            void addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond & q, const Eigen::Vector3d & t);
            PCLPointCloudPtr getLocalMap();
    };

} // namespace IESKFSlam