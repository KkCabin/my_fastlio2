/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-05 18:17:08
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-05 20:29:25
 */
#include <pcl/common/transforms.h>
#include "modules/map_manager.h"
#include "math/math.hpp"

namespace IESKFSlam {

    MapManager::MapManager(const std::string & config_path, const std::string & prefix)
                    : BaseModule(config_path, prefix, "MapManager") {
        local_map_ptr = pcl::make_shared<PCLPointCloud>();
    }
    MapManager::~MapManager() {}

    void MapManager::reset() {
        local_map_ptr->clear();
    }

    void MapManager::addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond & q, const Eigen::Vector3d & t) {
        PCLPointCloud scan;
        pcl::transformPointCloud(*curr_scan, scan, compositeTransform(q, t).cast<float>());
        *local_map_ptr += scan;
    }

    PCLPointCloudPtr MapManager::getLocalMap() {
        return local_map_ptr;
    }


} // namespace IESKFSlam