/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-04 16:56:56
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-04 22:28:53
 */
#include "modules/frontend.h"
#include <pcl/common/transforms.h>


namespace IESKFSlam {

    FrontEnd::FrontEnd(const std::string & config_path, const std::string & prefix)
                : BaseModule(config_path, prefix, "FrontEnd Module") {}
    FrontEnd::~FrontEnd() {}

    void FrontEnd::addImu(const IMU & imu) {
        imu_deque.push_back(imu);
    }

    void FrontEnd::addPointCloud(const PointCloud & pointcloud) {
        pointcloud_deque.push_back(pointcloud);
        std::cout << "receive cloud" << std::endl;
    }

    void FrontEnd::addPose(const Pose & pose) {
        pose_deque.push_back(pose);
        std::cout << "receive pose" << std::endl;
    }

    bool FrontEnd::track() {
        if(pose_deque.empty() || pointcloud_deque.empty()) {
            return false;
        }

        // 寻找同一时刻的点云和位姿
        while(!pose_deque.empty() && pose_deque.front().timestamp.nsec() < pointcloud_deque.front().timestamp.nsec()) {
            std::cout << "1" << std::endl;
            pose_deque.pop_front();
        }
        if(pose_deque.empty()) {
            return false;
        }
        while(!pointcloud_deque.empty() && pointcloud_deque.front().timestamp.nsec() < pose_deque.front().timestamp.nsec()) {
            std::cout << "2" << std::endl;
            pointcloud_deque.pop_front();
        }
        if(pointcloud_deque.empty()) {
            return false;
        }

        VoxelFilter voxel_filter;
        voxel_filter.setLeafSize(0.2, 0.2, 0.2);
        voxel_filter.setInputCloud(pointcloud_deque.front().cloud_ptr);
        voxel_filter.filter(*pointcloud_deque.front().cloud_ptr);

        Eigen::Matrix4f trans;
        trans.setIdentity();
        trans.block<3, 3>(0, 0) = pose_deque.front().quaterniond.toRotationMatrix().cast<float>();
        trans.block<3, 1>(0, 3) = pose_deque.front().position.cast<float>();
        pcl::transformPointCloud(*pointcloud_deque.front().cloud_ptr,current_pointcloud,trans);

        pointcloud_deque.pop_front();
        pose_deque.pop_front();
        return true;
    }

    const PCLPointCloud & FrontEnd::readCurrentPointCloud() {
        return current_pointcloud;
    }

} // namespace IESKFSlam