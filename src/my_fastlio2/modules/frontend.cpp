/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-04 16:56:56
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-05 20:54:44
 */
#include "modules/frontend.h"

namespace IESKFSlam {

    FrontEnd::FrontEnd(const std::string & config_path, const std::string & prefix)
                : BaseModule(config_path, prefix, "FrontEnd Module") {
        state_manage_ptr = std::make_shared<StateManager>(config_path, "state_manager");
        map_manage_ptr   = std::make_shared<MapManager>(config_path, "map_manager");
    }
    FrontEnd::~FrontEnd() {}

    void FrontEnd::addImu(const IMU & imu) {
        imu_deque.push_back(imu);
    }

    void FrontEnd::addPointCloud(const PointCloud & pointcloud) {
        pointcloud_deque.push_back(pointcloud);
        std::cout << "receive cloud" << std::endl;
    }

    bool FrontEnd::track() {
        MeasureGroup mg;
        if(syncMeasureGroup(mg)) {
            if(!imu_inited) {
                map_manage_ptr->reset();
                map_manage_ptr->addScan(mg.cloud.cloud_ptr, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
                initState(mg);
                return false;
            }
            std::cout << mg.imus.size() << " Scale: " << imu_scale << std::endl;
            return true;
        }

        return true;
    }

    const PCLPointCloud & FrontEnd::readCurrentPointCloud() {
        return current_pointcloud;
    }


    bool FrontEnd::syncMeasureGroup(MeasureGroup & mg) {
        mg.imus.clear();
        mg.cloud.cloud_ptr->clear();
        if(pointcloud_deque.empty() || imu_deque.empty()) return false;

        // 选择一帧 lidar 之间的 imu 数据
        double imu_end_time = imu_deque.back().timestamp.sec();
        double imu_start_time = imu_deque.front().timestamp.sec();
        double cloud_start_time = pointcloud_deque.front().timestamp.sec();
        double cloud_end_time = pointcloud_deque.front().cloud_ptr->points.back().offset_time / 1e9 + cloud_start_time;

        if (imu_end_time < cloud_end_time) return false;   // imu 没有覆盖到当前 lidar 帧结尾
        if (cloud_end_time < imu_start_time) {             // imu 和当前帧 lidar 没有关系, lidar 帧太老了
            pointcloud_deque.pop_front();
            return false;
        }

        mg.cloud = pointcloud_deque.front();
        pointcloud_deque.pop_front();
        mg.lidar_begin_time = cloud_start_time;
        mg.lidar_end_time   = cloud_end_time;

        while (!imu_deque.empty()) {
            if (imu_deque.front().timestamp.sec() < mg.lidar_begin_time) {
                mg.imus.push_back(imu_deque.front());
                imu_deque.pop_front();
            } else {
                break;
            }
        }

        // todo imu 数据有效性检验，有待提升
        if (mg.imus.size() <= 5) return false; // 如果 imu 为 100Hz, 那么一帧点云应该对应 10 个 imu 数据, imu 数据太少则不用

        return true;
    }

    void FrontEnd::initState(MeasureGroup & mg) {
        int imu_count = 0;
        Eigen::Vector3d mean_acc{0, 0, 0};
        Eigen::Vector3d mean_gyr{0, 0, 0};
        auto & state_manage = *state_manage_ptr;

        if (imu_inited) return ;

        for (size_t i = 0; i < mg.imus.size(); ++i) {
            imu_count++;
            mean_acc += mg.imus[i].acceleration;
            mean_gyr += mg.imus[i].gyroscope;
        }

        if(imu_count >= 5) {
            mean_acc /= double(imu_count);
            mean_gyr /= double(imu_count);

            auto system_state = state_manage.get_state();
            system_state.bg = mean_gyr;
            imu_scale = GRAVITY / mean_acc.norm();
            system_state.gravity = - mean_acc / imu_scale; // todo 这个负号是为什么？ RTG 上的 imu 应该怎么修改这个？
            state_manage.set_state(system_state);
            imu_inited = true;
        }

        return ;
    }

} // namespace IESKFSlam