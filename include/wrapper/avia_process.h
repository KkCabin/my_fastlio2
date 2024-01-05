/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-04 18:16:40
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-04 20:09:26
 */
#pragma once
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "data_types/base_type.h"

namespace Wrapper {

    class AVIAProcess {
        private:
            /*data*/
        public:
            AVIAProcess(){}
            ~AVIAProcess(){}
            bool process(const sensor_msgs::PointCloud2 & msg, IESKFSlam::PointCloud & cloud) {
                pcl::PointCloud<avia_ros::Point> avia_cloud;
                pcl::fromROSMsg(msg, avia_cloud);
                cloud.cloud_ptr->clear();
                // 关于 auto && 的介绍: https://zhuanlan.zhihu.com/p/611955620
                for (auto && point : avia_cloud) {
                    IESKFSlam::Point p;
                    p.x = point.x;
                    p.y = point.y;
                    p.z = point.z;
                    p.ring = point.line;
                    p.intensity = point.intensity;
                    p.offset_time = point.offset_time;
                    cloud.cloud_ptr->push_back(p);
                }
                cloud.timestamp.fromNsec(msg.header.stamp.toNSec());
                return true;
            }
    };

} // namespace Wrapper