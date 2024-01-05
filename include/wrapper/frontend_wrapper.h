/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-04 18:48:22
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-05 20:21:47
 */
#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "modules/frontend.h"
#include "avia_process.h"

namespace Wrapper {

    enum LIDAR_TYPE {
        AVIA = 0,
    };

    class FrontEndWrapper {
        private:
            ros::Subscriber cloud_sub;
            ros::Subscriber imu_sub;
            ros::Publisher curr_cloud_pub;

            std::shared_ptr<AVIAProcess> lidar_process_ptr;
            IESKFSlam::FrontEnd::Ptr front_end_ptr;

            IESKFSlam::PCLPointCloud curr_cloud;
            Eigen::Quaterniond curr_q;
            Eigen::Vector3d curr_t;

            void lidarCloudMsgCallback(const sensor_msgs::PointCloud2Ptr & msg);
            void imuMsgCallback(const sensor_msgs::ImuPtr & msg);
            void publishMsg();

            void run();

        public:
            FrontEndWrapper(ros::NodeHandle & nh);
            ~FrontEndWrapper();
    };

} // namespace Wrapper