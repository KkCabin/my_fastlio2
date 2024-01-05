/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-04 18:55:10
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-05 20:21:42
 */
#include <string.h>
#include "frontend_wrapper.h"

namespace Wrapper {

    FrontEndWrapper::FrontEndWrapper(ros::NodeHandle & nh) {
        std::string config_path, lidar_topic, imu_topic;
        nh.param<std::string>("wrapper/config_file_name", config_path, "");
        nh.param<std::string>("wrapper/lidar_topic", lidar_topic, "/lidar");
        nh.param<std::string>("wrapper/imu_topic", imu_topic, "/imu");

        front_end_ptr = std::make_shared<IESKFSlam::FrontEnd>(CONFIG_DIR + config_path, "front_end");

        cloud_sub = nh.subscribe(lidar_topic, 100, &FrontEndWrapper::lidarCloudMsgCallback, this);
        imu_sub = nh.subscribe(imu_topic, 100, &FrontEndWrapper::imuMsgCallback, this);

        int lidar_type = 0;
        nh.param<int>("wrapper/lidar_type", lidar_type, AVIA);
        if(lidar_type == AVIA) {
            lidar_process_ptr = std::make_shared<AVIAProcess>();
        } else {
            std::cout << "unsupport lidar type" << std::endl;
            std::exit(100);
        }

        curr_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("curr_cloud", 100);

        run();
    }

    FrontEndWrapper::~FrontEndWrapper() {}

    void FrontEndWrapper::lidarCloudMsgCallback(const sensor_msgs::PointCloud2Ptr & msg) {
        IESKFSlam::PointCloud cloud;
        lidar_process_ptr->process(*msg, cloud);
        front_end_ptr->addPointCloud(cloud);
    }

    void FrontEndWrapper::imuMsgCallback(const sensor_msgs::ImuPtr & msg) {
        IESKFSlam::IMU imu;
        imu.timestamp.fromNsec(msg->header.stamp.toNSec());
        imu.acceleration = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
        imu.gyroscope = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
        front_end_ptr->addImu(imu);
    }

    void FrontEndWrapper::run() {
        ros::Rate rate(500);
        while(ros::ok()) {
            rate.sleep();
            ros::spinOnce();
            if(front_end_ptr->track()) {
                publishMsg();
            }
        }
    }

    void FrontEndWrapper::publishMsg() {
        auto cloud = front_end_ptr->readCurrentPointCloud();
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = "map";
        curr_cloud_pub.publish(msg);
    }

} // namespace Wrapper
