/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-04 11:50:58
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-04 21:17:04
 */
#pragma once
#include <Eigen/Dense>
#include <data_types/timestamp.h>

namespace IESKFSlam {

    class IMU {
        public:
            TimeStamp timestamp;
            Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();   // 加速度计，测量的是线加速度
            Eigen::Vector3d gyroscope = Eigen::Vector3d::Zero();    // 陀螺仪，测量的角速度

        public:
            void clear() {
                acceleration = Eigen::Vector3d::Zero();
                gyroscope = Eigen::Vector3d::Zero();
                timestamp = 0;
            }
            IMU operator + (const IMU& imu) {
                IMU res;
                res.acceleration = this->acceleration + imu.acceleration;
                res.gyroscope = this->gyroscope + imu.gyroscope;
                return res;
            }
            IMU operator * (double k) {
                IMU res;
                res.acceleration = this->acceleration * k;
                res.gyroscope = this->gyroscope * k;
                return res;
            }
            IMU operator / (double k) {
                IMU res;
                res.acceleration = this->acceleration / k;
                res.gyroscope = this->gyroscope / k;
                return res;
            }
            // C++输出运算符<<为什么要声明为友元？ https://blog.csdn.net/qq_52698632/article/details/124269628
            friend std::ostream & operator << (std::ostream & ostream, const IMU & imu) {
                ostream << "imu time: "         << imu.timestamp.sec()
                        << "imu acceleration: " << imu.acceleration.transpose()
                        << "imu gyroscope: "    << imu.gyroscope.transpose();
                return ostream;
            }
    };

} // namespace IESKFSlam