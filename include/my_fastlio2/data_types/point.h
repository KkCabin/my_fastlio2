/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-03 19:33:35
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-04 22:04:49
 */
#pragma once                      // 防止头文件被重复包含,代替传统的 include guard（ifndef/define/endif）
#define PCL_NO_PRECOMPILE         // PCL 将不再使用预编译头文件, 而是采用按需包含头文件的方式进行编译

#include <pcl/impl/pcl_base.hpp>  // 缺少这个头文件，后面对自定义格式的点云进行滤波编译无法通过
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>

namespace IESKFSlam {
    struct EIGEN_ALIGN16 Point {           // 强制SSE填充以获得正确的内存对齐
        PCL_ADD_POINT4D;                   // 添加pcl里xyz+padding
        float intensity;
        std::uint32_t offset_time;
        std::int32_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW    // 确保新的分配器内存是对齐的
    };

} // namespace IESKFSlam

// 注册点类型宏(需要在命名空间外面进行)
POINT_CLOUD_REGISTER_POINT_STRUCT(IESKFSlam::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, offset_time, offset_time)
    (std::int32_t, ring, ring)
)

namespace avia_ros {

    struct EIGEN_ALIGN16 Point {           // 强制SSE填充以获得正确的内存对齐
        PCL_ADD_POINT4D;                   // 添加pcl里xyz+padding
        float intensity;
        std::uint32_t offset_time;
        std::uint8_t line;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW    // 确保新的分配器内存是对齐的
    };

} // namespace avia_ros

// 注册点类型宏(需要在命名空间外面进行)
POINT_CLOUD_REGISTER_POINT_STRUCT(avia_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, offset_time, offset_time)
    (std::uint8_t, line, line)
)


