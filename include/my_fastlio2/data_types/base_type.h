/*
 * @Descripttion:
 * @Author: Wangchao Yu
 * @Date: 2024-01-04 15:13:29
 * @LastEditors: Wangchao Yu
 * @LastEditTime: 2024-01-04 22:28:48
 */
#pragma once
// ! pointcloud.h 这个引用要在 voxel_grid 滤波的前面，因为 #define PCL_NO_PRECOMPILE 要在前面
// 参考链接 https://blog.csdn.net/fish_ttzz/article/details/121977749
#include "data_types/pointcloud.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace IESKFSlam
{

    // 体素滤波器
    using VoxelFilter = pcl::VoxelGrid<Point>;
    // KDTree
    // todo 后面替换成 ikdtree
    using KDTree = pcl::KdTreeFLANN<Point>;
    using KDTreePtr = KDTree::Ptr;
    // 定义重力常量
    const double GRAVITY = 9.81;

} // namespace IESKFSlam