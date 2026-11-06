/**
 * @file types.h
 * @brief
 * @author Liuzhao Li (liliuzhao@jushenzhiren.com)
 * @version 1.0
 * @date 2025-07-31
 * @copyright Copyright (C) 2025 具身智人(北京)科技有限公司
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
namespace robot::slam
{
    using Vec2i = Eigen::Vector2i;
    using Vec3i = Eigen::Vector3i;

    using Vec2d  = Eigen::Vector2d;
    using Vec2f  = Eigen::Vector2f;
    using Vec3d  = Eigen::Vector3d;
    using Vec3f  = Eigen::Vector3f;
    using Vec4f  = Eigen::Matrix<float, 4, 1>;
    using Vec4d  = Eigen::Matrix<double, 4, 1>;
    using Vec5d  = Eigen::Matrix<double, 5, 1>;
    using Vec5f  = Eigen::Matrix<float, 5, 1>;
    using Vec6d  = Eigen::Matrix<double, 6, 1>;
    using Vec6f  = Eigen::Matrix<float, 6, 1>;
    using Vec7f  = Eigen::Matrix<float, 7, 1>;
    using Vec15d = Eigen::Matrix<double, 15, 15>;

    using Mat1d  = Eigen::Matrix<double, 1, 1>;
    using Mat3d  = Eigen::Matrix3d;
    using Mat3f  = Eigen::Matrix3f;
    using Mat4d  = Eigen::Matrix4d;
    using Mat4f  = Eigen::Matrix4f;
    using Mat5d  = Eigen::Matrix<double, 5, 5>;
    using Mat5f  = Eigen::Matrix<float, 5, 5>;
    using Mat6d  = Eigen::Matrix<double, 6, 6>;
    using Mat6f  = Eigen::Matrix<float, 6, 6>;
    using Mat15d = Eigen::Matrix<double, 15, 15>;

    using Quatd = Eigen::Quaterniond;
    using Quatf = Eigen::Quaternionf;

    using VV3D = std::vector<Vec3d, Eigen::aligned_allocator<Vec3d>>;
    using VV3F = std::vector<Vec3f, Eigen::aligned_allocator<Vec3f>>;
    using VV4F = std::vector<Vec4f, Eigen::aligned_allocator<Vec4f>>;
    using VV4D = std::vector<Vec4d, Eigen::aligned_allocator<Vec4d>>;
    using VV5F = std::vector<Vec5f, Eigen::aligned_allocator<Vec5f>>;
    using VV5D = std::vector<Vec5d, Eigen::aligned_allocator<Vec5d>>;

    const Mat3d Eye3d = Mat3d::Identity();
    const Mat3f Eye3f = Mat3f::Identity();
    const Vec3d Zero3d(0, 0, 0);
    const Vec3f Zero3f(0, 0, 0);

    template <int Rows, int Cols>
    using MD = Eigen::Matrix<double, Rows, Cols>;

    template <int Rows>
    using VD = Eigen::Matrix<double, Rows, 1>;

    template <int Rows, int Cols>
    using MF = Eigen::Matrix<float, Rows, Cols>;

    template <int Rows>
    using VF = Eigen::Matrix<float, Rows, 1>;

    using PointType      = pcl::PointXYZINormal;
    using PointCloudType = pcl::PointCloud<PointType>;
    using CloudPtr       = PointCloudType::Ptr;
    using PointVector    = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
}  // namespace robot::slam