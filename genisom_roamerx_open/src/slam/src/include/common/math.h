/**
 * @file math.h
 * @brief
 * @author Liuzhao Li (liliuzhao@jushenzhiren.com)
 * @version 1.0
 * @date 2025-07-31
 * @copyright Copyright (C) 2025 具身智人(北京)科技有限公司
 */

#pragma once
#include "common/const_value.h"
#include "common/custom_type.h"
#include "common/types.h"

#include <filesystem>
namespace robot::slam
{

    inline bool checkDirExist(const std::string& map_dir)
    {
        if (!std::filesystem::exists(std::filesystem::path(map_dir).parent_path()))
        {
            if (!std::filesystem::create_directories(std::filesystem::path(map_dir).parent_path()))
            {
                return false;
            }
            if (!std::filesystem::create_directory(map_dir))
            {
                return false;
            }
            return true;
        }
        else
        {
            if (!std::filesystem::exists(map_dir))
            {
                if (!std::filesystem::create_directory(map_dir))
                {
                    return false;
                }
            }
            return true;
        }
    }

    template <typename T>
    T rad2deg(T radians)
    {
        return radians * 180.0 / PI_M;
    }

    template <typename T>
    T deg2rad(T degrees)
    {
        return degrees * PI_M / 180.0;
    }

    template <typename T>
    auto set_pose6d(const double t, const Eigen::Matrix<T, 3, 1>& a, const Eigen::Matrix<T, 3, 1>& g, const Eigen::Matrix<T, 3, 1>& v,
        const Eigen::Matrix<T, 3, 1>& p, const Eigen::Matrix<T, 3, 3>& R)
    {
        Pose6D rot_kp;
        rot_kp.offset_time = t;
        for (int i = 0; i < 3; i++)
        {
            rot_kp.acc = a;
            rot_kp.gyr = g;
            rot_kp.vel = v;
            rot_kp.pos = p;
            for (int j = 0; j < 3; j++)
                rot_kp.rot[i * 3 + j] = R(i, j);
        }
        return std::move(rot_kp);
    }

    template <typename T>
    bool esti_normvector(Eigen::Matrix<T, 3, 1>& normvec, const PointVector& point, const T& threshold, const int& point_num)
    {
        Eigen::MatrixXf A(point_num, 3);
        Eigen::MatrixXf b(point_num, 1);
        b.setOnes();
        b *= -1.0f;

        for (int j = 0; j < point_num; j++)
        {
            A(j, 0) = point[j].x;
            A(j, 1) = point[j].y;
            A(j, 2) = point[j].z;
        }
        normvec = A.colPivHouseholderQr().solve(b);

        for (int j = 0; j < point_num; j++)
        {
            if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y + normvec(2) * point[j].z + 1.0f) > threshold)
            {
                return false;
            }
        }

        normvec.normalize();
        return true;
    }

    template <typename T>
    bool esti_plane(Eigen::Matrix<T, 4, 1>& pca_result, const PointVector& point, const T& threshold)
    {
        Eigen::Matrix<T, NUM_MATCH_POINTS, 3> A;
        Eigen::Matrix<T, NUM_MATCH_POINTS, 1> b;
        A.setZero();
        b.setOnes();
        b *= -1.0f;

        for (int j = 0; j < NUM_MATCH_POINTS; j++)
        {
            A(j, 0) = point[j].x;
            A(j, 1) = point[j].y;
            A(j, 2) = point[j].z;
        }

        Eigen::Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

        T n           = normvec.norm();
        pca_result(0) = normvec(0) / n;
        pca_result(1) = normvec(1) / n;
        pca_result(2) = normvec(2) / n;
        pca_result(3) = 1.0 / n;

        for (int j = 0; j < NUM_MATCH_POINTS; j++)
        {
            if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
            {
                return false;
            }
        }
        return true;
    }

    float calc_dist(PointType p1, PointType p2);

}  // namespace robot::slam