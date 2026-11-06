/**
 * @file const_value.h
 * @brief
 * @author Liuzhao Li (liliuzhao@jushenzhiren.com)
 * @version 1.0
 * @date 2025-07-31
 * @copyright Copyright (C) 2025 具身智人(北京)科技有限公司
 */

#pragma once

#define MD(a, b) Eigen::Matrix<double, (a), (b)>
#define VD(a)    Eigen::Matrix<double, (a), 1>
#define MF(a, b) Eigen::Matrix<float, (a), (b)>
#define VF(a)    Eigen::Matrix<float, (a), 1>

#define VEC_FROM_ARRAY(v)       v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v)       v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define CONSTRAIN(v, min, max)  ((v > min) ? ((v < max) ? v : max) : min)
#define ARRAY_FROM_EIGEN(mat)   mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat) vector<decltype(mat)::Scalar>(mat.data(), mat.data() + mat.rows() * mat.cols())
#define DEBUG_FILE_DIR(name)    (string(string(ROOT_DIR) + "Log/" + name))

namespace robot::slam
{
    constexpr double PI_M             = 3.14159265358;
    constexpr double G_m_s2           = 9.81;  // Gravity const in GuangDong/China
    constexpr int    DIM_STATE        = 18;    // Dimension of states (Let Dim(SO(3)) = 3)
    constexpr int    DIM_PROC_N       = 12;    // Dimension of process noise (Let Dim(SO(3)) = 3)
    constexpr double CUBE_LEN         = 6.0;
    constexpr int    LIDAR_SP_LEN     = 2;
    constexpr double INIT_COV         = 1;
    constexpr int    NUM_MATCH_POINTS = 5;
    constexpr int    MAX_MEAS_DIM     = 10000;

    constexpr int MAX_INI_COUNT = 20;

    constexpr float  INIT_TIME       = 0.1;
    constexpr double LASER_POINT_COV = 0.001;
    constexpr int    MAXN            = 720000;
    constexpr int    PUBFRAME_PERIOD = 20;

}  // namespace robot::slam