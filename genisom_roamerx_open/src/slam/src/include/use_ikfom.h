/**
 * @file use_ikfom.h
 * @brief
 * @author Liuzhao Li (liliuzhao@jushenzhiren.com)
 * @version 1.0
 * @date 2025-07-31
 * @copyright Copyright (C) 2025 具身智人(北京)科技有限公司
 */
#pragma once

#include <mtk_iekf/esekfom/esekfom.hpp>

typedef MTK::vect<3, double>             vect3;
typedef MTK::SO3<double>                 SO3;
typedef MTK::S2<double, 98090, 10000, 1> S2;
typedef MTK::vect<1, double>             vect1;
typedef MTK::vect<2, double>             vect2;

MTK_BUILD_MANIFOLD(state_ikfom,
    ((vect3, pos))((SO3, rot))((SO3, offset_R_L_I))((vect3, offset_T_L_I))((vect3, vel))((vect3, bg))((vect3, ba))((S2, grav)));

MTK_BUILD_MANIFOLD(input_ikfom, ((vect3, acc))((vect3, gyro)));

MTK_BUILD_MANIFOLD(process_noise_ikfom, ((vect3, ng))((vect3, na))((vect3, nbg))((vect3, nba)));

/**
 * @brief Process noise covariance for IMU and LiDAR.
 *
 * This function generates and returns the process noise covariance matrix used in
 * the extended error Kalman filter for IMU and LiDAR data. The noise values are predefined
 * for different components, including the gyroscope, accelerometer, and their biases.
 *
 * @return The process noise covariance matrix.
 */
MTK::get_cov<process_noise_ikfom>::type process_noise_cov();

/**
 * @brief System function for ESKF state prediction.
 *
 * This function calculates the ESKF system function `f(x)`, which is used for state prediction.
 * It computes the new predicted state based on the current state and input measurements (e.g., accelerometer, gyroscope).
 *
 * @param s Current ESKF state, which includes position, velocity, and orientation.
 * @param in Input measurements, including accelerometer and gyroscope data.
 *
 * @return The predicted state vector (24x1).
 */
Eigen::Matrix<double, 24, 1> get_f(state_ikfom& s, const input_ikfom& in);

/**
 * @brief Jacobian of system function with respect to state (x).
 *
 * This function computes the Jacobian matrix `df/dx` of the system function with respect to the state variables.
 * It is used to linearize the system around the current state for the EKF update.
 *
 * @param s Current ESKF state.
 * @param in Input measurements.
 *
 * @return The Jacobian matrix (24x23) of the system function with respect to state.
 */
Eigen::Matrix<double, 24, 23> df_dx(state_ikfom& s, const input_ikfom& in);

/**
 * @brief Jacobian of system function with respect to process noise (w).
 *
 * This function computes the Jacobian matrix `df/dw` of the system function with respect to the process noise terms.
 * It is used to model the effect of process noise on the system's behavior during the EKF update.
 *
 * @param s Current EKF state.
 * @param in Input measurements.
 *
 * @return The Jacobian matrix (24x12) of the system function with respect to process noise.
 */
Eigen::Matrix<double, 24, 12> df_dw(state_ikfom& s, const input_ikfom& in);

/**
 * @brief Convert SO(3) rotation matrix to Euler angles.
 *
 * This function converts a given SO(3) rotation matrix to Euler angles (roll, pitch, yaw).
 * It handles the singularities at the north and south poles of the rotation matrix.
 *
 * @param orient The SO(3) rotation matrix to convert.
 *
 * @return The Euler angles (roll, pitch, yaw) as a vect3.
 */
vect3 SO3ToEuler(const SO3& orient);