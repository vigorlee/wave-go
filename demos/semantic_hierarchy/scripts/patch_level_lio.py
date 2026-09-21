#!/usr/bin/env python3
"""Apply an isolated, opt-in gyro gravity factor to the copied LIO source."""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]/'level_lio_workspace/src/robot_slam'

def change(path,old,new):
    p=ROOT/path;s=p.read_text()
    if new in s:return
    assert s.count(old)==1,(path,s.count(old))
    p.write_text(s.replace(old,new))

change('include/process/imu_process.h','    double             first_lidar_time;', '''    double             first_lidar_time;
    // Independent gyro integration, initialized from stationary accelerometer.
    // No simulator orientation, position, or map geometry is read here.
    robot::slam::Mat3d gravity_reference_R = robot::slam::Mat3d::Identity();
    robot::slam::Vec3d gravity_reference_bias = robot::slam::Vec3d::Zero();
    robot::slam::Vec3d gravity_reference_last_gyr = robot::slam::Vec3d::Zero();
    robot::slam::Vec3d gravity_body_up = robot::slam::Vec3d::UnitZ();
    double gravity_reference_time = -1.0;
    bool gravity_reference_ready = false;''')
change('src/process/imu_process.cpp','    init_state.bg           = mean_gyr;', '''    gravity_reference_R = init_state.rot.toRotationMatrix();
    gravity_reference_bias = mean_gyr;
    gravity_reference_time = meas.imu.back()->timestamp;
    last_lidar_end_time_ = meas.lidar_end_time;
    gravity_reference_last_gyr = meas.imu.back()->gyr;
    gravity_body_up = gravity_reference_R.transpose() * robot::slam::Vec3d::UnitZ();
    gravity_reference_ready = false;
    init_state.bg           = mean_gyr;''')
change('src/process/imu_process.cpp','if (init_iter_num > robot::slam::MAX_INI_COUNT)', 'if (init_iter_num > 400) // two seconds at 200 Hz; keep robot stationary')
change('src/process/imu_process.cpp','    /*** sort point clouds by offset time ***/', '''    // Integrate actual angular velocity at its original timestamps. The
    // independent reference is never updated from LiDAR residuals.
    for (const auto& imu : meas.imu) {
        const double step = imu->timestamp - gravity_reference_time;
        if (step <= 0) continue;
        if (step > 0.1) { gravity_reference_ready = false; break; }
        const robot::slam::Vec3d omega = 0.5 * (gravity_reference_last_gyr + imu->gyr) - gravity_reference_bias;
        gravity_reference_R = gravity_reference_R * Exp(omega, step);
        gravity_reference_time = imu->timestamp;
        gravity_reference_last_gyr = imu->gyr;
        gravity_reference_ready = true;
    }
    const double extrapolate = pcl_end_time - gravity_reference_time;
    if (extrapolate >= 0 && extrapolate < 0.025 && gravity_reference_ready) {
        const robot::slam::Vec3d omega_tail = gravity_reference_last_gyr - gravity_reference_bias;
        robot::slam::Mat3d at_scan = gravity_reference_R * Exp(omega_tail, extrapolate);
        gravity_body_up = at_scan.transpose() * robot::slam::Vec3d::UnitZ();
    } else { gravity_reference_ready = false; }

    /*** sort point clouds by offset time ***/''')
change('include/mapping_alg.h','        bool extrinsic_est_en = true, path_en = true;', '''        bool extrinsic_est_en = true, path_en = true;
        double gravity_prior_weight = 0.0;''')
change('src/mapping_alg.cpp','        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");', '''        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        gravity_prior_weight = this->declare_parameter<double>("mapping.gravity_prior_weight", 0.0);''')
change('src/mapping_alg.cpp','''        ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num, 12);  // 23
        ekfom_data.h.resize(effct_feat_num);''', '''        const bool use_gravity_prior = gravity_prior_weight > 0 && p_imu->gravity_reference_ready;
        const int rows = effct_feat_num + (use_gravity_prior ? 3 : 0);
        ekfom_data.h_x = Eigen::MatrixXd::Zero(rows, 12);
        ekfom_data.h.resize(rows);
        if (use_gravity_prior) {
            const Vec3d prediction = s.rot.conjugate() * Vec3d::UnitZ();
            Mat3d jacobian;
            jacobian << SKEW_SYM_MATRX(prediction);
            ekfom_data.h_x.block<3,3>(effct_feat_num,3) = gravity_prior_weight * jacobian;
            ekfom_data.h.tail<3>() = gravity_prior_weight * (p_imu->gravity_body_up - prediction);
        }''')
change('CMakeLists.txt','add_definitions(-DROOT_DIR="$ENV{HOME}/.jszr")', 'add_definitions(-DROOT_DIR="/home/unitree/Go2w+cosmos3Nav/wave-go-v2/demos/semantic_hierarchy/.runtime/level_lio_storage")')
print('Isolated LIO gravity factor prepared')
