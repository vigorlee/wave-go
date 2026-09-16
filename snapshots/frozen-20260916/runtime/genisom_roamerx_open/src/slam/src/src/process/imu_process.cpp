#include "process/imu_process.h"
const bool time_list(robot::slam::PointType& x, robot::slam::PointType& y)
{
    return (x.curvature < y.curvature);
}

ImuProcess::ImuProcess()
    : b_first_frame_(true),
      imu_need_init_(true),
      start_timestamp_(-1)
{
    init_iter_num   = 1;
    Q               = process_noise_cov();
    cov_acc         = robot::slam::Vec3d(0.1, 0.1, 0.1);
    cov_gyr         = robot::slam::Vec3d(0.1, 0.1, 0.1);
    cov_bias_gyr    = robot::slam::Vec3d(0.0001, 0.0001, 0.0001);
    cov_bias_acc    = robot::slam::Vec3d(0.0001, 0.0001, 0.0001);
    mean_acc        = robot::slam::Vec3d(0, 0, -1.0);
    mean_gyr        = robot::slam::Vec3d(0, 0, 0);
    angvel_last     = robot::slam::Zero3d;
    Lidar_T_wrt_IMU = robot::slam::Zero3d;
    Lidar_R_wrt_IMU = robot::slam::Eye3d;
    last_imu_.reset(new robot::slam::ImuMessage());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()
{
    // ROS_WARN("Reset ImuProcess");
    mean_acc         = robot::slam::Vec3d(0, 0, -1.0);
    mean_gyr         = robot::slam::Vec3d(0, 0, 0);
    angvel_last      = robot::slam::Zero3d;
    imu_need_init_   = true;
    start_timestamp_ = -1;
    init_iter_num    = 1;
    v_imu_.clear();
    IMUpose.clear();
    last_imu_.reset(new robot::slam::ImuMessage());
    cur_pcl_un_.reset(new robot::slam::PointCloudType());
}

void ImuProcess::set_extrinsic(const MD(4, 4) & T)
{
    Lidar_T_wrt_IMU = T.block<3, 1>(0, 3);
    Lidar_R_wrt_IMU = T.block<3, 3>(0, 0);
}

void ImuProcess::set_extrinsic(const robot::slam::Vec3d& transl)
{
    Lidar_T_wrt_IMU = transl;
    Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const robot::slam::Vec3d& transl, const robot::slam::Mat3d& rot)
{
    Lidar_T_wrt_IMU = transl;
    Lidar_R_wrt_IMU = rot;
}

void ImuProcess::reset()
{
    mean_acc         = robot::slam::Vec3d(0, 0, -1.0);
    mean_gyr         = robot::slam::Vec3d(0, 0, 0);
    angvel_last      = robot::slam::Zero3d;
    b_first_frame_   = true;
    imu_need_init_   = true;
    start_timestamp_ = -1;
    init_iter_num    = 1;
    IMUpose.clear();
    cur_pcl_un_.reset(new robot::slam::PointCloudType());
    Q            = process_noise_cov();
    cov_acc      = robot::slam::Vec3d(0.1, 0.1, 0.1);
    cov_gyr      = robot::slam::Vec3d(0.1, 0.1, 0.1);
    cov_bias_gyr = robot::slam::Vec3d(0.0001, 0.0001, 0.0001);
    cov_bias_acc = robot::slam::Vec3d(0.0001, 0.0001, 0.0001);
}

void ImuProcess::set_gyr_cov(const robot::slam::Vec3d& scaler)
{
    cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const robot::slam::Vec3d& scaler)
{
    cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const robot::slam::Vec3d& b_g)
{
    cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const robot::slam::Vec3d& b_a)
{
    cov_bias_acc = b_a;
}

void ImuProcess::IMU_init(const robot::slam::MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, int& N)
{
    robot::slam::Vec3d cur_acc, cur_gyr;

    if (b_first_frame_)
    {
        Reset();
        N                   = 1;
        b_first_frame_      = false;
        const auto& imu_acc = meas.imu.front()->acc;
        const auto& gyr_acc = meas.imu.front()->gyr;
        mean_acc << imu_acc.x(), imu_acc.y(), imu_acc.z();
        mean_gyr << gyr_acc.x(), gyr_acc.y(), gyr_acc.z();
        first_lidar_time = meas.lidar_beg_time;
    }

    for (const auto& imu : meas.imu)
    {
        const auto& imu_acc = imu->acc;
        const auto& gyr_acc = imu->gyr;
        cur_acc << imu_acc.x(), imu_acc.y(), imu_acc.z();
        cur_gyr << gyr_acc.x(), gyr_acc.y(), gyr_acc.z();

        mean_acc += (cur_acc - mean_acc) / N;
        mean_gyr += (cur_gyr - mean_gyr) / N;

        cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
        cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

        N++;
    }
    state_ikfom init_state = kf_state.get_x();

    Eigen::Vector3d gracity_norm(0.0, 0.0, mean_acc.norm());
    init_state.rot = Eigen::Quaterniond::FromTwoVectors(mean_acc, gracity_norm).toRotationMatrix();
    Eigen::Vector3d g(0.0, 0.0, -robot::slam::G_m_s2);
    init_state.grav = S2(g);

    init_state.bg           = mean_gyr;
    init_state.offset_T_L_I = Lidar_T_wrt_IMU;
    init_state.offset_R_L_I = Lidar_R_wrt_IMU;
    kf_state.change_x(init_state);

    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
    init_P.setIdentity();
    init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
    init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
    init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
    init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
    init_P(21, 21) = init_P(22, 22) = 0.00001;
    kf_state.change_P(init_P);
    last_imu_ = meas.imu.back();
}

void ImuProcess::UndistortPcl(
    const robot::slam::MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, robot::slam::PointCloudType& pcl_out)
{
    /*** add the imu of the last frame-tail to the of current frame-head ***/
    auto v_imu = meas.imu;
    v_imu.push_front(last_imu_);
    const double& imu_beg_time = v_imu.front()->timestamp;
    const double& imu_end_time = v_imu.back()->timestamp;
    const double& pcl_beg_time = meas.lidar_beg_time;
    const double& pcl_end_time = meas.lidar_end_time;

    /*** sort point clouds by offset time ***/
    pcl_out = *(meas.lidar);
    sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
    // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

    /*** Initialize IMU pose ***/
    state_ikfom imu_state = kf_state.get_x();
    IMUpose.clear();
    IMUpose.push_back(
        robot::slam::set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

    /*** forward propagation at each imu point ***/
    robot::slam::Vec3d angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    robot::slam::Mat3d R_imu;

    double dt = 0;

    input_ikfom in;
    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
    {
        auto&& head = *(it_imu);
        auto&& tail = *(it_imu + 1);

        double tail_stamp = tail->timestamp;
        double head_stamp = head->timestamp;

        if (tail_stamp < last_lidar_end_time_)
            continue;

        angvel_avr << 0.5 * (head->gyr.x() + tail->gyr.x()), 0.5 * (head->gyr.y() + tail->gyr.y()), 0.5 * (head->gyr.z() + tail->gyr.z());
        acc_avr << 0.5 * (head->acc.x() + tail->acc.x()), 0.5 * (head->acc.y() + tail->acc.y()), 0.5 * (head->acc.z() + tail->acc.z());

        acc_avr = acc_avr * robot::slam::G_m_s2 / mean_acc.norm();  // - state_inout.ba;

        if (head_stamp < last_lidar_end_time_)
        {
            dt = tail_stamp - last_lidar_end_time_;
        }
        else
        {
            dt = tail_stamp - head_stamp;
        }

        in.acc                         = acc_avr;
        in.gyro                        = angvel_avr;
        Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
        Q.block<3, 3>(3, 3).diagonal() = cov_acc;
        Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
        Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
        kf_state.predict(dt, Q, in);

        /* save the poses at each IMU measurements */
        imu_state   = kf_state.get_x();
        angvel_last = angvel_avr - imu_state.bg;
        acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
        for (int i = 0; i < 3; i++)
        {
            acc_s_last[i] += imu_state.grav[i];
        }
        double&& offs_t = tail_stamp - pcl_beg_time;
        IMUpose.push_back(
            robot::slam::set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
    }

    /*** calculated the pos and attitude prediction at the frame-end ***/
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt          = note * (pcl_end_time - imu_end_time);
    kf_state.predict(dt, Q, in);

    imu_state            = kf_state.get_x();
    last_imu_            = meas.imu.back();
    last_lidar_end_time_ = pcl_end_time;

    /*** undistort each lidar point (backward propagation) ***/
    if (pcl_out.points.begin() == pcl_out.points.end())
        return;
    auto it_pcl = pcl_out.points.end() - 1;
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
    {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu << MAT_FROM_ARRAY(head->rot);
        vel_imu << VEC_FROM_ARRAY(head->vel);
        pos_imu << VEC_FROM_ARRAY(head->pos);
        acc_imu << VEC_FROM_ARRAY(tail->acc);
        angvel_avr << VEC_FROM_ARRAY(tail->gyr);

        for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--)
        {
            dt = it_pcl->curvature / double(1000) - head->offset_time;

            robot::slam::Mat3d R_i(R_imu * Exp(angvel_avr, dt));

            robot::slam::Vec3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            robot::slam::Vec3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
            robot::slam::Vec3d P_compensate =
                imu_state.offset_R_L_I.conjugate()
                * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei)
                    - imu_state.offset_T_L_I);

            it_pcl->x = P_compensate(0);
            it_pcl->y = P_compensate(1);
            it_pcl->z = P_compensate(2);

            if (it_pcl == pcl_out.points.begin())
                break;
        }
    }
}

void ImuProcess::Process(
    const robot::slam::MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, robot::slam::CloudPtr cur_pcl_un_)
{
    double t1, t2, t3;
    t1 = omp_get_wtime();

    if (meas.imu.empty())
    {
        return;
    };
    assert(meas.lidar != nullptr);

    if (imu_need_init_)
    {
        IMU_init(meas, kf_state, init_iter_num);

        imu_need_init_ = true;

        last_imu_ = meas.imu.back();

        state_ikfom imu_state = kf_state.get_x();
        if (init_iter_num > robot::slam::MAX_INI_COUNT)
        {
            cov_acc *= pow(robot::slam::G_m_s2 / mean_acc.norm(), 2);
            imu_need_init_ = false;

            cov_acc = cov_acc_scale;
            cov_gyr = cov_gyr_scale;
            std::cout << "IMU Initial Done" << std::endl;
        }

        return;
    }

    UndistortPcl(meas, kf_state, *cur_pcl_un_);

    t2 = omp_get_wtime();
    t3 = omp_get_wtime();
}
