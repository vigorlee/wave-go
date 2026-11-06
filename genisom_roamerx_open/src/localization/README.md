# Localization
---
## 描述（Description）
A multi-sensor fusion localization package based on ROS2 Humble, subscribed to receive PCD point cloud maps, fused IMU and Lidar, using UKF filtering method for localization, and outputting localization information, including position information, direction information, velocity information.

## 依赖（Dependency）
    CMake >= 3.12
    G++/GCC >= 11.4.0
    ROS == Humble
    Eigen == 1.12.1

## 构建（Build）
Clone the repository and colcon build:
```
colcon build --packages-select fast_gicp ndt_omp localization
```

## 运行
run localization and load pcd map
```
source ./install/setup.bash
ros2 launch localization localization.launch.py 
ros2 service call /load_map_service robots_dog_msgs/srv/LoadMap "{pcd_path: /XXX/map.pcd}"
e.g.
ros2 service call /load_map_service robots_dog_msgs/srv/LoadMap "{pcd_path: /home/user_name/datd/map/map.pcd}"

```

## 配置参数（Arguments）
| 参数名称                      |      数据类型      |     默认值         | 参数说明                       |
| :--------------------------: | :---------------: | :-------------: | :----------------------------: |
| **基本参数 (Basic Parameters)** |
| points_topic                 | string | /livox/lidar         | 激光雷达点云话题名称  |
| imu_topic                    | string | /livox/imu           | IMU话题名称  |
| localization_odom_frame_id   | string | base_link            | 定位里程计frame名称  |
| **话题名称 (Topic Names)** |
| odom_topic                   | string | /odom/localization_odom  | 定位里程计话题名称  |
| aligned_points_topic         | string | /aligned_points      | 对齐点云话题名称  |
| localization_info_topic      | string | /localization_info   | 定位信息话题名称  |
| global_map_points_topic      | string | /global_map_points   | 全局地图点云话题名称  |
| **IMU参数 (IMU Parameters)** |
| use_imu                      | bool   | true                 | 是否使用IMU  |
| invert_acc                   | bool   | false                | 是否反转加速度计数据  |
| invert_gyro                  | bool   | false                | 是否反转陀螺仪数据  |
| imu_init_time                | double | 3.0                  | IMU初始化时间(s)  |
| imu_init_queue_size          | int    | 600                  | IMU初始化队列长度  |
| imu_init_max_gyro_var        | double | 0.05                 | IMU陀螺仪最大方差阈值  |
| imu_init_max_acce_var        | double | 0.5                  | IMU加速度计最大方差阈值  |
| imu_data_filter_num          | int    | 5                    | IMU数据滤波间隔数值  |
| **数值参数 (Numeric Parameters)** |
| globalmap_voxel_size         | float  | 0.3                  | 全局地图体素大小  |
| points_voxel_filter_size     | float  | 0.2                  | 点云滤波体素大小  |
| **其他参数 (Other Parameters)** |
| cool_time_duration           | double | 0.5                  | 估计器初始化等待时间  |
| **配准方法参数 (Registration Method Parameters)** |
| reg_method                   | string | NDT_OMP              | 配准方法  |
| ndt_neighbor_search_method   | string | DIRECT7              | NDT邻居搜索方法  |
| ndt_neighbor_search_radius   | double | 1.0                  | NDT邻居搜索半径  |
| ndt_resolution               | double | 0.5                  | NDT分辨率  |
| **初始位姿参数 (Initial Pose Parameters)** |
| specify_init_pose            | bool   | true                 | 是否指定初始位姿  |
| init_pos_x                   | float  | 0.0                  | 初始位置x  |
| init_pos_y                   | float  | 0.0                  | 初始位置y  |
| init_pos_z                   | float  | 0.0                  | 初始位置z  |
| init_ori_w                   | float  | 1.0                  | 初始方向w(四元数)  |
| init_ori_x                   | float  | 0.0                  | 初始方向x(四元数)  |
| init_ori_y                   | float  | 0.0                  | 初始方向y(四元数)  |
| init_ori_z                   | float  | 0.0                  | 初始方向z(四元数)  |
| **初始位姿初始化参数 (Initial Pose Initialization Parameters)** |
| init_match_count_threshold   | int    | 2                    | 初始化NDT有效匹配阈值  |
| init_match_score_threshold   | double | 0.15                 | 初始化NDT匹配得分阈值  |
| **全局定位参数 (Global Localization Parameters)** |
| use_global_localization_init | bool   | true                 | 是否使用全局定位初始化  |
| init_pose_change_threshold   | double | 0.01                 | 位置变化阈值(m)  |
| init_quat_change_threshold   | double | 0.01                 | 姿态变化阈值  |
| global_localization_timeout  | double | 10.0                 | 全局定位超时时间(s)  |
| **外参矩阵 (Extrinsic Parameters)** |
| init_R                       | std::vector<double>(9) | [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] | IMU安装位置的旋转矩阵  |
| init_T                       | std::vector<double>(16) | [1.0, 0.0, 0.0, -0.011, 0.0, 1.0, 0.0, -0.02329, 0.0, 0.0, 1.0, 0.04412, 0.0, 0.0, 0.0, 1.0] | 雷达到IMU的变换矩阵  |
