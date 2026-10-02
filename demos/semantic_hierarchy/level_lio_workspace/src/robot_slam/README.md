# robot_slam

## 描述（Description）

​	A SLAM package integrating multi-sensor data for simultaneous localization and mapping



## 依赖（Depend）

​	Cmake > 3.10

​	G++/GCC > 11.4.0

​	ROS == Humble

​	PCL == 1.12.1

​	OpenCV == 4.5.4

​	Eigen == 3.4.0

​	zsibot_interface



## 构建（Build）

### 单包构建（Single Build）

```c++
cp -r robot_slam 工作空间/src

cd 工作空间

colcon build --packages-select robot_slam
```

### 工作空间构建（ros2_workspace Build）

```C++
cd ros2_worksapce

colcon build --packages-select robot_slam
```



## 运行（Run）

```C++
cd 工作空间
source ./install/setup.bash
ros2 launch robot_slam mapping.launch.py 或者 ./run_slam
```



## 开始建图（Run Mapping）

```
ros2 service call /slam_state_service robots_dog_msgs/srv/MapState "{data: 3}"
```

`注：默认输入雷达话题为: "/livox/lidar"，imu话题为: "/livox/imu"，qos均为best_effort,运行程序后，需要发布服务才能开始建图`



## 保存建图（Save Map）

```
ros2 service call /slam_state_service robots_dog_msgs/srv/MapState "{data: 5}"
```

`注：开始键图后，当建图完成时(终端会打印: "Save Map Success.")，需要发送服务保存地图（主要包括pcd点云地图、pgm占用地图和建图轨迹.txt; 默认保存在主目录下的.jszr/map目录下）`



## 参数说明（Command Line Arguments）

#### RosWrapper Paramters

| 参数名称                      |      数据类型      |     默认值         | 参数说明                       |
| :---------------------------: | :-------------: | :-------------: | :----------------------------: |
| --feature_extract_enable | bool | false | 点云处理模块是否提取特征 |
| --point_filter_num | int   | 2 | 点云处理模块的采样数目     |
| --max_iteration | int   | 3 | 匹配迭代次数 |
| --filter_size_surf |   double   |      0.2      |                原始点云降采样体素边长                |
|     --filter_size_map     |  double  |               0.2               |                 地图点云降采样体素边长                 |
| --cube_side_length |   double   | 1000.0 | 地图裁剪立方体边长 |
| --common.lid_topic |   str    | "/livox/lidar" |                订阅雷达话题（默认Qos为best_effort）                |
|   --common.imu_topic   |   str    |     "/livox/imu"     | 订阅imu话题（默认Qos为best_effort） |
|    --common.time_sync_en    |   bool   |              false              | 是否进行时间同步 |
| --common.time_offset_lidar_to_imu |   double   |     0.0     | 雷达数据落后imu时间 |
|       --preprocess.lidar_type       |   int   |      1       |  雷达类型默认为1（livox: mid360）  |
|      --preprocess.scan_line      |   int   | 4 | 雷达线束数量 |
| --preprocess.blind | double | 0.5 | 雷达盲区半径 |
| --preprocess.timestamp_unit | int | 3 | 时间单位量级 |
| --preprocess.scan_rate | int | 10 | 雷达扫描频率 |
| --mapping.acc_cov | double | 0.1 | imu加速度噪声 |
| --mapping.gyr_cov | double | 0.1 | imu陀螺仪噪声 |
| --mapping.b_acc_cov | double | 0.0001 | imu加速度偏置噪声 |
| --mapping.b_gyr_cov | double | 0.0001 | imu陀螺仪偏置噪声 |
| --mapping.fov_degree | double | 360.0 | 视角度数 |
| --mapping.det_range | float | 100.0 | 范围阈值 |
| --mapping.extrinsic_est_en | bool | false | 是否优化雷达与imu的外参 |
| --mapping.extrinsic_T | vetor | [ -0.011, -0.02329, 0.04412 ] | lidar2imu平移 |
| --mapping.extrinsic_R | vetor | [ 1., 0., 0.,0.,1.,0.,0.,0.,1.] | lidar2imu旋转 |
| --publish.path_en | bool | true | 是否发布路径 |
| --publish.map_en | bool | false | 是否发布地图点云 |
| --publish.world_points_en | bool | true | 是否发布map坐标系下的点云 |
| --publish.body_points_en | bool | true | 是否发布body坐标系下的点云 |
| --pcd2pgm.file_name | str | "map" | 2D地图前缀名字 |
| --pcd2pgm.thre_z_min | double | 0.1 | 提取有效点云的最小z值 |
| --pcd2pgm.thre_z_max | double | 2.0 | 提取有效点云的最大z值 |
| --pcd2pgm.flag_pass_through | int | 0 | 统计滤波提取内点标志位 |
| --pcd2pgm.map_resolution | double | 0.05 | 地图分辨率 |
| --pcd2pgm.thre_radius | double | 0.1 | 半径滤波半径大小 |
| --pcd2pgm.thres_point_count | int | 10 | 半径滤波半径数量阈值 |

`注：发布map坐标系点云话题为: /world_points，qos为best_effort；发布body坐标系下点云话题为：/body_points，qos为reliable；发布路径话题为: /path，qos为reliable；发布odomtery话题为: /slam_odom，qos为reliable, tf随odom信息一起发布`
