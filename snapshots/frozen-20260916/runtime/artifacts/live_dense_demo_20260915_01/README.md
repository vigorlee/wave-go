# Go2-W 实际仿真演示与高密度 PCD 录制

日期：2026-09-15。本次实际启动 MATRiX / MuJoCo / UE 的 YardWorld 场景；机器狗连续执行楼梯上、楼梯下、静态障碍区、斜坡上、斜坡下及终点，共 11/11 阶段，状态 succeeded，一次 NavigateThroughPoses 动作，无中途重新派发。现场测得坡道高度增加约 0.450 m、下降约 0.441 m，见 course/result.json。

## PCD

文件：`/home/unitree/Go2w+cosmos3Nav/matrix_go2w_lcm_demo/artifacts/live_dense_demo_20260915_01/capture/go2w_live_dense_3cm.pcd`

- 点数：2,211,978。
- 文件字节数：35,391,854。
- 3 cm 体素去重，每个体素选择一个实际测得点；不是插值，也不是把点重复拷贝增密。
- 1851 帧，36,978,570 个完整配准点，时间跨度 184.981 s。
- 同批点云 12 cm 体素为 200,075 点，本次点数约 11.06 倍。该比例不是定位精度或地图精度。
- 字段 x/y/z/intensity，binary PCD；SHA256 7b2fdcf123b6340e7b69cc6e1034f70f5a46595bcc6b63cb5a7b8370a1a5650f。
- 坐标为 LIO 世界坐标，现有发布端将 native map 命名为 lio_map；不使用仿真真值位姿替代雷达建图。
- LiDAR 为仿真 Mid360，10 Hz，random_scan=true；LIO 点过滤由 2 调为 1，表面和地图匹配体素为 10 cm；输出配准帧取 feats_undistort 的完整点集，另行按 3 cm 导出。匹配分辨率与导出分辨率不同。
- 累计记录保留动态历史回波，未声称已做完动态物体剔除或闭环精度验证。

## 现场自适应膨胀

live_capture.py 接入真实本轮 /wavego/slam/registered_points 与 /wavego/navigation/odom，在附近原始障碍点上实时测宽，并计算膨胀半径，发布到 /wavego/live_dense/inflation。计算半径范围为 [0.6250000000000001, 1.2030539324280562] m。它在本次现场计算并可视化，但没有把未经实机标定的参数写入控制器；运动由原已验证导航策略执行。

现场 RViz 实时累计显示 6 cm 点云；采集结束后 show_saved_pcd.py 向同一显示话题发布完整 3 cm PCD 数据。现在 RViz 所见完整点云是本轮保存结果，不再标称实时刷新。

## 记录与复现

- capture/registered_xyzi.float32：全部收到的逐帧点记录，每点 4 个小端 float32，共 16 字节。
- capture/frames.jsonl：时间戳、帧长度、文件偏移与原始 frame 名。
- capture/lio_trajectory.jsonl：实测 LIO 位姿。
- capture/validation.json：PCD 点数、payload 长度、有限值、1851 帧索引及时间递增验证。
- capture/pcd_report.json：密度、边界及 SHA256。
- capture/go2w_live_dense_3cm.npz：对应 NumPy 数据。
- configuration/lio_dense.yaml、configuration/live_dense.rviz：本次配置。
- live_demo.mp4：实际四窗口桌面录屏。前段 RViz 因新配置格式错误为空白，后段修复并显示；UE 场景和 PCD 采集不中断。没有补绘或伪造同步画面。
- final_pcd_rviz.png：保存后的 PCD 在 RViz 中的截图。
- ue_stairs.png：真实场景中机器狗通过楼梯时的截图。

最初启动因 UE 窗口关闭触发联动退出；重新启动后进行了本次完整采集。RViz 新配置错误已修复并用 YAML 解析器验证。原始老 PCD 和历史运行结果未修改。

离线重新导出：`/usr/bin/python3 export_dense.py`。

显示已录 PCD：先 source ROS 2 Humble 与本项目 install/setup.bash，再设置 ROS_DOMAIN_ID=89、RMW_IMPLEMENTATION=rmw_zenoh_cpp，运行 `/usr/bin/python3 show_saved_pcd.py`。
