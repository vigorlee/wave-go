# Go2-W 四分屏演示视频

主文件：`/home/unitree/Go2w+cosmos3Nav/matrix_go2w_lcm_demo/artifacts/four_screen_demo_20260915_final/go2w_four_screen_demo.mp4`

视频包含一次实际 MATRiX / MuJoCo / UE 仿真连续通过楼梯、静态障碍区、斜坡并到达终点的过程。该次运行完成 11/11 个阶段，使用一次 NavigateThroughPoses 动作。它不是 Cosmos 模型在线决策结果。

## 四个画面与数据来源

1. 左上：该次 MuJoCo 实录，展示场景几何和运动。
2. 右上：该次原始 LiDAR 配准点和 LIO 轨迹，按传感器时间戳在 RViz 中回放，并用同次位姿及速度重算自适应膨胀。标题明确标注 `Same-run LiDAR replay + inflation`。
3. 左下：该次实际 RGB、Depth Anything V2、Grounding DINO + SAM 2 和语义掩膜实录。Depth Anything 显示的是相对深度。
4. 右下：该次 UE 机器狗跟随视角实录。

右上是数据回放，其余三路保留现场录制的连续画面。没有把其他轮次的点云混入本视频；没有插帧、补绘环境或复制点来增密。RGB 语义仍是模型输出，不是人工确认的真值。

原始视频：`/home/unitree/Go2w+cosmos3Nav/matrix_go2w_lcm_demo/artifacts/four_screen_demo_20260915_02/go2w_four_screen_demo.mp4`
原始点云记录：`/home/unitree/Go2w+cosmos3Nav/matrix_go2w_lcm_demo/artifacts/four_screen_demo_20260915_02/capture/registered_xyzi.float32`
点云帧索引：`/home/unitree/Go2w+cosmos3Nav/matrix_go2w_lcm_demo/artifacts/four_screen_demo_20260915_02/capture/frames.jsonl`
实测轨迹：`/home/unitree/Go2w+cosmos3Nav/matrix_go2w_lcm_demo/artifacts/four_screen_demo_20260915_02/capture/lio_trajectory.jsonl`
导航结果：`/home/unitree/Go2w+cosmos3Nav/matrix_go2w_lcm_demo/artifacts/four_screen_demo_20260915_02/course/result.json`

## 对齐和显示参数

- 输出：1920 × 1080，20 fps，H.264 MP4；时长约 96.58 秒。
- 三路实录按统一原始录屏时间截取，从开始导航前 3 秒到到达终点后 5 秒。
- RViz 每 1 秒更新一个累计点云快照，共 98 个显示快照；每个快照仅使用该时刻之前的传感器帧。
- 点云为 4 cm 体素去重，每个体素保留实际扫描点；最后累计 1,388,517 个体素点。
- 雷达数据源为本次仿真 Mid360 的原始回波及 LIO 配准；4 cm 是显示采样间距，不代表测量精度。
- 使用记录时间戳对齐，原始录屏启动时刻有亚秒级不确定性；不声称硬件级同步。
- 橙色范围由实测局部点云、走廊宽度和记录速度计算；它是可视化分析，没有接管原导航控制器。
- 累计点云保留动态历史回波；未宣称完成闭环优化或动态点剔除。

## 录制过程的限制

本次多次复录中，`four_screen_demo_20260915_01` 在楼梯中段停滞，`03` 和 `04` 触发楼梯横向偏移保护；`02` 完整成功，但原 RViz 窗口布局遮住了点云，因此采用该次点云的时间戳回放补齐右上画面。所有原始记录均保留，视频不能据此证明所有重复测试均稳定通过。

## 输出与复现

- `go2w_four_screen_demo.mp4`：成片。
- `video_contact_sheet.png`：六个阶段的抽帧检查图。
- `frame_01.png` 至 `frame_06.png`：对应阶段原尺寸视频帧。
- `video_validation.json`：编码、尺寸、帧率、时长、解码检查和抽帧时间。
- `source_manifest.json`、`replay_frames.json`：源记录和逐帧对齐关系。
- `render_same_run_replay.py`：从本次原始点云在 RViz 中生成回放快照。
- `assemble_video.py`：按时间对齐并生成四分屏视频。
- `assembly_command.json`：准确的视频处理命令。

未修改 PPT，未覆盖或删除原始视频和点云文件。
