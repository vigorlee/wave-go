# 当前加大场景与手动驾驶（2026-09-16）

本节描述当前加大版本；下方旧版记录只对应原来的小地形。

**最终完整复测通过：12/12 阶段，从起点连续上、下加大楼梯，通过上下坡，再上到建筑二楼，末端机身 z=5.898 m。**

- [本轮完整四分屏录像](artifacts/office_large_full_08_final/officeworld_inline_second_floor.mp4)
- [本轮阶段预览](artifacts/office_large_full_08_final/contact_sheet.jpg)
- [实际任务结果](artifacts/office_large_full_08/course/result.json)
- [R 键恢复实测记录](artifacts/office_native_entry_resume_07/recovery_key_request.json)：约 38 秒，重启后在起点站立，控制面板自动重开。

本轮没有用仿真复位完成任何导航阶段；R 恢复测试单独记录。单次完整通过不等于多轮可靠性认证。

- 同一直线：9 级上下楼梯，高 1.08 m、单级 0.12 m；坡道约高 0.431 m、坡角 10°。两者宽 2.3 m。
- 3 个圆柱放在台阶平台和上下坡面的中部；旁侧留通行空间。
- 场景 JSON 配置了 1 个 `human1` 原生行人，在坡后横穿。行人尚未验证完整往返和自主避让，不计作动态避障通过。
- `bash scripts/start_go2w_manual.sh` 打开本地控制面板。先点击“接管手动 / 解除停止锁定”，面板获得焦点后 W/↑ 前进、S/↓ 后退、A/← 左转、D/→ 右转。也可按住屏幕按钮。
- 上楼前对正楼梯、避开平台圆柱，点击“爬楼 / 越障”，再按住 W。普通路面点击“普通行走”。爬楼辅助跟随点击时的方向，不是自动识别所有楼梯。
- 松键或失焦停止；空格/Esc 锁定停止。继续前必须点击解除停止锁定。程序心跳过期后控制器停止驱动。
- **R / 恢复站立按钮是重新启动仿真到起点，再平滑站立**，不是原地翻身策略，也不用于真机。恢复期间锁定运动，约需 1–2 分钟；控制面板随后自动重新打开。每次创建新建图目录，保留之前数据。原生 Backspace 复位实测导致打包模拟器段错误，因此不采用该快捷键。
- 手动接管停止自动任务。显式运行 `record_office_inline_take.sh` 才交还自动控制。

加大版本测试记录在 `artifacts/office_large_manual_01/`、`office_large_manual_02/`、`office_large_recovery_03/`。第一次自动运行实际爬上平台（机身最高 1.442 m），但平台航点距离阈值过严导致未切换阶段，任务判为失败，保留原始失败记录。已调整判定，后续以对应运行的 `course/result.json` 为准。

补充实测：`office_large_recovery_05` 完成加大楼梯上、下楼；`office_large_ramp_resume_06` 调整坡道速度上限后完成上下坡；`office_native_entry_resume_07/course/result.json` 记录从一楼入口到二楼 6/6 阶段成功，末端机身 z=5.897 m。这些是分段测试，不能合并声称一轮完整通过。楼梯入口增加绕到前方的接近点，避免切入台阶侧边。

手动快捷键补充：`1` 普通行走；对正楼梯后按 `2` 启用爬楼辅助，然后按住 `W`。`R` 的重启恢复请求与旧版本崩溃日志均保留，详见各轮 artifacts。

修改前的独立备份在 `artifacts/office_before_large_manual/`。原冻结版本未替换。

---

# OfficeWorld：同一直线障碍段 → 二楼

本次按新的摆放要求调整：一楼的小楼梯、斜坡和圆柱沿 `y=0.7 m` 的同一条通道，由东向西顺路通过；然后进入建筑原有楼梯间，折返上到真正的二楼。二楼楼面约 `z=5.51 m`。原生楼梯是折返结构，因此楼梯间内仍要转向。

原版 GitHub 冻结分支 `freeze-20260916-before-office` 和标签 `frozen-20260916` 不变。所有修改在本独立 Office 副本。此前成功版本的视频、PCD 和测试结果保留；本轮修改前的源码、配置与控制器二进制另存于 `artifacts/office_before_inline_20260916/`。

## 已完成的交付

- [完整四分屏视频](artifacts/office_inline_full_01_final/officeworld_inline_second_floor.mp4)：164.30 秒，1920×1080、20 fps；完整解码检查通过。
- [阶段预览](artifacts/office_inline_full_01_final/contact_sheet.jpg)。
- [任务与录像校验](artifacts/office_inline_full_01_final/verification.json)：10/10 阶段，实际连续运行到二楼。
- [3 cm 累计 PCD](artifacts/office_inline_full_01/capture/go2w_live_dense_3cm.pcd)：2,781,906 点，44.51 MB。[数据报告](artifacts/office_inline_full_01/capture/pcd_report.json)包含范围和哈希。楼梯运动中的 LIO 漂移及离群点尚未作精度评定，不能作为高精度测量地图。
- [原版哈希复核](artifacts/office_inline_full_01/original_source_recheck.json)：1,240 个文件均未变化。

本轮从起点完整通过；分段调试历史单独保留，不代表已完成多轮可靠性认证。

## 布置与实际控制

- 小楼梯：高 0.36 m、宽 1.4 m，上下各 3 级，每级 0.12 m。
- 斜坡：高约 0.230 m、宽 1.4 m、坡角 10°，与小楼梯同轴。
- 两个静态圆柱位于通道两侧，沿同一直线穿过其间。
- 原生首段楼梯单级约 0.21 m，直接使用现有步态实测卡住。增加了 34 级约 0.095 m 高、0.17 m 深的可见踏步和连接平台，覆在原有首段上方，延长起步段、减小整体坡度。原建筑碰撞体未删除；二楼高度未降低。
- 二楼第二段沿用原生楼梯。新增实体通过 UE SceneLoader 生成，与 MuJoCo 实体的位置、大小和旋转逐个核对，共 47 个。

本轮采用基于实际 `/odom/mujoco_odom` 的连续三维航点跟踪，发布速度指令，经既有速度平滑器和 DreamWaQ RL 步态执行。桥接器增加可旋转楼梯局部坐标，支持朝西的小楼梯、朝南的首段和朝北的第二段。机器人位置、关节和楼层高度由物理模拟产生，不通过重置位置或播放关节动画来完成任务。

这是指定路径的闭环演示，不是本轮使用 Nav2 全局规划、Cosmos 在线推断或无图探索；圆柱通行使用已知净空路线，不代表完成动态行人避障测试。

## 四分屏

1. MuJoCo 实际 UDP 状态的只读剖视图；局部墙体/楼板透明或隐藏仅用于看清运动，不改变物理碰撞。
2. RViz 实时 LIO 点云、轨迹及膨胀可视化。
3. RGB、Depth Anything V2 相对深度、Grounding DINO + SAM2 语义。
4. UE 实时场景。

同一次抓屏形成四分屏；模型和传感器自身有处理延迟。Mid360 10 Hz，不显示绿色射线。PCD 来自实际模拟射线的 LIO 配准，不添加模型补点；保留动态历史，未进行地图精度认证。

## 复现

```bash
cd /home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916
bash scripts/start_office_inline_demo.sh office_inline_example_01
bash scripts/record_office_inline_take.sh artifacts/office_inline_example_01
/usr/bin/python3 scripts/office_live/export_dense.py artifacts/office_inline_example_01/capture
/usr/bin/python3 scripts/finalize_office_inline_demo.py artifacts/office_inline_example_01
```

每轮必须使用新名称，避免覆盖已保存的输出。录制期间不要移动四个窗口。

控制器源码：`controllers/go2w_rl_bridge/src/main.cpp`。本机 CUDA 编译工具不完整，因此 `scripts/build_inline_bridge.py` 复用既有 CMake 生成的 C++ 编译/链接参数，编译新桥接器；不需要重新编译 CUDA 内核。脚本依赖本机已有 Torch/eCAL/LCM SDK。

## 验证记录

- `office_inline_native_01`：启动检查拒绝未设置可执行标志的验证脚本，未开始行走。
- `office_inline_native_02`：原生首段第一阶卡住。
- `office_inline_native_03`：只拆分局部台阶仍不足，未通过首段。
- `office_inline_native_04/course`：缓和首段后通过中间平台和横移；进入第二段前转向速度限制过低，退出。
- `office_inline_native_04/upper_probe`：在实际平台姿态上继续进行分段调试，修正转向限制后到达二楼；这不是从起点连续成功的证据。
- **`office_inline_full_01`：从一楼起点连续完成 10/10 个阶段，到达二楼；最终机身 z=5.921 m。** 录像与点云同轮采集，未拼接分段调试结果。源配置保存在该轮 `configuration/`。

几何测试：`/usr/bin/python3 -m pytest tests/test_office_inline.py -q`。原生建筑实体保持一致，小楼梯和坡道同轴、实际有碰撞。UE 运行时实体检查见各轮 `ue_actor_validation.json`。
