 # DreamWaQLeggedWheel: 轮腿机器人强化学习运动控制

基于 NVIDIA Isaac Gym 的 **Go2W 轮腿机器人 locomotion 强化学习项目**，该项目的IsaacLab 版本，请参考[LocoLeggedWheel](https://github.com/zhaozijie2022/LocoLeggedWheel)。
本项目主要在针对轮腿机器人结构，对 **奖励函数** 与 **训练稳定性** 进行了优化，，并提供了在宇树 Unitree Go2W 上 **sim2real 部署** 的接口与配置。

<div align="center">
  <p align="right">
    <span> 🌎 <a href="README.md"> English </span> | <a href="README_CN.md"> 中文 </a>
  </p>
</div>

🦾 真机演示
---
<p align="center">
  <img src="docs/demos/gross.gif" alt="" width="23%"/>
  <img src="docs/demos/stair.gif" alt=“”" width="23%"/>
  <img src="docs/demos/stone.gif" alt="" width="23%"/>
  <img src="docs/demos/unilateral-bridge.gif" alt="" width="23%"/>
</p>


🖥️ 仿真训练
---
+ 安装 Isaac Gym Preview 4，有问题请参考 [Isaac Gym 官方文档](https://developer.nvidia.com/isaac-gym)。  
  **说明**：若使用 wget 无法直接下载，请从网页获取安装包后解压到当前目录，再执行后续步骤。
  ```bash
  conda create -n env_isaacgym python=3.8
  conda activate env_isaacgym

  # 若已从官网下载得到 .tar.gz，解压后进入 python 目录安装（以下路径按实际解压目录调整）
  tar -zxvf IsaacGym_Preview_4_Package.tar.gz
  cd IsaacGym_Preview_4_Package/isaacgym/python
  pip install -e .
  # 检查 Isaac Gym 是否正常安装
  pip show isaacgym
  ```
+ 本地安装 RSL-RL，Legged Gym
  ``` bash
  cd rsl_rl-1.0.2
  pip install -e .
  cd ../legged_gym
  pip install -e .
  ```
+ 训练与测试
  ``` bash 
  python legged_gym/scripts/train.py --task=go2w

  python legged_gym/scripts/play.py --task=go2w
  ```


🚀 实体部署
---
+ 安装 [unitree_sdk2_python](https://support.unitree.com/home/zh/Go2-W_developer/Python)；
+ 在 `deploy_real/configs/g2w.yaml` 中对齐仿真训练参数，特别注意机器人关节顺序仿真器与实体中并不一致，需要转换；
+ 代码支持两种部署方式（需更改网卡名）
  + 上位机运行，通过网线发送控制指令
  + 机器人本体运行
运行示例：
```bash
# Train
python deploy_real/deploy_real.py 
python deploy_real/deploy_real.py <网卡名> <配置文件名>
```

✨ 特性
---
1. 速度估计器
2. 非对称 actor-critic
3. cpp 部署


🙏 致谢
---
本项目基于 [go2w_rl_gym](https://github.com/ShengqianChen/go2w_rl_gym)，[MetaRobotics](https://github.com/LucienJi/MetaRobotics) 和 [DreamWaQ Implementated by Manaro-Alpha](https://github.com/Manaro-Alpha/DreamWaQ)，感谢这些项目作者对开源社区的贡献。

