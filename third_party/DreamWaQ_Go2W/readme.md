# DreamWaQLeggedWheel: Legged-Wheel Robot Reinforcement Learning Locomotion

A **Go2W legged-wheel robot locomotion reinforcement learning project** based on NVIDIA Isaac Gym. For the Isaac Lab version of this project, see [LocoLeggedWheel](https://github.com/zhaozijie2022/LocoLeggedWheel).  
This project focuses on optimizing **reward design** and **training stability** for the legged-wheel robot structure, and provides **sim2real deployment** interfaces and configs for the Unitree Go2W.

<div align="center">
  <p align="right">
    <span> 🌎 <a href="README.md"> English </span> | <a href="README_CN.md"> 中文 </a>
  </p>
</div>


## 🦾 Real Robot Demos

<p align="center">
  <img src="docs/demos/gross.gif" alt="" width="23%"/>
  <img src="docs/demos/stair.gif" alt="" width="23%"/>
  <img src="docs/demos/stone.gif" alt="" width="23%"/>
  <img src="docs/demos/unilateral-bridge.gif" alt="" width="23%"/>
</p>

---

## 🖥️ Simulation & Training

- **Install Isaac Gym Preview 4.** See [Isaac Gym documentation](https://developer.nvidia.com/isaac-gym) if you run into issues.  
  **Note:** The package usually cannot be downloaded directly via wget; download it from the NVIDIA website, extract it, then run the steps below.
  ```bash
  conda create -n env_isaacgym python=3.8
  conda activate env_isaacgym

  # After downloading the .tar.gz from the website, extract and install (adjust paths to your extracted directory)
  tar -zxvf IsaacGym_Preview_4_Package.tar.gz
  cd IsaacGym_Preview_4_Package/isaacgym/python
  pip install -e .
  # Verify Isaac Gym installation
  pip show isaacgym
  ```

- **Install RSL-RL and Legged Gym locally**
  ```bash
  cd rsl_rl-1.0.2
  pip install -e .
  cd ../legged_gym
  pip install -e .
  ```

- **Train and test**
  ```bash
  python legged_gym/scripts/train.py --task=go2w

  python legged_gym/scripts/play.py --task=go2w
  ```

---

## 🚀 Real Robot Deployment

- Install [unitree_sdk2_python](https://support.unitree.com/home/zh/Go2-W_developer/Python).
- Align simulation and real-robot parameters in `deploy_real/configs/g2w.yaml`. Note that **joint order differs** between the simulator and the real robot and must be mapped accordingly.
- Two deployment modes are supported (configure the network interface name as needed):
  - Run on a host PC and send commands over Ethernet.
  - Run on the robot onboard computer.

**Example:**
```bash
python deploy_real/deploy_real.py
python deploy_real/deploy_real.py <network_interface> <config_file>
```

---

## ✨ Features

1. Velocity estimator  
2. Asymmetric actor-critic  
3. C++ deployment support  

---

## 🙏 Acknowledgments

This project is built upon [go2w_rl_gym](https://github.com/ShengqianChen/go2w_rl_gym), [MetaRobotics](https://github.com/LucienJi/MetaRobotics), and [DreamWaQ by Manaro-Alpha](https://github.com/Manaro-Alpha/DreamWaQ). Thanks to the authors for their contributions to the open-source community.

This repo contains an implementation of the paper [Learning Robust Quadrupedal Locomotion With Implicit Terrain Imagination via Deep Reinforcement Learning](https://arxiv.org/abs/2301.10602) on Go2W.
