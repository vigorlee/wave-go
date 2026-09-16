# Go2W + Cosmos3Nav 多地形策略扩展

本扩展在不替换原有导航栈的前提下，增加了可验证的地形策略接口和独立 MuJoCo 场景。它是语义地图与后续受约束强化学习策略之间的工程过渡层，不代表楼梯、斜坡策略已经完成训练或真机验收。

## 内容

- `config/go2w_terrain_policy_profiles.json`：平地、上下坡、上下楼梯、复杂路障、窄通道、动态等待和安全停车参数。
- `scripts/go2w_terrain_policy.py`：按证据优先级选择策略，并对速度、角速度、机身高度和俯仰角做安全投影。
- `matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_coupled_nav.xml`：楼梯、15° 双向坡道、交错路障、窄门和横穿动态障碍组合场景。
- `matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_yard_extended_nav.xml`：稳定 YardWorld 路线的独立扩展版，含 4 个圆柱、连续的浅上坡/下坡（约 +2°/−2.7°）和远端路线；对应 UE XML 与 `scene/scene_go2w_extended_nav.json` 同步。
- `tests/test_go2w_terrain_policy.py`：策略切换、限幅和失效安全测试。

## 快速验证

```bash
python3 -m unittest discover -s tests -p 'test_*.py'

python3 scripts/go2w_terrain_policy.py \
  --observation-json '{"age_s":0.1,"localization_confidence":0.95,"slope_deg":12.0,"corridor_clearance_m":2.0}' \
  --requested-vx 0.4
```

若要在 Matrix 启动脚本中使用组合场景，将物理场景覆盖为：

```bash
MATRIX_MUJOCO_SCENE=go2w/scene_terrain_coupled_nav.xml
```

接入训练策略时，应保留 `safe_stop` 与 `dynamic_wait` 的优先级，并让 RL Actor 只在安全投影后的动作空间内输出连续指令。
