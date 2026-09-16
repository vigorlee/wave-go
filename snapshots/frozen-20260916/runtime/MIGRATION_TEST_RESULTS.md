# G1 / Go2W YardWorld stair migration test

Test date: 2026-07-16 (Asia/Shanghai)

## Migrated runtime

- UE runtime and YardWorld building assets: clean official MATRiX v0.1.2 release.
- Local G1 launcher/runtime extension retained.
- Go2W DreamWaQ + LCM bridge and RoamerX navigation workflow migrated from the
  previously verified `matrix_go2w_lcm_demo_V1_code` snapshot.
- Both launchers automatically select stair navigation mode: the static map
  remains enabled for walls, while LiDAR stair points are not marked as local
  obstacles.
- Go2W stair entry now aligns on the flat floor before the first riser. Once on
  the stairs it keeps a forward component while correcting lateral/heading
  error, preventing diagonal entry and in-place turns on a tread.

## G1 integrated test

Commands:

```bash
cd /home/unitree/matrix_go2w_lcm_demo
./scripts/start_g1_demo.sh --foreground
./scripts/send_stair_goal.sh up --feedback
```

Result:

- `/navigate_to_pose`: `SUCCEEDED`
- Physical simulated final position: approximately `[0.15, 4.74, 2.38]`
- Robot reached the upper platform; this was not a navigation-only success.
- MuJoCo, official UE YardWorld, LCM bridge, RoamerX topics/actions and TF were
  all active.

## Go2W integrated test

Commands:

```bash
cd /home/unitree/matrix_go2w_lcm_demo
ROAMERX_RVIZ_READ_ONLY=1 ./scripts/start_demo.sh --foreground
./scripts/send_stair_goal.sh up --feedback
```

Result from the final exact-command run:

- `/navigate_to_pose`: `SUCCEEDED`
- Navigation time reported by the action: `25.900 s`
- Final action distance: `0.18899 m` (goal tolerance `0.20 m`)
- Physical bridge state after settling: approximately
  `pos=[0.346, 4.482, 2.031]`, stage `idle`
- The robot climbed the stairs and settled on the upper platform.
- Full action feedback: `logs/go2w_stair_goal_feedback.log`
- Controller trace: `logs/bridge.log`

## Navigation and visualization checks

`scripts/check_demo.sh` passed:

- MATRiX MuJoCo
- MATRiX UE5
- Go2W controller bridge
- Zenoh, robot_forward and LCM velocity publisher
- `/odom/mujoco_odom`, `/livox/lidar`, `/plan`, `/cmd_vel_nav`, `/cmd_vel`
- `odom -> base_link -> lidar` TF

Active stair-mode values:

```text
obstacle_layer=True
pointcloud_marking=False
velocity_limits=[0.45, 0.0, 0.60]
goal_tolerance=0.20
```

The active UE process loads `/Game/Maps/YardWorld`; it is the official textured
building scene, not the red MuJoCo diagnostic renderer.

## Main migrated files

- `scripts/start_g1_demo.sh`
- `scripts/start_demo.sh`
- `matrix/scripts/run_sim.sh`
- `controllers/go2w_rl_bridge/src/main.cpp`
- `install/bin/go2w_rl_bridge`
- `matrix/src/UeSim/Linux/`

