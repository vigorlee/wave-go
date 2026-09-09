# WAVE-Go

World-model action adaptation and verified execution for the Unitree Go2-W.

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![Latest release](https://img.shields.io/github/v/release/vigorlee/wave-go?label=release)](https://github.com/vigorlee/wave-go/releases/latest)
[![Extended demo](https://img.shields.io/badge/extended%20demo-18%2F18%20passed-1F883D)](https://github.com/vigorlee/wave-go/releases/latest)

WAVE-Go is a Go2-W robotics project built around a pretrained vision-action world model and explicit execution checks. The repository contains two related demos with different control boundaries:

- **Mapless charging** — the world model proposes nominal actions; a geometry adapter and veto-only safety layer convert and filter them before execution.
- **Hybrid long-range navigation** — Cosmos3-Edge selects one approved route; Nav2/RoamerX performs navigation and obstacle avoidance; DreamWaQ controls the robot.

The second demo is the long-range stair, ramp, cylinder, and pedestrian scenario developed in the referenced Codex session.

## Contents

| Component | Description |
| --- | --- |
| [`demos/go2w-cosmos-extended-navigation`](demos/go2w-cosmos-extended-navigation) | Continuous 18-stage Go2-W navigation demo |
| [`README_MAPLESS_CHARGER_SEARCH.md`](README_MAPLESS_CHARGER_SEARCH.md) | Mapless charging setup and operation |
| [`README_COSMOS3_NAVIGATION_DATA_PLAN.md`](README_COSMOS3_NAVIGATION_DATA_PLAN.md) | Data collection and training plan |
| [`config/`](config) and [`scripts/`](scripts) | Mapless charging implementation |
| [`controllers/`](controllers) and [`tests/`](tests) | Go2-W bridge and regression tests |

## Extended navigation demo

The demo uses the following control chain:

```text
language task + camera
        │
        ▼
Cosmos3-Edge: select one allowlisted route_id
        │
        ▼
mission supervisor: one NavigateThroughPoses action
        │
        ▼
Nav2 / RoamerX: global path, MPPI local control, obstacle avoidance
        │
        ▼
DreamWaQ + Go2-W
        │
        └── RGB-D + LiDAR + odometry feedback
```

The world model does not publish velocity commands, local trajectories, or replacement paths. The demo scene contains:

- two staircases with center-course cylinders;
- a physical `+10° → 0.42 m crest → −10°` ramp;
- four terrain-center cylinders and four post-stair slalom cylinders;
- three moving pedestrians represented in the world-model obstacle cloud;
- MuJoCo and UE scene files kept as a matched pair.

### Verified result

| Metric | Result |
| --- | --- |
| Route | `farthest_end_via_stairs_ramps_cylinders_pedestrians` |
| Stages | 18 / 18 |
| Nav2 actions | `NavigateThroughPoses=1`, intermediate restarts `=0` |
| Recorded travel | 60.15 m |
| Physical ramp height | 0.413 → 0.851 → 0.397 m |
| Terrain-center obstacles | 4 / 4 passed |
| Moving pedestrians | 3, nearest center distance 0.974 m |
| Recording | H.264, 2560×1440, 15 fps, 347.134 s |

The machine-readable summary is [`demos/go2w-cosmos-extended-navigation/evidence/result.json`](demos/go2w-cosmos-extended-navigation/evidence/result.json). The complete recording and ramp excerpt are available in the [extended-navigation release](https://github.com/vigorlee/wave-go/releases/tag/v0.2.0-extended-navigation).

## Quick start

The repository does not vendor Matrix/HouseWorld, ROS workspaces, CUDA libraries, DreamWaQ weights, or Cosmos3-Edge checkpoints. Point the demo at an existing runtime workstation:

```bash
git clone https://github.com/vigorlee/wave-go.git
cd wave-go/demos/go2w-cosmos-extended-navigation

export WAVE_GO_RUNTIME_ROOT=/home/unitree/matrix_go2w_lcm_demo
export COSMOS_VLN_ROOT=/home/unitree/matrix_g1_lcm_demo
```

Run the portable checks first:

```bash
./validate.sh
```

Run the full desktop demo and write evidence to a new directory:

```bash
OUT="$PWD/artifacts/extended_navigation_$(date +%Y%m%d_%H%M%S)"
COSMOS_VLN_ARTIFACT_DIR="$OUT" ./scripts/record_demo.sh
```

For manual operation:

```bash
./scripts/start_extended_stack.sh
./scripts/start_cosmos_vln_visualization.sh
# inspect the running stack, then stop it
./scripts/stop_extended_stack.sh
```

Run as the normal desktop user. Do not use `sudo`.

## Repository layout

```text
wave-go/
├── config/                         mapless charging configuration
├── controllers/                    mapless charging Go2-W bridge
├── demos/
│   └── go2w-cosmos-extended-navigation/
│       ├── config/                 route and RViz configuration
│       ├── controller/             extended stair-steering snapshot
│       ├── evidence/               compact result and keyframes
│       ├── scene/                  MuJoCo, UE, and scene JSON
│       ├── scripts/                bridge, supervisor, launcher, validators
│       └── tests/                  protocol and terrain regression tests
├── scripts/                        mapless charging runtime
├── tests/                          mapless charging tests
└── README_*.md                     focused technical documents
```

## Validation

`demos/go2w-cosmos-extended-navigation/validate.sh` checks:

- MuJoCo/UE scene parity and physical ramp geometry;
- route and pedestrian contracts;
- Python compilation and shell syntax;
- optional ROS regression tests when `WAVE_GO_RUN_ROS_TESTS=1` is set.

The published snapshot was validated with 32 ROS regression tests in addition to the portable checks.

## Limitations

The published result is a reproducible MuJoCo/UE simulation run. It is not a real-robot safety guarantee, a contact-force experiment, or evidence of unknown-scene generalization. The recorded height trace comes from `/odom/mujoco_odom`; the run does not include foot-force, contact-force, or joint-torque measurements.

## Releases

Large media files are kept out of Git history. Download the latest demo assets from [GitHub Releases](https://github.com/vigorlee/wave-go/releases/latest) and verify them with the included `SHA256SUMS` file.
