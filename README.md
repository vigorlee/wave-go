# WAVE-Go Successful Demo Backup

This repository is a focused backup of the Go2-W mapless charging demo that was
successfully executed and validated in the source workspace. It preserves the
runtime path, generator adapter, launch/configuration files, controller bridge,
regression tests, and final run evidence needed to reproduce that demo.

## Validated behavior

The successful run completed the following closed-loop sequence:

1. Search for the charging marker.
2. Detect ArUco marker ID 560.
3. Accept a strict six-field Reasoner JSON response.
4. Approach with RGB-D feedback and repeated close-range confirmation.
5. Stop locomotion and command the Go2-W to crouch.
6. Finish with runtime state `succeeded` and posture `charge`.

The restored policy uses a Reasoner budget of 1024 tokens and rejects incomplete
six-field JSON rather than guessing missing fields. Motion is executed in bounded
chunks so perception and safety checks remain in the loop.

## Repository contents

- `scripts/`: mission runtime, generator adapter, visualizer, sensor bridges, and launcher
- `config/`: mapless-search, navigation, SLAM, and RViz configuration
- `controllers/go2w_rl_bridge/`: DreamWaQ/Go2-W velocity bridge
- `tests/`: focused generator, mapless-search, and runtime regression tests
- `evidence/`: successful terminal log and final visualization
- `README_MAPLESS_CHARGER_SEARCH.md`: detailed setup and operating notes

## External dependencies

This is a source backup, not a self-contained model distribution. The following
large or machine-specific dependencies are intentionally excluded:

- Cosmos-family model weights and inference framework
- Matrix/HouseWorld simulator assets
- the external ROS 2 workspace and its `build/`, `install/`, and `log/` products
- CUDA/NVIDIA runtime libraries

Restore those dependencies in the target environment before launching the demo.
See [README_MAPLESS_CHARGER_SEARCH.md](README_MAPLESS_CHARGER_SEARCH.md) for the
detailed runtime procedure.

## Focused regression tests

With ROS 2 Humble and the external workspace available:

```bash
source /opt/ros/humble/setup.bash
source /path/to/genisom_roamerx_open/install/setup.bash
python3 -m unittest \
  tests.test_go2w_house_generator_action \
  tests.test_go2w_house_mapless_search \
  tests.test_go2w_house_runtime
```

This focused backup currently passes all 98 included regression tests.

The successful execution record is retained in
[`evidence/charger_search_success.log`](evidence/charger_search_success.log), and
the final annotated frame is in
[`evidence/final_visualization.jpg`](evidence/final_visualization.jpg).
