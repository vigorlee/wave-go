# Go2-W bridge delta

`go2w_rl_bridge/main.cpp` is the controller-side snapshot used by the validated extended run. It retains the DreamWaQ bridge and adds the opt-in, bounded stair steering blend used when `GO2W_RL_STAIR_OBSTACLE_AVOIDANCE=1`.

The file is provided for review and patch comparison. Building it still requires the workstation's eCAL, LCM, Protobuf, Torch, `zsibot_common`, generated LCM types and DreamWaQ assets; those dependencies are intentionally outside this public repository. The external runtime selected by `WAVE_GO_RUNTIME_ROOT` must use a bridge built from this snapshot (or an equivalent compatible build) for the physical demo command to reproduce the recorded result.
