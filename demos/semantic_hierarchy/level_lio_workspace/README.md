# Isolated gravity-reference LIO experiment

`src/robot_slam` is copied from the local MATRiX `genisom_roamerx_open/src/slam/src` source. Original source headers and LICENSE are retained. The runtime source and the frozen WaveGo V2 are not overwritten.

Local changes: stationary 400-sample IMU initialization; initialization of the previous LiDAR end time; independent gyro integration and opt-in rank-two gravity residual; finite / 30 m range filtering; separate archive directory. `mapping.gravity_prior_weight` defaults to zero in C++; the OfficeWorld experiment config uses 30.

This is not an official FAST-LIO2 release. The gravity reference is a correlated heuristic alongside IMU propagation, not a new independently validated estimator. It improved the preserved replay but did not remove all drift on the fresh recording. Use the reports and retained raw LIO PCD to assess that limitation. The structural-calibrated PCD additionally uses explicitly known floor heights; see `../docs/level_mapping.md`.

Build with `bash ../scripts/build_level_lio.sh` from this directory, or `bash scripts/build_level_lio.sh` from the extension root. Runtime dependencies remain provided by ROS 2 Humble and the existing MATRiX workspace.
