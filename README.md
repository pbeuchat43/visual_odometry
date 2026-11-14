# visual_odometry

ROS 2 bringup and capture-aware output for NVIDIA Isaac ROS Visual SLAM.
Camera acquisition remains in `rgbd_camera`; this package consumes its stereo
images, calibration and optional combined IMU. It never opens a camera.

The development implementation provides Off/VO/VIO integration through
`robot_deployment`, with Docker or native execution chosen during installation.
Both use the same capture-aware estimate and health contract. See the
[interface](docs/interface.md), [installation](docs/installation.md), and
[deployment settings and troubleshooting](https://github.com/pbeuchat43/robot_deployment/blob/dev/docs/visual-odometry-troubleshooting.md).

This repository is under development. The initial reference is Ubuntu 24.04
arm64, ROS 2 Jazzy and Isaac ROS 4.6 on a compatible Jetson. Stationary fixture
measurements establish feasibility, not driving accuracy or deployment acceptance.

The [initial integration evidence](https://github.com/pbeuchat43/robot_deployment/blob/dev/docs/evidence/2026-11-13-visual-odometry/README.md)
records the tested source/image identities, native/Docker comparisons, full-workload
checks and remaining limits.

`visual_odometry_interfaces` is independently installable without NVIDIA or a
camera SDK. `visual_odometry` owns launch presets and the estimate/health adapter.
`robot_deployment` selects source/image identities and robot mounting calibration;
`robot_runtime` supervises the run and any owned Docker container.

Canonical repository: https://github.com/pbeuchat43/visual_odometry.
`dev` carries the consolidated implementation; `main` remains the initial
baseline scaffold and is not a qualified release.

MIT License, copyright 2026 Paul N. Beuchat. NVIDIA dependencies retain their
own licenses; this repository does not redistribute their source or binaries.
