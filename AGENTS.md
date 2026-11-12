# Repository guidance

This repository owns reusable VO/VIO bringup, input validation and atomic
estimate/health outputs. Camera acquisition, clocks, internal calibration and
combined IMU belong to `rgbd_camera`. Installation and robot mounting calibration
belong to `robot_deployment`; process/container ownership belongs to `robot_runtime`.

- Read README.md and docs/interface.md before changing public behavior.
- Keep the interfaces independently installable, without NVIDIA or camera SDKs.
- Preserve capture timestamps and generation/calibration identity. Bound retained
  work and reject stale, unmatched or discontinuous estimates explicitly.
- Run execution checks only on reserved Jah (`ssh jah`), using isolated files,
  processes, ROS domains and containers. Local editing, Git and inspection are
  allowed. Preserve the selected deployment and unrelated workloads.
- Record exact software/configuration identities and evidence limits. Software
  and stationary tests do not establish physical calibration or driving accuracy.
- Use feature branches from `dev` and intermediate commits. Squash to `dev` after
  the user's finish confirmation. Promotion to `main` is a separate agreement.
- Coordinate interface changes and source pins with affected repositories.
- Follow the user's workflow preferences; ordinary work needs no new ceremony.
- Project-owned material is MIT, copyright Paul N. Beuchat. Preserve third-party
  notices and licenses; do not commit device identity, private imagery or secrets.
