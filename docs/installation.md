# Installation and launch

Normal robot integration belongs to `robot_deployment`. Enable
`visual_odometry: {enabled: true, backend: docker}` in the private site and stage
a new release during maintenance. Choose `native` there for a private prefix.
The runtime selector offers Off/VO/VIO; it does not change backend or download
dependencies. Existing sites stay Off. Camera, detector, recorder and odometry
must be rebuilt together against camera interfaces 0.2.0.

The estimator recipe currently admits Ubuntu 24.04 arm64, ROS Jazzy, L4T 39.2.1
and CUDA 13.2. It deliberately rejects a different host rather than upgrading
shared GPU libraries. This is a qualified development recipe, not a general
Jetson installer. Native ELF resolution and a real sensor launch are separate
checks; a successful build alone does not establish runtime compatibility.

`deployment/dependencies.lock.json` pins the NVIDIA base image digest, Visual
SLAM package version, and hashes of 14 native archives. The native installer
extracts those archives under its destination, checks ELF dependencies, and
builds interfaces and the small adapter there. It uses existing host ROS,
OpenCV and GPU dependencies and records their critical versions. Preparation
refuses a changed recorded dependency version. It never replaces host packages.

Docker requires an already installed NVIDIA runtime. The build records its
resolved package list and immutable image ID. The complete transitive apt
closure is recorded rather than individually version locked; rebuilds at a
later date may produce a different image. Use the recorded image ID as the
release artifact identity. Runtime owns creation, graceful stop and recovery;
the estimator never receives the Docker socket or opens a camera.

## Building a backend separately

Run on the reserved target with clean checkouts at the selected commits. The
integration installer verifies those checkouts before invoking this helper.
For a separate build, the caller must provide the same verified identities:

```bash
python3 /absolute/visual_odometry/deployment/install.py \
  --backend native \
  --camera-source /absolute/rgbd_camera \
  --source-id FULL_VISUAL_ODOMETRY_COMMIT \
  --camera-source-id FULL_CAMERA_COMMIT \
  --destination /absolute/new-estimator-prefix \
  --work /absolute/new-build-directory \
  --cache /absolute/verified-archive-cache
```

Use `--backend docker` to build an image. Destination and work paths must be
new; failed attempts are retained for inspection. The cache is reusable and
every archive is checked before extraction. The resulting `manifest.json`
records source file hashes, source commits, preset and dependency identities,
host dependencies, and the explicit launch argv. Native prefixes contain
absolute paths and must remain at their installed location.

The independent `visual_odometry_interfaces` package needs only ROS standard
messages and `rgbd_camera_interfaces`; it can be built with colcon without
installing NVIDIA packages. Building the adapter additionally needs
`isaac_ros_visual_slam_interfaces`.

## Launch contract

`ros2 launch visual_odometry odometry.launch.py config_file:=/absolute/config.json`
requires a bounded prepared JSON object. `robot_deployment` produces and freezes
it with the camera settings and mounting transform. Its fields are:

| Field | Meaning |
|---|---|
| `schema_version` | `1` |
| `mode` | `vo` or `vio` |
| `camera_backend` | `oak` or `realsense` |
| `namespace`, `camera_node` | Absolute ROS namespace and camera node path |
| `frames` | Distinct `body`, `odom`, `left`, `right`, `imu` frame names |
| `executor` | `single` for RealSense, `multi` for OAK |
| `preset_sha256` | SHA-256 of the installed `config/estimator.yaml` |
| `backend_identity` | Immutable Docker image ID or `sha256:` plus native manifest hash |

Host and container need the same ROS domain, middleware, local discovery and
clock domain. Runtime mounts the prepared file read-only, uses host network,
IPC and PID namespaces and the runtime account UID/GID. Large images go directly
from the host camera to the NVIDIA node; the adapter subscribes only to metadata,
upstream odometry and status. It publishes atomic `odometry/estimate` and
`odometry/status`, plus a convenience `odometry/odometry` topic.

Mapping, localization and upstream TF publication are disabled. Whole-run
restart is the operator reset operation. During camera clock recovery, the
supervisor stops the estimator and adapter together and starts a fresh pair
after the camera is stable; the camera session stays open. The runtime still
owns the enclosing process tree or container. The default pose frame is the camera
body; publishing robot-frame odometry requires a measured approved mount.
Stationary throughput and successful tracking do not establish moving-car
accuracy, IMU axes/noise calibration, or calibrated cone compensation.
