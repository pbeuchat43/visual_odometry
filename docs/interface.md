# Odometry interface

The adapter/bringup version 0.2.0 requires camera interfaces 0.3.0 for clock
recovery status. The independent odometry interface package remains version
0.1.0; it needs neither NVIDIA binaries nor camera SDKs. Build all participants
against the same camera types.

The camera owns images, clocks and calibration. Standard mono8 image and
CameraInfo topics feed Isaac ROS Visual SLAM directly. The adapter receives only
small `StereoCapture`, camera `Status`, upstream Odometry and VisualSlamStatus.
It does not relay images or open a camera. Camera-internal TF is published by
the camera; deployment owns the measured robot mount.

The pinned upstream wrapper selects the maximum timestamp of the input image
set. With `min_num_images=2`, `override_publishing_stamp=false`, and a 1 ms stereo
matcher, it gives tracking odometry and status the later exposure midpoint.
See the [pinned timestamp selection](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/v4.6-0/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp#L657-L676)
and [status publication](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/v4.6-0/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp#L960-L973).
DDS delivery order across topics is unspecified. The adapter uses a bounded,
order-independent exact-stamp join of both source references, odometry and
same-stamp successful upstream status, requiring fresh camera inputs.

`~/estimate` is authoritative: odometry, the original stereo capture metadata,
estimator generation, association epoch, output sequence and configuration
identity are atomic. `~/odometry` is a convenience copy without generation
identity. Its stamp is acquisition time, never publication time. Unknown pose
and twist covariance are NaN; `covariance_known=false`. The adapter does not
produce calibrated uncertainty from an upstream default or zero covariance.

An association key comprises `(estimator_generation, association_epoch,
capture.left.source_id, capture.left.generation)`. Consumers clear pose history
and pending associations when it changes. Camera generation/calibration changes,
tracking failure, expired input/output, or a gap in accepted poses break
continuity. A supervised restart gives a new estimator generation. Pending work
is bounded to 32 joins with a 500 ms default expiry. Out-of-order older output
cannot overwrite newer estimates; stale/unmatched output is counted and omitted.

`~/status` emits every 200 ms and expires after 1 second in the host monotonic
clock. States are initializing, tracking, lost and error. Tracking means fresh,
matched input and upstream success. The upstream wrapper reports success when
its active tracker returns a pose; it supplies no independent visual-support
flag in VIO and no calibrated confidence. `visual_support_known=false` remains
explicit even while tracking. This interface alone does not qualify poses for
driving or calibrated cone compensation. Those require physical acceptance.

The upstream reset service emits no atomic reset marker and does not clear all
input synchronizers. Invoking it externally remains outside the continuity
contract. The odometry launch now owns a persistent supervisor that observes
camera timing generations. A camera clock recovery or generation change latches
the old adapter lost: it cannot accept a new generation using the old tracker.
The supervisor stops the estimator and adapter together, waits for their complete
process group to exit and for the camera streams to recover, then starts a new
pair with fresh subscriptions and a new estimator generation. The camera process
and its device session remain open. This works inside the existing native process
tree or Docker container; runtime ownership of that enclosing run is unchanged.

The supervisor forwards the adapter's status through the existing public topic
and reports initializing/lost during deliberate restarts. It does not relay images
or estimates. Unexpected child exits and duplicate/changing input publishers within
an estimator generation remain errors. ROS graph identity is not authentication.
Mapping/localization and upstream TF outputs are disabled.
Default poses use a correctly named camera-body frame. Enabling robot-frame TF
requires an approved measured mount and exactly one publisher for that edge.

The YAML preset records upstream VIO noise defaults: gyro density 0.000244
rad/(s·√Hz), gyro random walk 0.000019393 rad/(s²·√Hz), acceleration density
0.001862 m/(s²·√Hz), and acceleration random walk 0.003 m/(s³·√Hz), at a nominal
200 Hz calibration frequency. That frequency describes the noise model, not
measured sensor delivery. These are not device-specific calibration results.
[Pinned cuVSLAM units](https://github.com/nvidia-isaac/cuVSLAM/blob/33da60662b27c0c5b63e0597bd778066af2731ed/libs/cuvslam/cuvslam2.h#L204-L214).
