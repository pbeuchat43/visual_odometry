uint16 schema_version 1
# Authoritative atomic association. Never join ordinary Odometry to latest Status.
nav_msgs/Odometry odometry
rgbd_camera_interfaces/StereoCapture capture
string estimator_generation
uint64 association_epoch
uint64 sequence
string configuration_id
string mode
int64 published_ns
# The pinned upstream success flag is neither calibrated confidence nor a
# discriminator of visual support during inertial continuation.
bool visual_support_known false
bool covariance_known false
