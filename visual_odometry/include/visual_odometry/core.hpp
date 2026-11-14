// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#pragma once
#include <nav_msgs/msg/odometry.hpp>
#include <rgbd_camera_interfaces/msg/stereo_capture.hpp>
#include <rgbd_camera_interfaces/msg/status.hpp>
#include <visual_odometry_interfaces/msg/estimate.hpp>
#include <visual_odometry_interfaces/msg/status.hpp>
#include <map>
#include <optional>
#include <vector>

namespace visual_odometry {
using Capture = rgbd_camera_interfaces::msg::StereoCapture;
using CameraStatus = rgbd_camera_interfaces::msg::Status;
using Odometry = nav_msgs::msg::Odometry;
using Estimate = visual_odometry_interfaces::msg::Estimate;
using Status = visual_odometry_interfaces::msg::Status;
struct Config {
  std::string mode = "vo", configuration_id, estimator_generation, clock_domain;
  std::string camera_generation;  // Supervisor's immutable generation for this tracker.
  std::string odom_frame = "odom", body_frame = "camera_link";
  bool rectified = false;
  int64_t max_age_ns = 500000000, continuity_gap_ns = 50000000;
  size_t capacity = 32;
  void validate() const;
};
int64_t stamp_ns(const builtin_interfaces::msg::Time& stamp);
class Core {
 public:
  explicit Core(Config config);
  void capture(Capture::ConstSharedPtr message, int64_t now);
  void odometry(Odometry::ConstSharedPtr message, int64_t now);
  void upstream(int64_t stamp, uint8_t state, int64_t now);
  void camera(CameraStatus::ConstSharedPtr message, int64_t now);
  std::vector<Estimate> take(int64_t now);
  Status status(int64_t now);
  void fail(const std::string& reason);
 private:
  struct Entry {
    Capture::ConstSharedPtr capture;
    Odometry::ConstSharedPtr odometry;
    std::optional<uint8_t> state;
    int64_t received_ns = 0;
  };
  Entry& entry(int64_t stamp, int64_t now);
  void expire(int64_t now);
  bool inputs_fresh(int64_t now) const;
  void lose(const std::string& reason);
  void require_reset();
  Config config_;
  Status status_;
  CameraStatus::ConstSharedPtr camera_;
  std::map<int64_t, Entry> entries_;
  int64_t newest_capture_ = 0, last_output_stamp_ = 0, boundary_stamp_ = 0;
  int64_t offset_ = 0;
  bool bound_ = false;
  bool reset_required_ = false;
};
}
