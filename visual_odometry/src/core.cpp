// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#include "visual_odometry/core.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <regex>
#include <stdexcept>

namespace visual_odometry {
namespace {
bool bounded(const std::string& s, size_t max = 128) { return !s.empty() && s.size() <= max; }
bool finite_pose(const Odometry& odom) {
  const auto& p = odom.pose.pose.position; const auto& q = odom.pose.pose.orientation;
  const auto& v = odom.twist.twist.linear; const auto& w = odom.twist.twist.angular;
  for (double x : {p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z}) if (!std::isfinite(x)) return false;
  return std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w - 1) < 1e-3;
}
}
int64_t stamp_ns(const builtin_interfaces::msg::Time& stamp) {
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000) return 0;
  return int64_t(stamp.sec)*1000000000 + stamp.nanosec;
}
void Config::validate() const {
  const std::regex frame("[A-Za-z][A-Za-z0-9_/]{0,127}");
  if ((mode != "vo" && mode != "vio") || !std::regex_match(configuration_id,std::regex("[a-f0-9]{64}")) ||
      !bounded(estimator_generation) || !bounded(clock_domain) ||
      !std::regex_match(odom_frame,frame) || !std::regex_match(body_frame,frame) || odom_frame == body_frame ||
      max_age_ns < 10000000 || max_age_ns > 2000000000 || continuity_gap_ns < 34000000 ||
      continuity_gap_ns > 100000000 || capacity < 2 || capacity > 64)
    throw std::invalid_argument("invalid odometry configuration");
}
Core::Core(Config config) : config_(std::move(config)) {
  config_.validate(); status_.state = "initializing"; status_.reason = "waiting_for_matched_estimate";
  status_.mode = config_.mode; status_.configuration_id = config_.configuration_id;
  status_.estimator_generation = config_.estimator_generation; status_.association_epoch = 1;
  status_.clock_domain = config_.clock_domain; status_.expires_after_ns = 1000000000;
}
void Core::lose(const std::string& reason) {
  if (status_.state == "error") return;
  if (status_.state == "tracking") ++status_.association_epoch;
  boundary_stamp_ = std::max(boundary_stamp_,last_output_stamp_);
  status_.unmatched += entries_.size(); entries_.clear();
  status_.state = "lost"; status_.reason = reason;
}
void Core::fail(const std::string& reason) {
  lose(reason); status_.state = "error"; status_.reason = reason.substr(0,256);
}
void Core::require_reset() {
  if (status_.state == "error") return;
  lose("camera_clock_reset_required"); reset_required_ = true;
}
bool Core::inputs_fresh(int64_t now) const {
  if (reset_required_ || !bound_ || !camera_ || camera_->clock_state == "recovering" || camera_->clock_domain != config_.clock_domain ||
      camera_->source_id != status_.source_id || camera_->generation != status_.camera_generation ||
      camera_->observed_ns <= 0 || camera_->observed_ns > now || camera_->expires_after_ns <= 0 ||
      camera_->expires_after_ns > 1000000000 || now-camera_->observed_ns > camera_->expires_after_ns ||
      camera_->state == "error" || !camera_->stereo_enabled || camera_->stereo_state != "streaming" ||
      camera_->last_stereo_ns <= 0 || camera_->last_stereo_ns > now || now-camera_->last_stereo_ns > config_.max_age_ns) return false;
  if (config_.mode == "vio" && (!camera_->imu_enabled || !camera_->combined_imu_enabled ||
      camera_->combined_imu_state != "streaming" || camera_->last_combined_imu_ns <= 0 ||
      camera_->last_combined_imu_ns-now > 5000000 || now-camera_->last_combined_imu_ns > config_.max_age_ns)) return false;
  return true;
}
void Core::expire(int64_t now) {
  if (status_.state == "tracking" && (!inputs_fresh(now) || now-status_.last_output_ns > config_.max_age_ns))
    lose(inputs_fresh(now) ? "output_stale" : "input_stale");
  for (auto it = entries_.begin(); it != entries_.end();) {
    if (now-it->second.received_ns > config_.max_age_ns || it->first <= boundary_stamp_) {
      it = entries_.erase(it); ++status_.unmatched;
    } else ++it;
  }
  while (entries_.size() > config_.capacity) { entries_.erase(entries_.begin()); ++status_.unmatched; }
}
Core::Entry& Core::entry(int64_t stamp, int64_t now) {
  auto& e = entries_[stamp]; if (!e.received_ns) e.received_ns = now; return e;
}
void Core::capture(Capture::ConstSharedPtr m, int64_t now) {
  expire(now); if (status_.state == "error" || reset_required_) return;
  const auto& a = m->left; const auto& b = m->right;
  const auto valid_reference = [&](const auto& c) {
    return c.schema_version == 2 && bounded(c.source_id,256) && bounded(c.generation) &&
      c.clock_domain == config_.clock_domain && c.acquisition_ns > 0 && c.exposure_midpoint_ns > 0 &&
      c.acquisition_ns <= c.received_ns && c.exposure_midpoint_ns <= c.received_ns && c.received_ns <= now &&
      c.exposure_duration_ns >= 0 && c.exposure_duration_ns <= 1000000000;
  };
  const int64_t acquired = std::max(a.exposure_midpoint_ns,b.exposure_midpoint_ns);
  const int64_t stamp = stamp_ns(m->header.stamp);
  if (m->schema_version != 1 || !valid_reference(a) || !valid_reference(b) || a.source_id != b.source_id ||
      a.generation != b.generation || a.unix_offset_ns != b.unix_offset_ns || a.stream_id != "stereo_left" ||
      b.stream_id != "stereo_right" || m->pairing_tolerance_ns != 1000000 ||
      std::abs(a.exposure_midpoint_ns-b.exposure_midpoint_ns) > m->pairing_tolerance_ns ||
      m->rectified != config_.rectified || !bounded(m->calibration_id,256) ||
      a.unix_offset_ns < -acquired || a.unix_offset_ns > std::numeric_limits<int64_t>::max()-acquired ||
      stamp != acquired+a.unix_offset_ns || acquired <= newest_capture_ ||
      now-std::min(a.exposure_midpoint_ns,b.exposure_midpoint_ns) > config_.max_age_ns) {
    ++status_.rejected; return;
  }
  const bool changed = bound_ && (a.source_id != status_.source_id || a.generation != status_.camera_generation);
  if (changed || (!config_.camera_generation.empty() && a.generation != config_.camera_generation)) {
    require_reset(); return;
  }
  if (bound_ && !changed && (m->calibration_id != status_.calibration_id || a.unix_offset_ns != offset_)) {
    fail("calibration_or_clock_changed_within_generation"); return;
  }
  bound_ = true; offset_ = a.unix_offset_ns; status_.source_id = a.source_id; status_.camera_generation = a.generation;
  status_.calibration_id = m->calibration_id; newest_capture_ = acquired; status_.last_input_ns = acquired;
  if (stamp <= boundary_stamp_) { ++status_.rejected; return; }
  entry(stamp,now).capture = std::move(m); expire(now);
}
void Core::odometry(Odometry::ConstSharedPtr m, int64_t now) {
  expire(now); if (status_.state == "error" || reset_required_) return;
  const int64_t stamp = stamp_ns(m->header.stamp);
  if (stamp <= boundary_stamp_ || stamp <= last_output_stamp_ || m->header.frame_id != config_.odom_frame ||
      m->child_frame_id != config_.body_frame || !finite_pose(*m)) { ++status_.rejected; return; }
  auto& e = entry(stamp,now);
  if (e.odometry) { fail("duplicate_odometry_stamp"); return; }
  e.odometry = std::move(m); expire(now);
}
void Core::upstream(int64_t stamp, uint8_t state, int64_t now) {
  expire(now); if (status_.state == "error" || reset_required_) return;
  if (stamp <= boundary_stamp_ || stamp <= last_output_stamp_ || state > 2) { ++status_.rejected; return; }
  status_.upstream_vo_state = state;
  if (state != 1) { lose("upstream_tracking_failed"); boundary_stamp_ = std::max(boundary_stamp_,stamp); return; }
  auto& e = entry(stamp,now);
  if (e.state) { fail("duplicate_upstream_status_stamp"); return; }
  e.state = state; expire(now);
}
void Core::camera(CameraStatus::ConstSharedPtr m, int64_t now) {
  if (m->clock_domain != config_.clock_domain || m->observed_ns <= 0 || m->observed_ns > now ||
      !bounded(m->source_id,256) || !bounded(m->generation) || (camera_ && m->observed_ns <= camera_->observed_ns)) {
    ++status_.rejected; return;
  }
  if (m->clock_state == "recovering" ||
      (!config_.camera_generation.empty() && m->generation != config_.camera_generation) ||
      (bound_ && (m->source_id != status_.source_id || m->generation != status_.camera_generation))) require_reset();
  if (camera_ && m->source_id == camera_->source_id && m->generation == camera_->generation &&
      config_.mode == "vio" && (m->combined_imu_rejected > camera_->combined_imu_rejected ||
      m->combined_imu_overflows > camera_->combined_imu_overflows) && !reset_required_) lose("combined_imu_discontinuity");
  camera_ = std::move(m); expire(now);
}
std::vector<Estimate> Core::take(int64_t now) {
  expire(now); if (status_.state == "error" || !inputs_fresh(now)) return {};
  // Prefer the newest complete entry; late older work cannot overwrite it.
  auto selected = entries_.end();
  for (auto it = entries_.begin(); it != entries_.end(); ++it) {
    const auto& e = it->second;
    if (e.capture && e.odometry && e.state == 1 && it->first > last_output_stamp_ && it->first > boundary_stamp_ &&
        e.capture->left.source_id == status_.source_id && e.capture->left.generation == status_.camera_generation)
      selected = it;
  }
  if (selected == entries_.end()) return {};
  const auto stamp = selected->first; const auto e = selected->second;
  const auto acquired = std::max(e.capture->left.exposure_midpoint_ns,e.capture->right.exposure_midpoint_ns);
  if (now-acquired > config_.max_age_ns) { entries_.erase(selected); ++status_.unmatched; return {}; }
  if (status_.state == "tracking" && acquired-status_.last_output_ns > config_.continuity_gap_ns) {
    ++status_.association_epoch; status_.reason = "pose_gap_new_epoch";
  } else status_.reason = "fresh_upstream_success_unqualified";
  Estimate output; output.odometry = *e.odometry; output.capture = *e.capture;
  output.odometry.pose.covariance.fill(std::numeric_limits<double>::quiet_NaN());
  output.odometry.twist.covariance.fill(std::numeric_limits<double>::quiet_NaN());
  output.estimator_generation = config_.estimator_generation; output.association_epoch = status_.association_epoch;
  output.sequence = ++status_.estimates; output.configuration_id = config_.configuration_id; output.mode = config_.mode; output.published_ns = now;
  last_output_stamp_ = stamp; status_.last_output_ns = acquired; status_.state = "tracking";
  for (auto it = entries_.begin(); it != entries_.end() && it->first <= stamp;) {
    if (it->first != stamp) ++status_.unmatched;
    it = entries_.erase(it);
  }
  return {std::move(output)};
}
Status Core::status(int64_t now) {
  expire(now); status_.observed_ns = now; status_.pending = entries_.size(); return status_;
}
}
