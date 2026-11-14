// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#include "visual_odometry/core.hpp"
#include <isaac_ros_visual_slam_interfaces/msg/visual_slam_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <array>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <optional>

namespace visual_odometry {
namespace {
int64_t monotonic_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::string identifier(const char* path) {
  std::ifstream input(path); std::string result;
  if (!std::getline(input,result) || result.empty()) throw std::runtime_error("host identity unavailable");
  return result;
}
template<class T> T setting(rclcpp::Node& node,const char* name,T value) {
  rcl_interfaces::msg::ParameterDescriptor d; d.read_only=true; return node.declare_parameter<T>(name,value,d);
}
}
class Adapter final : public rclcpp::Node {
 public:
  Adapter() : Node("odometry") {
    if (get_parameter("use_sim_time").as_bool()) throw std::invalid_argument("physical odometry cannot use simulated time");
    Config config;
    config.mode = setting<std::string>(*this,"mode","vo");
    config.configuration_id = setting<std::string>(*this,"configuration_id","");
    config.estimator_generation = identifier("/proc/sys/kernel/random/uuid");
    config.camera_generation = setting<std::string>(*this,"camera_generation","");
    config.clock_domain = "linux_monotonic:"+identifier("/proc/sys/kernel/random/boot_id");
    config.rectified = setting<bool>(*this,"rectified_images",false);
    config.odom_frame = setting<std::string>(*this,"odom_frame","odom");
    config.body_frame = setting<std::string>(*this,"body_frame","camera_link");
    core_ = std::make_unique<Core>(std::move(config));
    const auto qos = rclcpp::SensorDataQoS().keep_last(32);
    estimates_ = create_publisher<Estimate>("~/estimate",qos);
    odometry_ = create_publisher<Odometry>("~/odometry",qos);
    statuses_ = create_publisher<Status>("~/status",rclcpp::QoS(1).best_effort());
    captures_ = create_subscription<Capture>("stereo_capture",qos,[this](Capture::ConstSharedPtr m,const rclcpp::MessageInfo& info) {
      if (publisher(0,info)) core_->capture(std::move(m),monotonic_ns());
      publish();
    });
    poses_ = create_subscription<Odometry>("tracking_odometry",qos,[this](Odometry::ConstSharedPtr m,const rclcpp::MessageInfo& info) {
      if (publisher(1,info)) core_->odometry(std::move(m),monotonic_ns());
      publish();
    });
    upstream_ = create_subscription<isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>("tracking_status",qos,
      [this](isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus::ConstSharedPtr m,const rclcpp::MessageInfo& info) {
        if (publisher(2,info)) core_->upstream(stamp_ns(m->header.stamp),m->vo_state,monotonic_ns());
        publish();
      });
    camera_ = create_subscription<CameraStatus>("camera_status",rclcpp::QoS(1).best_effort(),
      [this](CameraStatus::ConstSharedPtr m,const rclcpp::MessageInfo& info) {
        if (publisher(3,info)) core_->camera(std::move(m),monotonic_ns());
        publish();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(200),[this] {
      if (captures_->get_publisher_count()>1 || poses_->get_publisher_count()>1 || upstream_->get_publisher_count()>1 || camera_->get_publisher_count()>1)
        core_->fail("multiple_input_publishers");
      publish(); auto status=core_->status(monotonic_ns());status.header.stamp=now();statuses_->publish(status);
    });
  }
 private:
  bool publisher(size_t slot,const rclcpp::MessageInfo& info) {
    std::array<uint8_t,RMW_GID_STORAGE_SIZE> gid{};
    std::copy_n(info.get_rmw_message_info().publisher_gid.data,gid.size(),gid.begin());
    if (gids_[slot] && *gids_[slot]!=gid) {core_->fail("input_publisher_changed_restart_required");return false;}
    gids_[slot]=gid;return true;
  }
  void publish() {
    for (auto& estimate:core_->take(monotonic_ns())) {odometry_->publish(estimate.odometry);estimates_->publish(estimate);}
  }
  std::unique_ptr<Core> core_;
  std::array<std::optional<std::array<uint8_t,RMW_GID_STORAGE_SIZE>>,4> gids_;
  rclcpp::Publisher<Estimate>::SharedPtr estimates_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_;
  rclcpp::Publisher<Status>::SharedPtr statuses_;
  rclcpp::Subscription<Capture>::SharedPtr captures_;
  rclcpp::Subscription<Odometry>::SharedPtr poses_;
  rclcpp::Subscription<CameraStatus>::SharedPtr camera_;
  rclcpp::Subscription<isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>::SharedPtr upstream_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<visual_odometry::Adapter>());rclcpp::shutdown();return 0;}
  catch (const std::exception& e) {RCLCPP_ERROR(rclcpp::get_logger("odometry"),"%s",e.what());rclcpp::shutdown();return 1;}
}
