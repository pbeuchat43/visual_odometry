// Copyright (c) 2026 Paul N. Beuchat. MIT License.
// Synthetic association/freshness tests; no GPU or camera.
#include "visual_odometry/core.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace visual_odometry;
constexpr int64_t offset = 1700000000000000000LL;
void check(bool condition,const char* reason) {if(!condition)throw std::runtime_error(reason);}
Config config(const std::string& mode="vo") {
  Config c;c.configuration_id=std::string(64,'a');c.estimator_generation="estimator";c.clock_domain="synthetic";c.mode=mode;return c;
}
builtin_interfaces::msg::Time stamp(int64_t value) {
  builtin_interfaces::msg::Time t;t.sec=value/1000000000;t.nanosec=value%1000000000;return t;
}
Capture::SharedPtr capture(int64_t t,const std::string& generation="A") {
  auto m=std::make_shared<Capture>();m->header.stamp=stamp(t+offset);m->header.frame_id="left";
  m->calibration_id="calibration";m->pairing_tolerance_ns=1000000;
  for(auto* r:{&m->left,&m->right}) {
    r->source_id="camera";r->generation=generation;r->clock_domain="synthetic";r->unix_offset_ns=offset;
    r->acquisition_ns=t-1000;r->exposure_midpoint_ns=t;r->exposure_duration_ns=2000;r->received_ns=t+1000000;r->sequence=t;
  }
  m->left.stream_id="stereo_left";m->right.stream_id="stereo_right";m->left.exposure_midpoint_ns=t-100;return m;
}
Odometry::SharedPtr pose(int64_t t) {
  auto m=std::make_shared<Odometry>();m->header.stamp=stamp(t+offset);m->header.frame_id="odom";m->child_frame_id="camera_link";
  m->pose.pose.orientation.w=1;return m;
}
CameraStatus::SharedPtr camera(int64_t t,const std::string& generation="A") {
  auto m=std::make_shared<CameraStatus>();m->source_id="camera";m->generation=generation;m->clock_domain="synthetic";
  m->state="streaming";m->stereo_enabled=true;m->stereo_state="streaming";m->last_stereo_ns=t;
  m->observed_ns=t+1000000;m->expires_after_ns=1000000000;m->imu_enabled=m->combined_imu_enabled=true;
  m->combined_imu_state="streaming";m->last_combined_imu_ns=t;return m;
}
void add(Core& core,int64_t t,const std::string& generation="A") {
  core.camera(camera(t,generation),t+2000000);core.capture(capture(t,generation),t+2000000);
  core.odometry(pose(t),t+2000000);core.upstream(t+offset,1,t+2000000);
}
int main() {
  try {
    // Every topic arrival order must produce the same exact capture association.
    std::array<int,3> order{0,1,2};
    do {
      Core core(config());const int64_t t=1000000000;core.camera(camera(t),t+2000000);
      for(int item:order) {
        if(item==0)core.capture(capture(t),t+2000000);
        if(item==1)core.odometry(pose(t),t+2000000);
        if(item==2)core.upstream(t+offset,1,t+2000000);
      }
      auto result=core.take(t+2000000);
      check(result.size()==1 && result[0].capture.left.exposure_midpoint_ns==t-100 &&
        result[0].association_epoch==1 && std::isnan(result[0].odometry.pose.covariance[0]) &&
        !result[0].visual_support_known,"order-independent exact join failed");
      check(core.take(t+3000000).empty(),"output repeated");
    } while(std::next_permutation(order.begin(),order.end()));
    Core core(config());int64_t t=1000000000;add(core,t);auto first=core.take(t+2000000).front();
    core.capture(capture(t+33333333),t+35000000);core.odometry(pose(t+33333334),t+35000000);
    core.upstream(t+33333334+offset,1,t+35000000);check(core.take(t+35000000).empty(),"nearby stamp incorrectly joined");
    t+=100000000;add(core,t);auto gap=core.take(t+2000000).front();
    check(gap.association_epoch>first.association_epoch,"pose gap bridged association epochs");
    auto stale=core.status(t+600000000);check(stale.state=="lost" && stale.pending==0,"stale output remained ready");
    t+=700000000;add(core,t,"B");
    check(core.take(t+2000000).empty() && core.status(t+2000000).reason=="camera_clock_reset_required","camera generation reused old tracker");
    add(core,t+33333333,"B");check(core.take(t+35000000).empty(),"reset-required latch resumed without tracker restart");
    Core restarted(config());add(restarted,t,"B");auto restart=restarted.take(t+2000000);
    check(restart.size()==1 && restart[0].capture.left.generation=="B","fresh tracker could not bind new generation");
    Core held(config());add(held,t);check(!held.take(t+2000000).empty(),"initial pose missing");
    auto recovering=camera(t+33333333);recovering->clock_state="recovering";
    held.camera(recovering,t+35000000);add(held,t+66666666);
    check(held.take(t+70000000).empty() && held.status(t+70000000).reason=="camera_clock_reset_required","clock hold failed to latch");
    core.upstream(t+33333333+offset,2,t+35000000);
    check(core.status(t+35000000).state=="lost","upstream failure ignored");
    Core vio(config("vio"));t=1000000000;add(vio,t);auto missing=camera(t);missing->observed_ns+=1000000;missing->combined_imu_state="unavailable";
    vio.camera(missing,t+3000000);check(vio.take(t+3000000).empty(),"VIO without fresh combined IMU accepted");
    Core bounded(config());
    for(int i=0;i<100;++i)bounded.odometry(pose(t+i),t+2000000);
    check(bounded.status(t+2000000).pending<=32 && bounded.status(t+2000000).unmatched==68,"join capacity unbounded");
    Core invalid(config());auto bad=capture(t);bad->right.generation="wrong";invalid.capture(bad,t+2000000);
    check(invalid.status(t+2000000).rejected==1,"mixed generation accepted");
    add(invalid,t);check(invalid.take(t+2000000).size()==1,"invalid capture poisoned future input");
    auto changed=capture(t+33333333);changed->calibration_id="different";invalid.capture(changed,t+35000000);
    check(invalid.status(t+35000000).state=="error","same-generation calibration mutation accepted");
    std::cout<<"odometry join and freshness checks passed (synthetic)\n";return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
