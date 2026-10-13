// SPDX-License-Identifier: Apache-2.0
// 좌·우회전 사각지대의 시간 충돌과 정지 상태를 ROS 없이 계산합니다.

#pragma once

#include "planning/trajectory_stop.hpp"

#include <optional>
#include <string>
#include <vector>

namespace planning {

struct BlindSpotObject {
  std::string id;
  int classification = 0;
  bool inside_detection_area = false;
  double arc_on_detection_lane_m = 0.0;
  double longitudinal_velocity_mps = 0.0;
};

struct BlindSpotParameters {
  bool use_pass_judge_line = true;
  double ttc_min_s = -5.0;
  double ttc_max_s = 5.0;
  double ego_minimum_velocity_mps = 5.0;
  double release_margin_s = 0.0;
};

struct BlindSpotInput {
  double now_s = 0.0;
  double ego_velocity_mps = 0.0;
  double ego_path_arc_m = 0.0;
  double default_stop_arc_m = 0.0;
  double critical_stop_arc_m = 0.0;
  double vehicle_front_m = 0.0;
  double maximum_stop_acceleration_mps2 = -2.5;
  double response_delay_s = 0.0;
  double ego_detection_lane_arc_m = 0.0;
  double stop_detection_lane_arc_m = 0.0;
  std::vector<BlindSpotObject> objects;
};

enum class BlindSpotDecisionKind { Safe, Unsafe, OverPassJudge };

struct BlindSpotDecision {
  BlindSpotDecisionKind kind = BlindSpotDecisionKind::Safe;
  double stop_arc_m = 0.0;
  double ego_time_to_stop_s = 0.0;
  std::optional<std::string> collision_object_id;
  std::optional<double> collision_ttc_s;
};

class BlindSpotCore {
 public:
  explicit BlindSpotCore(BlindSpotParameters parameters = {});
  BlindSpotDecision evaluate(const BlindSpotInput &input);
  void reset();

 private:
  BlindSpotParameters parameters_;
  bool stopped_ = false;
  bool passed_judge_line_ = false;
  std::optional<double> release_start_s_;
};

// 정지 판단의 호길이부터 경로 끝까지 영속도를 적용합니다.
TrajectoryStopResult applyBlindSpotDecision(
    const Trajectory &path, const BlindSpotDecision &decision);

}  // planning 네임스페이스
