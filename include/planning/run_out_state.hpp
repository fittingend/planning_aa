// Copyright 2022 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 돌발 진입 장애물의 정지·접근 전환, 순간 검출과 정지점 유지 상태입니다.

#pragma once

#include "planning/trajectory.hpp"
#include "planning/trajectory_stop.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>

namespace planning {

enum class RunOutState { Go, Stop, Approach };

struct RunOutStateParameters {
  double stop_velocity_mps = 2.0;
  double stop_duration_s = 1.0;
  double disable_approach_distance_m = 1.0;
  double keep_approach_duration_s = 1.0;
};

struct RunOutStateInput {
  double now_s = 0.0;
  double velocity_mps = 0.0;
  double distance_to_collision_m = 0.0;
  std::optional<std::uint64_t> obstacle_id;
};

struct RunOutStateResult {
  RunOutState state = RunOutState::Go;
  std::optional<std::uint64_t> target_obstacle_id;
};

class RunOutStateCore {
 public:
  explicit RunOutStateCore(RunOutStateParameters parameters = {});
  RunOutStateResult update(const RunOutStateInput &input);
  RunOutState state() const noexcept { return state_; }

 private:
  RunOutStateParameters parameters_;
  RunOutState state_ = RunOutState::Go;
  double stop_since_s_ = 0.0;
  double last_approach_obstacle_s_ = 0.0;
  std::optional<std::uint64_t> previous_obstacle_id_;
};

struct RunOutTemporalParameters {
  bool ignore_momentary_detection = true;
  double detection_threshold_s = 0.15;
  double keep_stop_point_s = 2.0;
  bool approaching_enabled = false;
};

struct RunOutStopPoint {
  double x = 0.0;
  double y = 0.0;
};

class RunOutTemporalCore {
 public:
  explicit RunOutTemporalCore(RunOutTemporalParameters parameters = {});
  bool acceptCollision(double now_s, bool detected);
  void recordStop(double now_s, RunOutStopPoint stop_point);
  std::optional<RunOutStopPoint> maintainedStop(
      double now_s, bool inserted_now) const;

 private:
  RunOutTemporalParameters parameters_;
  std::optional<double> first_detected_s_;
  std::optional<double> last_stop_s_;
  std::optional<RunOutStopPoint> last_stop_point_;
};

struct RunOutApproachParameters {
  double velocity_limit_mps = 1.0;
  double margin_m = 1.0;
  double front_offset_m = 3.79;
};

struct RunOutApproachResult {
  Trajectory trajectory;
  std::optional<double> stop_arc_m;
};

// 원본의 접근 분기처럼 차량 부근부터 감속하고 충돌점 앞에 정지점을 삽입합니다.
// 충돌점의 경로 호길이는 호출자가 계산해 전달합니다.
RunOutApproachResult applyRunOutApproachVelocity(
    const Trajectory &input, std::size_t ego_segment_index,
    double collision_arc_m, const RunOutApproachParameters &parameters = {});

// 제동 저크를 별도로 지정하지 않은 원본 분기의 충돌점 앞 정지점을 만듭니다.
TrajectoryStopResult applyRunOutStopVelocity(
    const Trajectory &input, double collision_arc_m,
    double stop_margin_m = 1.0, double front_offset_m = 3.79);

struct RunOutBrakingInput {
  double collision_arc_m = 0.0;
  double ego_arc_m = 0.0;
  double velocity_mps = 0.0;
  double acceleration_mps2 = 0.0;
  bool object_detection = true;
};

struct RunOutBrakingParameters {
  double stop_margin_m = 2.5;
  double front_offset_m = 3.79;
  double deceleration_jerk_mps3 = -0.3;
  double normal_min_jerk_mps3 = -0.5;
  double normal_min_acceleration_mps2 = -0.5;
  double limit_min_acceleration_mps2 = -2.5;
};

struct RunOutBrakingResult {
  Trajectory trajectory;
  std::optional<double> stop_arc_m;
  double required_stop_distance_m = 0.0;
  bool distance_valid = true;
  bool deceleration_required = false;
};

// 원본의 저크 지정 분기에서 제동 거리와 충돌 여유를 비교합니다.
RunOutBrakingResult planRunOutBraking(
    const Trajectory &trajectory, const RunOutBrakingInput &input,
    const RunOutBrakingParameters &parameters = {});

}  // planning 네임스페이스
