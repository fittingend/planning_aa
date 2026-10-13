// Copyright 2022-2024 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 장애물 순항 계획의 저속 통과 구간과 객체별 상태입니다.

#pragma once

#include "planning/motion_velocity_core.hpp"

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace planning {

struct ObstacleCruiseSlowdownCandidate {
  std::string id;
  double front_collision_arc = 0.0;
  double back_collision_arc = 0.0;
  double lateral_distance = 0.0;
  double longitudinal_velocity = 0.0;
  double lateral_velocity = 0.0;
};

struct ObstacleCruiseSlowdownParameters {
  double moving_speed_threshold = 0.5;
  double moving_hysteresis = 0.1;
  double minimum_lateral_margin = 0.2;
  double maximum_lateral_margin = 1.0;
  double moving_minimum_velocity = 2.0;
  double moving_maximum_velocity = 8.0;
  double static_minimum_velocity = 4.0;
  double static_maximum_velocity = 8.0;
  double ego_front_offset = 3.79;
  double target_velocity_time_margin = 1.5;
  double velocity_filter_gain = 0.99;
  double lateral_filter_gain = 0.999;
  double distance_filter_gain = 0.7;
  double minimum_acceleration = -1.0;
  double slowdown_acceleration = -1.0;
  double slowdown_jerk = -1.0;
};

struct ObstacleCruiseSlowdownResult {
  Trajectory trajectory;
  MotionVelocityPlan intervals;
};

class ObstacleCruiseSlowdownCore {
 public:
  explicit ObstacleCruiseSlowdownCore(
      ObstacleCruiseSlowdownParameters parameters = {});
  ObstacleCruiseSlowdownResult plan(
      const Trajectory &path, MotionVelocityPoint2D ego,
      double ego_velocity, double ego_acceleration,
      const std::vector<ObstacleCruiseSlowdownCandidate> &obstacles);
  void reset();

 private:
  struct Previous {
    std::optional<MotionVelocityPoint2D> start;
    MotionVelocityPoint2D end;
    double target_velocity = 0.0;
    double feasible_velocity = 0.0;
    double lateral_distance = 0.0;
    bool moving = false;
  };
  ObstacleCruiseSlowdownParameters parameters_;
  std::unordered_map<std::string, Previous> previous_;
};

}  // planning 네임스페이스
