// Copyright 2020 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 보행로 정지 위치와 정차 후 통과 상태를 계산합니다.

#pragma once

#include "planning/crosswalk_core.hpp"
#include "planning/stop_line_core.hpp"

#include <optional>
#include <vector>

namespace planning {

struct WalkwayParameters {
  double stop_distance_m = 3.5;
  double front_offset_m = 3.79;
  double stop_duration_s = 0.1;
};

enum class WalkwayState { Approach, Stop, Surpassed };

struct WalkwayResult {
  Trajectory trajectory;
  std::optional<double> stop_arc_m;
  WalkwayState state = WalkwayState::Approach;
};

class WalkwayCore {
 public:
  WalkwayCore(std::vector<CrosswalkPoint> polygon,
              std::vector<StopLineSegment> stop_lines = {},
              WalkwayParameters parameters = {});

  WalkwayResult apply(const Trajectory &path, double ego_x, double ego_y,
                      bool vehicle_stopped, double now_s);
  WalkwayState state() const noexcept { return state_; }

 private:
  std::vector<CrosswalkPoint> polygon_;
  std::vector<StopLineSegment> stop_lines_;
  WalkwayParameters parameters_;
  WalkwayState state_ = WalkwayState::Approach;
  std::optional<double> stopped_since_s_;
};

}  // planning 네임스페이스
