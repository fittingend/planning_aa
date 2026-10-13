// Copyright 2020 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 사유 도로 합류의 정지와 출발 상태를 지도 없이 계산합니다.

#pragma once

#include "planning/intersection_stuck_core.hpp"
#include "planning/trajectory.hpp"

#include <optional>
#include <vector>

namespace planning {

struct MergeFromPrivateResult {
  Trajectory trajectory;
  std::optional<double> stop_arc_m;
  bool stopped = false;
};

class MergeFromPrivateCore {
 public:
  MergeFromPrivateResult apply(
      const Trajectory &path,
      const std::vector<CrosswalkBounds> &conflicting_lanes,
      double ego_x, double ego_y, double ego_velocity_mps,
      double now_s, double minimum_conflict_arc_m);

 private:
  bool go_ = false;
  std::optional<double> stopped_since_s_;
};

}  // planning 네임스페이스
