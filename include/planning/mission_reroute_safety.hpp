// SPDX-License-Identifier: Apache-2.0
// 원본 mission planner의 공통 route 구간 재계획 안전 길이 검사.
#pragma once

#include "planning/mission_planner_core.hpp"

namespace planning {

struct MissionRerouteGeometry {
  std::function<std::vector<CrosswalkPoint>(std::int64_t)> centerline;
  std::function<bool(std::int64_t, RouteCheckpoint)> contains;
};

bool checkMissionRerouteSafety(
    const RouteSelectorRoute &original,
    const RouteSelectorRoute &target,
    const MissionPlannerEgo &ego,
    const MissionRerouteGeometry &geometry,
    double reroute_time_threshold_s = 10.0,
    double minimum_reroute_length_m = 30.0);

}  // namespace planning
