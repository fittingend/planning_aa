// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "planning/lanelet_bin_adapter.hpp"
#include "planning/behavior_route_action.hpp"

#include <cstdint>
#include <optional>

namespace adaptive {

struct BehaviorSignalDecision {
  std::uint8_t turn = 0;    // autoware_vehicle_msgs/TurnIndicatorsCommand
  std::uint8_t hazard = 1;  // DISABLE
};

// Intersection search window follows the original TurnSignalDecider defaults:
// 30 m plus three seconds of travel. The action input carries lane change and
// pull over decisions already made by PlanningPipelineMapAdapter.
BehaviorSignalDecision decideBehaviorSignals(
    const planning::LaneletBinAdapter &map, const planning::MissionRoute &route,
    planning::RouteCheckpoint ego, double speed_mps,
    const std::optional<planning::BehaviorRouteActionResult> &action);

}  // namespace adaptive
