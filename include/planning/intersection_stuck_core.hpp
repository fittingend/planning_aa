// Copyright 2024 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 교차로의 정체 차량 정지 위치를 계산하는 독립 코어입니다.

#pragma once

#include "planning/crosswalk_core.hpp"
#include "planning/lanelet_osm_adapter.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace planning {

struct IntersectionStuckObject {
  CrosswalkPoint position;
  double speed_mps = 0.0;
  bool target_vehicle = true;
  std::uint8_t classification = 1;
};

struct IntersectionStuckParameters {
  double front_m = 3.79;
  double rear_m = 1.1;
  double width_m = 1.896;
  double stop_margin_m = 3.0;
  double sample_interval_m = 0.1;
  double velocity_threshold_mps = 0.833;
  double path_lateral_tolerance_m = 1.7;
  double minimum_conflict_arc_m = 0.0;
  std::optional<double> lane_entry_arc_m;
  // 다음 차로 초입까지 정체 차량을 찾을 때 적용하는 탐색 끝점입니다.
  std::optional<double> maximum_detection_arc_m;
  std::vector<double> multi_lane_anchor_arcs_m;
  std::vector<CrosswalkBounds> detection_lanes;
  std::vector<std::vector<CrosswalkPoint>> conflict_centerlines;
  bool insert_control_points = false;
  double ego_velocity_mps = 0.0;
  double ego_acceleration_mps2 = 0.0;
};

struct IntersectionStuckResult {
  Trajectory trajectory;
  std::optional<double> first_conflict_arc_m;
  std::optional<double> stop_arc_m;
  bool stuck_detected = false;
};

IntersectionStuckResult planIntersectionStuckStop(
    const Trajectory &path,
    const std::vector<CrosswalkBounds> &conflicting_lanes,
    const std::vector<IntersectionStuckObject> &objects,
    double ego_x, double ego_y,
    IntersectionStuckParameters parameters = {});

IntersectionStuckResult planIntersectionConflictStop(
    const Trajectory &path,
    const std::vector<CrosswalkBounds> &conflicting_lanes,
    double ego_x, double ego_y,
    IntersectionStuckParameters parameters = {});

}  // planning 네임스페이스
