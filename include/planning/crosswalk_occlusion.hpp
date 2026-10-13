// Copyright 2024 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 횡단보도 가림 영역의 점유 격자 판정과 감속 상태를 계산합니다.

#pragma once

#include "planning/crosswalk_core.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace planning {

struct CrosswalkOccupancyGrid {
  std::size_t width = 0;
  std::size_t height = 0;
  double resolution_m = 1.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw_rad = 0.0;
  std::vector<std::int8_t> values;
};

struct CrosswalkOcclusionParameters {
  int free_space_max = 43;
  int occupied_min = 58;
  double minimum_size_m = 1.0;
  double time_buffer_s = 0.5;
  double assumed_object_speed_mps = 1.0;
  double minimum_slow_speed_mps = 1.0;
  double safety_slow_distance_m = 2.0;
  double vehicle_front_m = 3.79;
  double maximum_slow_jerk_mps3 = -1.5;
  double maximum_slow_acceleration_mps2 = -2.5;
  double no_relax_speed_mps = 2.78;
  bool ignore_with_traffic_light = true;
  bool ignore_behind_dynamic_objects = true;
  double object_inflate_size_m = 0.5;
  double maximum_distance_to_path_m = 0.0;
};

struct CrosswalkOcclusionMaskObject {
  CrosswalkPose2d pose;
  double length_m = 0.0;
  double width_m = 0.0;
  double longitudinal_speed_mps = 0.0;
  double minimum_mask_speed_mps = 0.5;
};

std::vector<std::vector<CrosswalkPoint>> crosswalkDetectionAreas(
    const std::vector<CrosswalkPoint> &left_bound,
    const std::vector<CrosswalkPoint> &right_bound,
    CrosswalkPoint crossing,
    double detection_range_m);

double crosswalkDetectionRange(
    double assumed_object_speed_mps,
    double distance_to_crosswalk_m, double ego_velocity_mps);

bool isCrosswalkOccluded(
    const CrosswalkOccupancyGrid &grid,
    const std::vector<std::vector<CrosswalkPoint>> &detection_areas,
    CrosswalkOcclusionParameters parameters = {},
    const std::vector<CrosswalkOcclusionMaskObject> &objects = {},
    const std::vector<CrosswalkPoint> &path = {});

double crosswalkFeasibleSlowVelocity(
    double jerk_mps3, double acceleration_limit_mps2,
    double ego_acceleration_mps2, double ego_velocity_mps,
    double distance_to_target_m);

class CrosswalkOcclusionPlanner {
 public:
  explicit CrosswalkOcclusionPlanner(CrosswalkOcclusionParameters parameters = {});

  bool update(
      double now_s, bool occluded, bool enabled,
      bool crosswalk_has_traffic_light, bool map_skips_occlusion,
      bool ego_on_crosswalk);

  Trajectory applySlowdown(
      const Trajectory &path, double entry_arc_m, double exit_arc_m,
      double ego_arc_m, double ego_velocity_mps,
      double ego_acceleration_mps2);

 private:
  CrosswalkOcclusionParameters parameters_;
  std::optional<double> initial_occlusion_s_;
  std::optional<double> most_recent_occlusion_s_;
  bool passed_slow_point_ = false;
};

}  // planning 네임스페이스
