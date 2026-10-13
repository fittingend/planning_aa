// SPDX-License-Identifier: Apache-2.0
// 일반 가림 영역에서 보행자 출현 가능 지점과 통과 속도를 계산합니다.
#pragma once

#include "planning/crosswalk_occlusion.hpp"
#include "planning/obstacle_velocity_limiter_core.hpp"

#include <optional>
#include <vector>

namespace planning {

enum class OcclusionSpotDetectionMethod { OccupancyGrid, StoppedObjects };

struct OcclusionSpotParameters {
  OcclusionSpotDetectionMethod method =
      OcclusionSpotDetectionMethod::OccupancyGrid;
  double detection_length_m = 100.0;
  double maximum_lateral_distance_m = 5.0;
  double minimum_occlusion_size_m = 1.0;
  double pedestrian_speed_mps = 1.5;
  double pedestrian_radius_m = 0.3;
  double vehicle_front_m = 3.79;
  double vehicle_half_width_m = 0.948;
  double stuck_vehicle_speed_mps = 1.0;
  double safety_ratio = 0.8;
  double maximum_stop_jerk_mps3 = -0.3;
  double maximum_stop_acceleration_mps2 = -1.5;
  double maximum_slow_jerk_mps3 = -0.3;
  double maximum_slow_acceleration_mps2 = -1.5;
  double minimum_velocity_mps = 1.0;
  double safe_margin_m = 2.0;
  double response_delay_s = 0.0;
  int free_space_max = 43;
  int occupied_min = 57;
  bool use_moving_vehicle_ray_mask = true;
};

struct OcclusionSpotResult {
  Trajectory trajectory;
  std::vector<double> collision_arcs_m;
};

// 원본의 경로 길이 자르기와 1m Akima 위치 보간·속도 이전점 유지를 적용합니다.
Trajectory resampleOcclusionDetectionPath(
    const Trajectory &path, double maximum_length_m,
    double interval_m = 1.0);

OcclusionSpotResult planOcclusionSpot(
    const Trajectory &path, CrosswalkPoint ego,
    double ego_speed_mps, double ego_acceleration_mps2,
    const std::optional<CrosswalkOccupancyGrid> &grid,
    const std::vector<CrosswalkOcclusionMaskObject> &vehicles,
    const OcclusionSpotParameters &parameters = {},
    const std::vector<VelocityLimiterSegment> &partitions = {});

}  // planning 네임스페이스
