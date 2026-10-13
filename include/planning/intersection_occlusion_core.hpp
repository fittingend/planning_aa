// Copyright 2024 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 교차로 주목 영역의 점유 격자 가림을 ROS와 지도 어댑터 없이 판정합니다.

#pragma once

#include "planning/crosswalk_occlusion.hpp"

#include <optional>

namespace planning {

struct IntersectionOcclusionParameters {
  // 원본의 1m 잡음 제거와 후보 객체 면적 조건을 정사각형으로 근사한 값입니다.
  double minimum_unknown_square_m = 3.5;
  // 차로 탐색선이 없는 단독 호출에서만 사용하는 평면 거리 제한입니다.
  double maximum_distance_to_path_m = 53.0;
  // 지도 탐색선이 있을 때 교차점부터 차로 방향으로 누적할 원본 제한입니다.
  double maximum_clearance_distance_m = 55.0;
  // 이 속도보다 느린 차량만 탐색선 뒤쪽 미확인 격자를 가립니다.
  double parked_vehicle_speed_threshold_mps = 0.8333;
};

struct IntersectionOcclusionBlocker {
  CrosswalkPose2d pose;
  double length_m = 3.0;
  double width_m = 3.0;
  double speed_mps = 0.0;
};

enum class IntersectionOcclusionStatus {
  Clear, Static, Dynamic
};

struct IntersectionOcclusionReleaseParameters {
  double static_release_delay_s = 0.5;
};

// 원본의 기본 정지선 통과와 정적 가림 해제 타이머를 차로별로 유지합니다.
class IntersectionOcclusionReleaseState {
 public:
  explicit IntersectionOcclusionReleaseState(
      IntersectionOcclusionReleaseParameters parameters = {});

  bool update(IntersectionOcclusionStatus status, bool has_traffic_light,
              bool collision, bool stopped_at_default_line,
              bool stopped_at_peeking_line, double now_s);
  void reset();

 private:
  IntersectionOcclusionReleaseParameters parameters_;
  bool passed_default_line_ = false;
  bool released_ = false;
  std::optional<double> static_wait_started_s_;
};

// 가장 가까운 가림과 자차 사이의 가시 영역에 객체가 있으면 동적 가림입니다.
IntersectionOcclusionStatus classifyIntersectionOcclusion(
    const CrosswalkOccupancyGrid &grid,
    const std::vector<std::vector<CrosswalkPoint>> &attention_areas,
    const Trajectory &ego_path, CrosswalkPoint ego_position,
    IntersectionOcclusionParameters parameters = {},
    const std::vector<std::vector<CrosswalkPoint>> &lane_divisions = {},
    const std::vector<IntersectionOcclusionBlocker> &attention_vehicles = {});

bool detectIntersectionOcclusion(
    const CrosswalkOccupancyGrid &grid,
    const std::vector<std::vector<CrosswalkPoint>> &attention_areas,
    const Trajectory &ego_path,
    IntersectionOcclusionParameters parameters = {},
    const std::vector<std::vector<CrosswalkPoint>> &lane_divisions = {},
    const std::vector<IntersectionOcclusionBlocker> &parked_vehicles = {});

}  // planning 네임스페이스
