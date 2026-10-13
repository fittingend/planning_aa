// SPDX-License-Identifier: Apache-2.0
// 목표 차로 중심선을 연결하고 차로 변경에 필요한 구간을 추출합니다.

#pragma once

#include "planning/behavior_path_shift.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace planning {

struct LaneChangeCenterline {
  std::int64_t lane_id = 0;
  Trajectory points;
};

struct LaneChangeMappedReference {
  Trajectory path;
  std::vector<std::vector<std::int64_t>> lane_ids;
  double start_arc_m = 0.0;
  double end_arc_m = 0.0;
};

// 지도에서 얻은 목표 차로 중심선과 차로 ID를 이어 경로 후보의 기준선을 만듭니다.
std::optional<LaneChangeMappedReference> buildLaneChangeMappedReference(
    const std::vector<LaneChangeCenterline> &centerlines,
    BehaviorPathShiftAnchor change_start, double change_length_m,
    double change_velocity_mps, double forward_path_length_m,
    double next_change_buffer_m,
    std::optional<BehaviorPathShiftAnchor> goal = std::nullopt);

struct LaneChangeRegulatoryDistances {
  std::optional<double> intersection_m;
  std::optional<double> crosswalk_m;
  std::optional<double> traffic_light_m;
};

// 활성 규제 요소 가운데 가장 가까운 전방 거리를 고릅니다.
std::optional<double> laneChangeNextRegulatoryDistance(
    const LaneChangeRegulatoryDistances &distances,
    bool regulate_intersection, bool regulate_crosswalk,
    bool regulate_traffic_light);

}  // planning 네임스페이스
