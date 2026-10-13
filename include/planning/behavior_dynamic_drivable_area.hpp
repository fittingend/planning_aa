// SPDX-License-Identifier: Apache-2.0
// 동적 장애물의 제외 다각형을 최적화 입력의 좌우 주행 경계로 변환합니다.

#pragma once

#include "planning/behavior_path_object_safety.hpp"
#include "planning/path_optimizer_bounds.hpp"
#include "planning/trajectory.hpp"

#include <cstddef>
#include <vector>

namespace planning {

struct BehaviorDynamicDrivableArea {
  std::vector<PathOptimizerPoint2D> left_bound;
  std::vector<PathOptimizerPoint2D> right_bound;
  std::size_t applied_obstacles = 0;
};

// 경로 호길이와 측방 좌표로 다각형을 투영합니다. 지도 경계의 전진 순서가
// 유지되는 구간에만 적용하고, 투영이 모호한 경계는 원본을 반환합니다.
BehaviorDynamicDrivableArea extractDynamicDrivableArea(
    const Trajectory &reference,
    const std::vector<PathOptimizerPoint2D> &left_bound,
    const std::vector<PathOptimizerPoint2D> &right_bound,
    const std::vector<BehaviorSafetyPolygon> &exclusion_polygons,
    const std::vector<bool> &collision_on_left = {});

}  // namespace planning
