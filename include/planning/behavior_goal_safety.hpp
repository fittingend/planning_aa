// SPDX-License-Identifier: Apache-2.0
// 목표·주차 자세의 객체·점유 격자 안전 판정을 ROS 없이 계산합니다.

#pragma once

#include "planning/behavior_goal_refinement.hpp"
#include "planning/behavior_path_object_safety.hpp"

#include <functional>
#include <vector>

namespace planning {

enum class BehaviorGoalSafetyMode { CandidateUpdate, Recheck };

enum class BehaviorGoalSafetyReason {
  Safe,
  GoalGridCollision,
  FootprintObjectCollision,
  LongitudinalGridCollision,
  LongitudinalObjectTooClose,
};

struct BehaviorGoalSafetyInput {
  BehaviorGoalPose goal;
  BehaviorSafetyFootprint vehicle;
  std::vector<BehaviorSafetyPolygon> object_polygons;
  bool use_object_recognition = true;
  bool use_occupancy_grid = false;
  bool use_occupancy_for_longitudinal = false;
  double object_hard_margin_m = 0.0;
  double longitudinal_margin_m = 0.0;
  double occupancy_grid_margin_m = 0.0;
  double margin_scale_factor = 1.0;
  BehaviorGoalSafetyMode mode = BehaviorGoalSafetyMode::CandidateUpdate;
};

// 점유 격자 조회는 호출자가 지도 좌표계 변환까지 적용한 함수로 제공합니다.
BehaviorGoalSafetyReason evaluateBehaviorGoalSafety(
    const BehaviorGoalSafetyInput &input,
    const std::function<bool(BehaviorGoalPose)> &occupancy_collision = {});

}  // planning 네임스페이스
