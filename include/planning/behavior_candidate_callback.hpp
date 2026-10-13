// SPDX-License-Identifier: Apache-2.0
// 출발·목표 후보에 같은 콜백의 인식 객체 묶음을 전달합니다.

#pragma once

#include "planning/behavior_goal_safety.hpp"
#include "planning/behavior_path_lane_safety.hpp"

#include <cstdint>
#include <functional>

namespace planning {

struct BehaviorCandidatePerceptionBatch {
  std::int64_t receipt_time_ns = 0;
  std::vector<StartPullOutStopObject> objects;
};

struct BehaviorCandidateCallbackInput {
  std::int64_t callback_time_ns = 0;
  std::int64_t maximum_perception_age_ns = 0;
  BehaviorCandidatePerceptionBatch perception;
  StartPullOutCandidatePlanInput start;
  BehaviorGoalSafetyInput goal;
};

struct BehaviorCandidateCallbackResult {
  std::vector<BehaviorSafetyPose> start_candidates;
  BehaviorGoalSafetyReason goal_safety = BehaviorGoalSafetyReason::Safe;
  std::size_t perception_object_count = 0;
};

// 한 번 고른 인식 메시지를 두 후보 계산에 함께 적용하고 시각 범위를 검증합니다.
BehaviorCandidateCallbackResult evaluateBehaviorCandidateCallback(
    const BehaviorCandidateCallbackInput &input,
    const std::function<bool(BehaviorGoalPose)> &occupancy_collision = {});

}  // planning 네임스페이스
