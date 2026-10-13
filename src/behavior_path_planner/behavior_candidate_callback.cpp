// SPDX-License-Identifier: Apache-2.0
// 같은 인식 메시지에서 출발 객체와 목표 객체 외곽을 조립합니다.

#include "planning/behavior_candidate_callback.hpp"

#include <stdexcept>

namespace planning {

BehaviorCandidateCallbackResult evaluateBehaviorCandidateCallback(
    const BehaviorCandidateCallbackInput &input,
    const std::function<bool(BehaviorGoalPose)> &occupancy_collision) {
  if (input.callback_time_ns < input.perception.receipt_time_ns ||
      input.maximum_perception_age_ns < 0 ||
      input.callback_time_ns - input.perception.receipt_time_ns >
          input.maximum_perception_age_ns) {
    throw std::invalid_argument("후보 콜백의 인식 입력 시각이 유효하지 않습니다");
  }
  auto start = input.start;
  auto goal = input.goal;
  start.objects = input.perception.objects;
  goal.object_polygons.clear();
  goal.object_polygons.reserve(input.perception.objects.size());
  for (const auto &object : input.perception.objects) {
    goal.object_polygons.push_back(
        behaviorSafetyWorldPolygon(object.pose, object.shape));
  }
  BehaviorCandidateCallbackResult result;
  result.perception_object_count = input.perception.objects.size();
  result.start_candidates = planStartPullOutPoseCandidates(start);
  result.goal_safety = evaluateBehaviorGoalSafety(
      goal, occupancy_collision);
  return result;
}

}  // planning 네임스페이스
