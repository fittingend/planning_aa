#include "planning/behavior_goal_refinement.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
void check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

bool near(double first, double second) {
  return std::abs(first - second) < 1.0e-6;
}
}  // 익명 네임스페이스

int main() {
  planning::BehaviorGoalPath path;
  for (int index = 0; index <= 20; ++index) {
    planning::TrajectoryPoint point;
    point.x = index;
    point.z = 0.1 * index;
    point.longitudinal_velocity_mps = 5.0F;
    path.points.push_back(point);
    path.lane_ids.push_back(index >= 15 ?
        std::vector<std::int64_t>{10, 20} :
        std::vector<std::int64_t>{10});
  }
  path.left_bound.push_back({0.0, 2.0, 0.0});
  path.right_bound.push_back({0.0, -2.0, 0.0});
  const auto refined = planning::refineBehaviorGoalPath(
      path, {20.0, 0.0, 0.0}, 20, 7.5);
  check(refined && refined->points.size() == 15 &&
            near(refined->points[12].x, 12.0) &&
            near(refined->points[13].x, 19.0) &&
            near(refined->points[14].x, 20.0) &&
            near(refined->points[14].z, 2.0) &&
            near(refined->points[14].longitudinal_velocity_mps, 0.0),
        "검색 반경 밖의 마지막 점 뒤에 목표 전 점과 정지 목표를 붙입니다");
  check(refined->lane_ids[13] == std::vector<std::int64_t>({10, 20}) &&
            refined->lane_ids[14] == path.lane_ids.back() &&
            refined->left_bound.size() == 1 &&
            refined->right_bound.size() == 1,
        "생략한 경로의 차로 ID를 모으고 좌우 경계를 보존합니다");
  int validation_calls = 0;
  const auto retried = planning::refineFixedBehaviorGoalPath(
      path, {20.0, 0.0, 0.0}, 20, 7.5,
      [&](const auto &) { return ++validation_calls >= 3; });
  check(validation_calls == 3 && retried.points.size() == 17 &&
            near(retried.points[14].x, 14.0),
        "지도 유효성 검사가 거부하면 반경을 1m씩 줄입니다");
  const auto absent = planning::refineBehaviorGoalPath(
      path, {20.0, 0.0, 0.0}, 99, 7.5);
  check(!absent, "목표 차로 ID가 없으면 목표를 추가하지 않습니다");
  auto changing_speed = path;
  changing_speed.points[18].longitudinal_velocity_mps = 3.0F;
  changing_speed.points[19].longitudinal_velocity_mps = 4.0F;
  const auto stepped_speed = planning::refineBehaviorGoalPath(
      changing_speed, {20.0, 0.0, 0.0}, 20, 7.5);
  check(stepped_speed &&
            near(stepped_speed->points[stepped_speed->points.size() - 2]
                     .longitudinal_velocity_mps, 3.0),
        "원본은 목표 전 점의 속도에 선택된 선분의 시작 속도를 씁니다");
  const auto fallback = planning::refineFixedBehaviorGoalPath(
      path, {20.0, 0.0, 0.0}, 99, 7.5);
  check(fallback.points.size() == 21 &&
            near(fallback.points.back().longitudinal_velocity_mps, 0.0),
        "목표를 넣지 못해도 원본처럼 종점 속도를 0으로 만듭니다");

  using DecisionKind = planning::BehaviorGoalDecisionKind;
  planning::BehaviorGoalDecisionInput decision;
  decision.found_pull_over_path = true;
  decision.has_pull_over_path = true;
  decision.enable_safety_check = true;
  decision.is_current_safe = true;
  decision.keep_unsafe_time_s = 0.5;
  decision.distance_to_parking_start_m = 4.0;
  decision.decide_path_distance_m = 5.0;
  decision.use_object_recognition = true;
  auto state = planning::advanceBehaviorGoalDecisionState({}, decision);
  check(state.kind == DecisionKind::NotDecided &&
            !state.is_stable_safe && state.safe_start_time_s == 0.0,
        "첫 안전 관측은 즉시 승인하지 않고 안전 시작 시각을 기록합니다");
  decision.now_s = 0.5;
  state = planning::advanceBehaviorGoalDecisionState(state, decision);
  check(state.kind == DecisionKind::NotDecided,
        "안전 유지 시간의 정확한 경계에서는 결정하지 않습니다");
  decision.now_s = 0.6;
  state = planning::advanceBehaviorGoalDecisionState(state, decision);
  check(state.kind == DecisionKind::Deciding &&
            state.deciding_start_time_s == 0.6,
        "안전 유지 후 주차 시작점 근처에서 충돌 관찰 단계에 들어갑니다");
  decision.now_s = 1.6;
  state = planning::advanceBehaviorGoalDecisionState(state, decision);
  check(state.kind == DecisionKind::Deciding,
        "주차 경로 관찰 1초 경계는 아직 결정 전입니다");
  decision.now_s = 1.61;
  state = planning::advanceBehaviorGoalDecisionState(state, decision);
  check(state.kind == DecisionKind::Decided &&
            !state.deciding_start_time_s,
        "주차 경로를 1초 넘게 관찰하면 경로를 확정합니다");
  decision.found_pull_over_path = false;
  decision.is_current_safe = false;
  state = planning::advanceBehaviorGoalDecisionState(state, decision);
  check(state.kind == DecisionKind::Decided && !state.is_stable_safe,
        "확정된 경로의 상태는 안전 갱신 뒤에도 유지합니다");

  decision.found_pull_over_path = true;
  decision.is_current_safe = true;
  decision.is_activated = true;
  decision.now_s = 2.0;
  state = planning::advanceBehaviorGoalDecisionState({}, decision);
  check(state.kind == DecisionKind::Deciding,
        "활성 승인 상태에서는 안전 유지 대기 전에도 관찰을 시작합니다");
  decision.now_s = 2.1;
  decision.has_modified_goal = true;
  decision.is_goal_safe_with_margin = false;
  state = planning::advanceBehaviorGoalDecisionState(state, decision);
  check(state.kind == DecisionKind::NotDecided &&
            !state.deciding_start_time_s,
        "관찰 중 목표 충돌은 시작 시각을 지우고 결정을 취소합니다");

  decision.enable_safety_check = false;
  decision.use_object_recognition = false;
  decision.has_modified_goal = false;
  decision.distance_to_parking_start_m = 6.0;
  state = planning::advanceBehaviorGoalDecisionState({}, decision);
  check(state.kind == DecisionKind::NotDecided,
        "주차 시작점이 결정 거리 밖이면 기다립니다");
  decision.distance_to_parking_start_m = 5.0;
  state = planning::advanceBehaviorGoalDecisionState(state, decision);
  check(state.kind == DecisionKind::Decided &&
            !state.is_stable_safe && !state.safe_start_time_s,
        "객체 인식을 끄면 결정 거리 경계에서 즉시 확정하고 안전 상태를 초기화합니다");
  return EXIT_SUCCESS;
}
