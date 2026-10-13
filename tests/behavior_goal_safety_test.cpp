#include "planning/behavior_goal_safety.hpp"

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

planning::BehaviorSafetyPolygon rectangle(
    double minimum_x, double maximum_x,
    double minimum_y, double maximum_y) {
  return {{minimum_x, minimum_y}, {maximum_x, minimum_y},
          {maximum_x, maximum_y}, {minimum_x, maximum_y}};
}
}  // 익명 네임스페이스

int main() {
  using Reason = planning::BehaviorGoalSafetyReason;
  planning::BehaviorGoalSafetyInput input;
  input.vehicle = {3.0, 1.0, 1.0};
  input.object_hard_margin_m = 0.5;
  input.longitudinal_margin_m = 3.0;
  input.object_polygons = {rectangle(5.5, 6.5, -0.5, 0.5)};
  check(planning::evaluateBehaviorGoalSafety(input) ==
            Reason::LongitudinalObjectTooClose,
        "차체 앞에서 2.5m 떨어진 같은 측방 영역 객체는 3m 종방향 여유로 거부합니다");
  input.object_polygons = {rectangle(5.5, 6.5, 3.0, 4.0)};
  check(planning::evaluateBehaviorGoalSafety(input) == Reason::Safe,
        "측방 여유 밖의 객체는 종방향 거리가 가까워도 제외합니다");
  input.object_polygons = {rectangle(3.2, 4.0, -0.5, 0.5)};
  check(planning::evaluateBehaviorGoalSafety(input) ==
            Reason::FootprintObjectCollision,
        "목표 차체 외곽 0.2m 앞의 객체는 0.5m 여유에서 먼저 거부합니다");
  input.object_polygons.clear();
  input.use_occupancy_grid = true;
  check(planning::evaluateBehaviorGoalSafety(
            input, [](auto pose) { return std::abs(pose.x) < 1e-9; }) ==
            Reason::GoalGridCollision,
        "후보 갱신은 목표 자세의 점유 격자 충돌을 먼저 검사합니다");
  input.mode = planning::BehaviorGoalSafetyMode::Recheck;
  check(planning::evaluateBehaviorGoalSafety(
            input, [](auto pose) { return std::abs(pose.x) < 1e-9; }) ==
            Reason::Safe,
        "승인 전 재검사는 목표 자세의 점유 격자 재검사를 생략합니다");
  input.use_object_recognition = false;
  check(planning::evaluateBehaviorGoalSafety(
            input) == Reason::Safe,
        "객체 인식을 끈 승인 전 재검사는 다른 안전 입력 없이 통과합니다");
  input.mode = planning::BehaviorGoalSafetyMode::CandidateUpdate;
  check(planning::evaluateBehaviorGoalSafety(
            input, [](auto) { return true; }) ==
            Reason::GoalGridCollision,
        "객체 인식을 꺼도 최초 후보 갱신의 점유 격자 검사는 유지합니다");
  input.use_object_recognition = true;
  input.use_occupancy_for_longitudinal = true;
  input.longitudinal_margin_m = 2.0;
  input.occupancy_grid_margin_m = 0.5;
  check(planning::evaluateBehaviorGoalSafety(
            input, [](auto pose) {
              return pose.x > 1.4 && pose.x < 1.6;
            }) == Reason::LongitudinalGridCollision,
        "종방향 여유에서 격자 여유를 뺀 1.5m 앞 자세도 검사합니다");
  input.use_occupancy_grid = false;
  input.use_occupancy_for_longitudinal = false;
  input.longitudinal_margin_m = 0.0;
  input.object_hard_margin_m = 0.4;
  input.object_polygons = {rectangle(3.34, 4.0, -0.5, 0.5)};
  check(planning::evaluateBehaviorGoalSafety(input) ==
            Reason::FootprintObjectCollision,
        "최초 후보의 객체 외곽 여유를 적용합니다");
  input.mode = planning::BehaviorGoalSafetyMode::Recheck;
  input.margin_scale_factor = 0.8;
  check(planning::evaluateBehaviorGoalSafety(input) == Reason::Safe,
        "승인 전 재검사의 축소 여유에서는 같은 객체를 허용합니다");
  return EXIT_SUCCESS;
}
