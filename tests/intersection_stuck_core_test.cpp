#include "planning/intersection_stuck_core.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

void check(bool valid, const char *message) {
  if (!valid) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

}  // 익명 네임스페이스

int main() {
  planning::Trajectory path(31);
  for (std::size_t i = 0; i < path.size(); ++i) {
    path[i].x = static_cast<double>(i);
    path[i].longitudinal_velocity_mps = 5.0F;
  }
  const planning::CrosswalkBounds conflict{
      {{10.0, 2.0}, {12.0, 2.0}},
      {{10.0, -2.0}, {12.0, -2.0}}};
  const auto stopped = planning::planIntersectionStuckStop(
      path, {conflict}, {{{15.0, 0.0}, 0.0, true}}, 0.0, 0.0);
  check(stopped.stuck_detected && stopped.first_conflict_arc_m &&
            stopped.stop_arc_m &&
            std::abs(*stopped.stop_arc_m - 3.3) < 0.2,
        "정체 차량이 있으면 첫 교차 영역 앞에 정지합니다");
  const auto fast = planning::planIntersectionStuckStop(
      path, {conflict}, {{{15.0, 0.0}, 2.0, true}}, 0.0, 0.0);
  check(!fast.stuck_detected && !fast.stop_arc_m,
        "움직이는 차량은 정체 차량에서 제외합니다");
  const auto outside = planning::planIntersectionStuckStop(
      path, {conflict}, {{{15.0, 5.0}, 0.0, true}}, 0.0, 0.0);
  check(!outside.stuck_detected && !outside.stop_arc_m,
        "경로 바깥의 차량은 정체 차량에서 제외합니다");
  const auto pedestrian = planning::planIntersectionStuckStop(
      path, {conflict}, {{{15.0, 0.0}, 0.0, false}}, 0.0, 0.0);
  check(!pedestrian.stuck_detected && !pedestrian.stop_arc_m,
        "대상 차량 분류가 아니면 정체 판단에서 제외합니다");
}
