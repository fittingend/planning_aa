#include "planning/walkway_core.hpp"

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

planning::Trajectory path() {
  planning::Trajectory result(21);
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index].x = static_cast<double>(index);
    result[index].longitudinal_velocity_mps = 5.0F;
  }
  return result;
}

}  // 익명 네임스페이스

int main() {
  const std::vector<planning::CrosswalkPoint> polygon{
      {10.0, -2.0}, {12.0, -2.0}, {12.0, 2.0}, {10.0, 2.0}};
  planning::WalkwayCore walkway(polygon);
  const auto approaching = walkway.apply(path(), 0.0, 0.0, false, 0.0);
  check(approaching.stop_arc_m &&
            std::abs(*approaching.stop_arc_m - 2.71) < 1.0e-9 &&
            approaching.state == planning::WalkwayState::Approach,
        "기본 보행로 정지점은 진입 전 7.29m입니다");
  check(approaching.trajectory.size() == 22 &&
            approaching.trajectory[3].longitudinal_velocity_mps == 0.0F,
        "정지점 뒤쪽 속도를 0으로 제한합니다");
  const auto first_stop = walkway.apply(path(), 2.5, 0.0, true, 1.0);
  check(first_stop.state == planning::WalkwayState::Approach,
        "정차 지속 시간 전에는 접근 상태입니다");
  const auto held_stop = walkway.apply(path(), 2.5, 0.0, true, 1.11);
  check(held_stop.state == planning::WalkwayState::Stop &&
            held_stop.stop_arc_m,
        "0.1초 정차하면 정지 상태로 전환하되 해당 출력은 정지시킵니다");
  const auto surpassed = walkway.apply(path(), 2.5, 0.0, true, 1.2);
  check(surpassed.state == planning::WalkwayState::Surpassed &&
            !surpassed.stop_arc_m &&
            surpassed.trajectory[3].longitudinal_velocity_mps == 5.0F,
        "다음 입력에서 통과 상태로 넘어가 정지를 해제합니다");

  planning::WalkwayCore with_line(polygon, {{9.0, -2.0, 9.0, 2.0}});
  const auto explicit_stop = with_line.apply(path(), 0.0, 0.0, false, 0.0);
  check(explicit_stop.stop_arc_m &&
            std::abs(*explicit_stop.stop_arc_m - 5.21) < 1.0e-9,
        "명시적 정지선에는 차량 전방 길이만 적용합니다");
}
