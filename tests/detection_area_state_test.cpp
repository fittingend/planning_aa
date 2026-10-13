#include "planning/detection_area_core.hpp"

#include <cstdlib>
#include <iostream>

namespace {

void check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

}  // 익명 네임스페이스

int main() {
  planning::DetectionAreaCore core(
      {10.0, -2.0, 10.0, 2.0},
      {{{11.0, -2.0}, {14.0, -2.0}, {14.0, 2.0}, {11.0, 2.0}}});
  planning::Trajectory path;
  for (int index = 0; index <= 20; ++index) {
    planning::TrajectoryPoint point;
    point.x = index;
    point.longitudinal_velocity_mps = 5.0;
    path.push_back(point);
  }
  check(!core.apply(path, 0.0, 0.0, 0.0, {{12.0, 3.0}}).stopped,
        "영역 밖의 장애물은 정지 원인이 아닙니다");
  const auto stopped = core.apply(path, 0.0, 0.0, 0.1, {{12.0, 0.0}});
  check(stopped.stopped && stopped.stop_arc_m &&
            *stopped.stop_arc_m > 6.0 && *stopped.stop_arc_m < 6.3,
        "영역 안의 장애물은 차량 앞부분을 고려해 정지합니다");
  check(core.apply(path, 0.0, 0.0, 2.09, {}).stopped,
        "마지막 관측 후 2초 미만에는 정지를 유지합니다");
  check(!core.apply(path, 0.0, 0.0, 2.1, {}).stopped,
        "마지막 관측 후 2초에 정지를 해제합니다");
  check(core.apply(path, 0.0, 0.0, 2.2, {{12.0, 0.0}}).stopped,
        "새 장애물이 나타나면 다시 정지합니다");
}
