#include "planning/merge_from_private_core.hpp"

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
  planning::MergeFromPrivateCore core;
  const auto approach = core.apply(path, {conflict}, 0.0, 0.0, 5.0, 0.0, 0.0);
  check(approach.stopped && approach.stop_arc_m,
        "합류 전에 정지점을 만듭니다");
  const double stop = *approach.stop_arc_m;
  const auto first = core.apply(path, {conflict}, stop, 0.0, 0.0, 1.0, 0.0);
  const auto waiting = core.apply(path, {conflict}, stop, 0.0, 0.0, 1.8, 0.0);
  const auto release = core.apply(path, {conflict}, stop, 0.0, 0.0, 2.1, 0.0);
  const auto after = core.apply(path, {conflict}, stop, 0.0, 0.0, 2.2, 0.0);
  check(first.stopped && waiting.stopped && release.stopped,
        "1초 정차 완료 프레임까지 정지 경로를 유지합니다");
  check(!after.stopped && !after.stop_arc_m,
        "다음 프레임에 통과 상태를 유지합니다");

  planning::MergeFromPrivateCore moving;
  moving.apply(path, {conflict}, stop, 0.0, 0.0, 1.0, 0.0);
  moving.apply(path, {conflict}, stop, 0.0, 1.0, 1.5, 0.0);
  const auto restarted = moving.apply(path, {conflict}, stop, 0.0, 0.0, 1.6, 0.0);
  const auto still_waiting = moving.apply(path, {conflict}, stop, 0.0, 0.0, 2.1, 0.0);
  check(restarted.stopped && still_waiting.stopped,
        "움직이면 정차 시간을 다시 계산합니다");
}
