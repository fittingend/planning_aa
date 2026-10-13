#include "planning/analytical_velocity_smoother.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

void check(bool value, const char *message) {
  if (!value) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

}  // 익명 네임스페이스

int main() {
  planning::Trajectory straight(11);
  for (std::size_t index = 0; index < straight.size(); ++index) {
    straight[index].x = static_cast<double>(index);
    straight[index].longitudinal_velocity_mps = 10.0F;
  }
  const auto forward = planning::applyAnalyticalForwardJerkFilter(
      straight, 0, 2.0, 0.0);
  check(std::abs(forward.front().longitudinal_velocity_mps - 2.0F) < 1.0e-6 &&
            forward[1].longitudinal_velocity_mps >= 2.0F &&
            forward.back().longitudinal_velocity_mps > 2.0F &&
            forward.back().longitudinal_velocity_mps < 10.0F,
        "현재 속도에서 시작해 전방 저크 제한으로 점진적으로 가속합니다");
  const double first_dt = 1.0 / 2.0;
  check(std::abs(forward[1].acceleration_mps2 -
                 0.3 * first_dt) < 1.0e-6,
        "첫 점의 최대 양의 저크는 0.3m/s³입니다");

  planning::Trajectory curve(3);
  curve[0].x = 0.0;
  curve[0].y = 0.0;
  curve[1].x = 5.0;
  curve[1].y = 0.0;
  curve[2].x = 5.0;
  curve[2].y = 5.0;
  for (auto &point : curve) point.longitudinal_velocity_mps = 10.0F;
  const auto lateral = planning::applyAnalyticalLateralAccelerationLimit(
      curve);
  check(lateral.size() > curve.size() &&
            lateral[lateral.size() / 2].longitudinal_velocity_mps < 10.0F &&
            lateral[lateral.size() / 2].longitudinal_velocity_mps >= 2.0F,
        "굽은 경로의 속도를 횡가속도 상한 안으로 낮춥니다");
  planning::Trajectory stopping(81);
  for (std::size_t index = 0; index < stopping.size(); ++index) {
    stopping[index].x = static_cast<double>(index);
    stopping[index].longitudinal_velocity_mps = 8.0F;
  }
  stopping.back().longitudinal_velocity_mps = 0.0F;
  const auto stopped = planning::smoothAnalyticalVelocity(
      stopping, 6.0, 0.0);
  check(stopped.size() == stopping.size() &&
            std::abs(stopped.front().longitudinal_velocity_mps - 6.0F) < 1.0e-6 &&
            std::abs(stopped.back().longitudinal_velocity_mps) < 0.3F &&
            stopped[70].longitudinal_velocity_mps <
                stopped[40].longitudinal_velocity_mps,
        "앞쪽 속도와 종점 정지를 저크 제한으로 연결합니다");
  return EXIT_SUCCESS;
}
