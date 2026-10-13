#include "planning/behavior_frenet_sampling.hpp"

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
  planning::BehaviorFrenetSamplingInput input;
  for (int x = 0; x <= 40; ++x) {
    planning::TrajectoryPoint point;
    point.x = x;
    point.longitudinal_velocity_mps = 5.0F;
    input.reference.push_back(point);
  }
  input.initial.s_m = 5.0;
  input.target_lengths_m = {10.0, 20.0};
  input.target_lateral_positions_m = {0.0, 2.0};
  input.target_lateral_derivatives = {0.0};
  input.resolution_m = 0.5;
  planning::TrajectoryPoint ego;
  ego.x = 5.25;
  ego.y = 1.0;
  const auto projected = planning::projectBehaviorFrenetState(
      input.reference, ego);
  check(projected && std::abs(projected->s_m - 5.25) < 0.02 &&
            std::abs(projected->lateral_m - 1.0) < 0.02 &&
            std::abs(projected->lateral_derivative) < 1.0e-6,
        "기준 스플라인의 최근접 호길이와 측방 거리를 투영합니다");
  const auto candidates = planning::generateBehaviorFrenetCandidates(input);
  check(candidates.size() == 4 && candidates[0].path.size() == 20 &&
            candidates[2].path.size() == 40,
        "목표 길이와 측방 위치 조합에서 표본 경로를 생성합니다");
  check(std::abs(candidates[1].path.front().y) < 0.01 &&
            std::abs(candidates[1].path.back().y - 2.0) < 1.0e-8 &&
            std::abs(candidates[1].path.back().x - 15.0) < 1.0e-8,
        "직선 기준 경로의 측방 이동은 목표 위치에 도달합니다");
  check(candidates[1].curvatures_per_m.size() ==
            candidates[1].path.size() &&
            candidates[1].arc_lengths_m.size() ==
                candidates[1].path.size(),
        "후보의 호길이와 곡률을 모든 점에 계산합니다");
  input.reference[10] = input.reference[9];
  check(planning::generateBehaviorFrenetCandidates(input).empty(),
        "중복 기준점은 스플라인 입력에서 거부합니다");
}
