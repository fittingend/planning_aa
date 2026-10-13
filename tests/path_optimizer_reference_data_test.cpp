// SPDX-License-Identifier: Apache-2.0
// 앞바퀴 방향 보정과 경계 침범 비용의 전파를 검사합니다.

#include "planning/path_optimizer_reference_data.hpp"

#include <cmath>
#include <stdexcept>

namespace {
void check(bool condition) {
  if (!condition) throw std::runtime_error("경로 최적화 기준점 정보 검사 실패");
}
}  // 네임스페이스

int main() {
  planning::PathOptimizerReferenceGeometry geometry;
  geometry.points.resize(50);
  geometry.curvature.resize(50, 0.0);
  geometry.arc_length_to_next.resize(50, 1.0);
  geometry.arc_length_to_next.back() = 0.0;
  for (std::size_t i = 0; i < 50; ++i) geometry.points[i].x = static_cast<double>(i);
  std::vector<planning::PathOptimizerBound> bounds(50, {-1.0, 1.0});
  bounds[25].upper = -0.25;
  const auto references = planning::buildPathOptimizerReferenceData(geometry, bounds);
  check(references.size() == 50);
  check(references[0].alpha == 0.0);
  check(references[25].avoidance_cost == 0.5);
  check(references[20].avoidance_cost == 0.5);
  check(references[15].avoidance_cost > 0.0);
  check(references[0].avoidance_cost == 0.0);

  geometry.points[1].x = 0.0;
  geometry.points[1].y = 1.0;
  geometry.points[2].x = 0.0;
  geometry.points[2].y = 2.0;
  planning::PathOptimizerAvoidanceParameters parameters;
  parameters.wheelbase = 1.5;
  const auto bent = planning::buildPathOptimizerReferenceData(
      geometry, bounds, parameters);
  check(std::abs(bent[0].alpha - 1.5707963267948966) < 1e-12);
}
