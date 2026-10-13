// SPDX-License-Identifier: Apache-2.0
// 기준점 자르기·방향·곡률 계산을 직선과 원호에서 검사합니다.

#include "planning/path_optimizer_reference_geometry.hpp"

#include <cmath>
#include <stdexcept>

namespace {
void check(bool condition) {
  static int number = 0;
  ++number;
  if (!condition) throw std::runtime_error(
      "경로 최적화 기준 경로 검사 실패: " + std::to_string(number));
}
}  // 네임스페이스

int main() {
  planning::Trajectory line;
  for (int i = 0; i <= 600; ++i) {
    planning::TrajectoryPoint point;
    point.x = static_cast<double>(i) * 0.5;
    point.longitudinal_velocity_mps = 10.0F;
    line.push_back(point);
  }
  planning::TrajectoryPoint ego;
  ego.x = 50.0;
  const auto straight = planning::preparePathOptimizerReferenceGeometry(line, ego);
  check(straight.points.size() == 100);
  check(straight.curvature.size() == 100);
  check(straight.arc_length_to_next.size() == 100);
  check(straight.points.front().x >= 43.0 && straight.points.front().x <= 45.0);
  check(straight.points.back().x > 140.0);
  for (std::size_t i = 0; i < straight.points.size(); ++i) {
    check(std::abs(straight.points[i].y) < 1e-10);
    check(std::abs(straight.curvature[i]) < 1e-10);
    if (i + 1 < straight.points.size()) {
      check(std::abs(straight.arc_length_to_next[i] - 1.0) < 1e-8);
    }
  }
  auto previous = straight;
  previous.optimized_states.resize(previous.points.size());
  for (std::size_t i = 0; i < previous.points.size(); ++i)
    previous.optimized_states[i] = std::array<double, 2>{0.1, 0.02};
  ego.x = 51.0;
  const auto following = planning::preparePathOptimizerReferenceGeometry(
      line, ego, {}, &previous);
  check(following.fixed_states.size() == following.points.size());
  check(following.fixed_states.front().has_value());
  check(std::abs((*following.fixed_states.front())[0] - 0.1) < 1e-12);

  planning::Trajectory circle;
  for (int i = 0; i <= 200; ++i) {
    const double angle = static_cast<double>(i) * 0.005;
    planning::TrajectoryPoint point;
    point.x = 20.0 * std::sin(angle);
    point.y = 20.0 * (1.0 - std::cos(angle));
    point.longitudinal_velocity_mps = 5.0F;
    circle.push_back(point);
  }
  ego.x = circle[80].x;
  ego.y = circle[80].y;
  const auto curved = planning::preparePathOptimizerReferenceGeometry(circle, ego);
  check(curved.points.size() >= 15);
  check(curved.curvature.size() == curved.points.size());
  check(std::abs(curved.curvature[10] - 0.05) < 0.005);
}
