// SPDX-License-Identifier: Apache-2.0
// 직선과 대각선 차선 경계의 부호·차량 폭 보정을 검사합니다.

#include "planning/path_optimizer_bounds.hpp"

#include <cmath>
#include <stdexcept>

namespace {
void check(bool condition) {
  if (!condition) throw std::runtime_error("경로 최적화 차선 경계 검사 실패");
}
}  // 네임스페이스

int main() {
  planning::Trajectory reference(3);
  for (std::size_t i = 0; i < reference.size(); ++i) {
    reference[i].x = static_cast<double>(i);
  }
  const std::vector<planning::PathOptimizerPoint2D> left{{-1.0, 2.0}, {4.0, 2.0}};
  const std::vector<planning::PathOptimizerPoint2D> right{{-1.0, -2.0}, {4.0, -2.0}};
  const auto bounds = planning::pathOptimizerCenterBounds(
      reference, left, right, 2.0, 0.1);
  check(bounds.size() == 3);
  for (const auto &bound : bounds) {
    check(std::abs(bound.lower + 0.9) < 1e-12);
    check(std::abs(bound.upper - 0.9) < 1e-12);
  }
  const std::vector<planning::PathOptimizerPoint2D> diagonal{
      {-1.0, 1.0}, {4.0, 3.5}};
  const auto angled = planning::pathOptimizerCenterBounds(
      reference, diagonal, right, 2.0, 0.1);
  check(std::abs(angled[0].upper - 0.9) < 1e-12);
  check(std::abs(angled[2].upper - 1.4) < 1e-12);

  planning::PathOptimizerReferenceGeometry geometry;
  geometry.points = reference;
  geometry.curvature = {0.0, 0.0, 0.0};
  geometry.arc_length_to_next = {1.0, 1.0, 0.0};
  std::vector<planning::PathOptimizerReference> references(3);
  const std::vector<planning::PathOptimizerCircle> circles{
      {-1.0, 0.5}, {0.0, 0.5}, {1.0, 0.5}};
  planning::attachPathOptimizerVehicleBounds(
      geometry, bounds, circles, references);
  for (const auto &point : references) {
    check(point.beta.size() == 3);
    check(point.circle_bounds.size() == 3);
    for (std::size_t i = 0; i < 3; ++i) {
      check(std::abs(point.beta[i]) < 1e-12);
      check(std::abs(point.circle_bounds[i].lower + 0.9) < 1e-12);
      check(std::abs(point.circle_bounds[i].upper - 0.9) < 1e-12);
    }
  }
  auto narrow = bounds;
  narrow[1] = {0.3, 0.5};
  planning::keepPathOptimizerMinimumBoundsWidth(
      narrow, geometry.curvature, 1.0, 2.0, 3.0);
  check(std::abs(narrow[1].lower + 0.5) < 1e-12);
  check(std::abs(narrow[1].upper - 0.5) < 1e-12);
  const std::vector<planning::PathOptimizerBound> violated{
      {-1.0, 1.0}, {0.5, 1.0}, {-1.0, 1.0}};
  const auto extended = planning::extendPathOptimizerViolatedBounds(
      violated, 1.0, 1.0);
  check(std::abs(extended[0].lower - 0.5) < 1e-12);
  check(std::abs(extended[2].lower - 0.5) < 1e-12);
  auto reused = bounds;
  const std::vector<planning::PathOptimizerBound> old_bounds{
      {-0.1, 0.1}, {-0.2, 0.2}, {-0.3, 0.3}};
  planning::TrajectoryPoint ego;
  ego.x = 1.0;
  planning::reusePathOptimizerPreviousBounds(
      reused, geometry, ego, 1.0, geometry, old_bounds, 2.0, 1.0);
  check(std::abs(reused[0].upper - 0.1) < 1e-12);
  check(std::abs(reused[1].upper - 0.2) < 1e-12);
  check(std::abs(reused[2].upper - 0.3) < 1e-12);

  auto crossing_previous = geometry;
  crossing_previous.points.insert(
      crossing_previous.points.begin(), geometry.points.front());
  crossing_previous.points.front().orientation_z = 1.0;
  crossing_previous.points.front().orientation_w = 0.0;
  auto crossing_reused = bounds;
  const std::vector<planning::PathOptimizerBound> crossing_bounds{
      {-0.05, 0.05}, {-0.15, 0.15}, {-0.25, 0.25},
      {-0.35, 0.35}};
  planning::reusePathOptimizerPreviousBounds(
      crossing_reused, geometry, ego, 1.0, crossing_previous,
      crossing_bounds, 2.0, 1.0);
  check(std::abs(crossing_reused[0].upper - 0.15) < 1e-12);
}
