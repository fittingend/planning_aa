// SPDX-License-Identifier: Apache-2.0

#include "planning/behavior_dynamic_drivable_area.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}
}  // namespace

int main() {
  using namespace planning;
  try {
    Trajectory path(3);
    path[0].x = 0.0;
    path[1].x = 10.0;
    path[2].x = 20.0;
    const std::vector<PathOptimizerPoint2D> left{
        {0.0, 3.0}, {10.0, 3.0}, {20.0, 3.0}};
    const std::vector<PathOptimizerPoint2D> right{
        {0.0, -3.0}, {10.0, -3.0}, {20.0, -3.0}};
    const BehaviorSafetyPolygon obstacle{
        {8.0, 1.0}, {12.0, 1.0},
        {12.0, 4.0}, {8.0, 4.0}};
    const auto clipped = extractDynamicDrivableArea(
        path, left, right, {obstacle});
    check(clipped.applied_obstacles == 1 &&
              clipped.left_bound.size() > left.size(),
          "obstacle adds longitudinal boundary stations");
    check(clipped.right_bound.size() == right.size(),
          "right boundary is unaffected by left obstacle");
    bool narrowed = false;
    for (const auto &point : clipped.left_bound)
      narrowed |= point.x >= 8.0 && point.x <= 12.0 &&
                  point.y <= 1.001;
    check(narrowed, "left obstacle must narrow drivable area");
    const BehaviorSafetyPolygon crossing_center{
        {8.0, -2.0}, {12.0, -2.0},
        {12.0, 1.0}, {8.0, 1.0}};
    const auto explicit_side = extractDynamicDrivableArea(
        path, left, right, {crossing_center}, {true});
    check(explicit_side.right_bound.size() == right.size(),
          "collision side, not polygon centroid, selects boundary");
    bool left_crosses_center = false;
    for (const auto &point : explicit_side.left_bound)
      left_crosses_center |= point.x >= 8.0 && point.x <= 12.0 &&
                             point.y < -1.9;
    check(left_crosses_center,
          "explicit left collision follows the original side contract");
    const std::vector<PathOptimizerPoint2D> extended_left{
        {-2.0, 3.0}, {10.0, 3.0}, {22.0, 3.0}};
    const auto extended = extractDynamicDrivableArea(
        path, extended_left, right, {obstacle});
    check(extended.left_bound.front().x == -2.0 &&
              extended.left_bound.back().x == 22.0,
          "boundary endpoints beyond reference must be preserved");
    const auto unchanged = extractDynamicDrivableArea(path, left, right, {});
    check(unchanged.left_bound.size() == left.size() &&
              unchanged.right_bound.size() == right.size(),
          "no obstacle leaves boundaries unchanged");
    Trajectory corner(3);
    corner[0].x = 0.0;
    corner[1].x = 10.0;
    corner[2].x = 10.0;
    corner[2].y = 10.0;
    const std::vector<PathOptimizerPoint2D> corner_left{
        {0.0, 1.0}, {9.0, 1.0}, {9.0, 10.0}};
    const std::vector<PathOptimizerPoint2D> corner_right{
        {0.0, -1.0}, {11.0, -1.0}, {11.0, 10.0}};
    const BehaviorSafetyPolygon across_corner{
        {8.0, 0.6}, {12.0, 0.6},
        {12.0, 2.0}, {8.0, 2.0}};
    const auto curved = extractDynamicDrivableArea(
        corner, corner_left, corner_right,
        {across_corner}, {true});
    bool world_edge_at_corner = false;
    for (const auto &point : curved.left_bound)
      world_edge_at_corner |= std::abs(point.x - 9.0) < 1e-6 &&
                              std::abs(point.y - 1.418181818) < 1e-5;
    check(world_edge_at_corner,
          "corner slice intersects the world polygon edge at y=0.6");
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
