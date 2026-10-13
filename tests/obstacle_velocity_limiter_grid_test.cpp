// SPDX-License-Identifier: Apache-2.0
// 격자 임계값, 마스크와 윤곽 좌표 변환을 검사합니다.

#include "planning/obstacle_velocity_limiter_grid.hpp"

#include <cmath>
#include <stdexcept>

namespace {
void check(bool condition) {
  if (!condition) throw std::runtime_error("점유 격자 장애물 검사 실패");
}
}  // 익명 네임스페이스

int main() {
  planning::VelocityLimiterOccupancyGrid grid;
  grid.width = 20;
  grid.height = 20;
  grid.resolution = 1.0;
  grid.origin = {100.0, 200.0};
  grid.cells.assign(400, 0);
  for (int y = 5; y <= 8; ++y)
    for (int x = 5; x <= 8; ++x)
      grid.cells[y * 20 + x] = 90;
  grid.cells[2 * 20 + 2] = 30;
  auto obstacles = planning::extractVelocityLimiterGridObstacles(
      grid, 50);
  check(!obstacles.empty());
  for (const auto &segment : obstacles) {
    check(segment.from.x >= 104.0 && segment.from.x <= 109.0);
    check(segment.from.y >= 204.0 && segment.from.y <= 209.0);
  }
  planning::Trajectory path(10);
  std::vector<double> times(10);
  for (std::size_t i = 0; i < path.size(); ++i) {
    path[i].x = 100.0 + i;
    path[i].y = 206.0;
    path[i].longitudinal_velocity_mps = 10.0F;
    times[i] = 0.1 * i;
  }
  planning::ObstacleVelocityLimiterParameters parameters;
  parameters.downsample_factor = 1;
  const auto plan = planning::planVelocityLimiterOccupancyGrid(
      path, times, 0, 0.0, grid, 50, {}, parameters);
  check(!plan.slowdowns.empty());
  planning::VelocityLimiterObstacleMasks masks;
  masks.positive = {{100.0, 200.0}, {110.0, 200.0},
                    {110.0, 210.0}, {100.0, 210.0}};
  check(!planning::extractVelocityLimiterGridObstacles(
      grid, 50, masks).empty());
  masks.negative = {{{104.0, 204.0}, {110.0, 204.0},
                     {110.0, 210.0}, {104.0, 210.0}}};
  check(planning::extractVelocityLimiterGridObstacles(
      grid, 50, masks).empty());
  planning::VelocityLimiterOccupancyGrid single = grid;
  single.cells.assign(400, 0);
  single.cells[6 * 20 + 5] = 90;
  const auto single_obstacle =
      planning::extractVelocityLimiterGridObstacleGeometry(single, 50);
  check(single_obstacle.points.size() == 1 &&
        single_obstacle.segments.empty());
  check(std::abs(single_obstacle.points.front().x - 105.0) < 1e-9 &&
        std::abs(single_obstacle.points.front().y - 206.0) < 1e-9);
  check(planning::planVelocityLimiterOccupancyGrid(
      path, times, 0, 0.0, single, 50, {}, parameters).slowdowns.empty());
}
