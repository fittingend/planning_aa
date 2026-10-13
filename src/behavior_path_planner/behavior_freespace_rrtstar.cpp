// SPDX-License-Identifier: Apache-2.0
// 원본 RRT* 탐색을 점유 격자와 차량 외곽 검사에 연결합니다.

#include "planning/behavior_pull_over_core.hpp"
#include "behavior_freespace_grid.hpp"
#include "autoware/freespace_planning_algorithms/rrtstar_core.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace planning {

std::optional<ShiftPullOverCandidate> generateRrtStarPullOverCandidate(
    const TrajectoryPoint &ego, BehaviorGoalPose goal,
    const CrosswalkOccupancyGrid &grid,
    const FreespacePullOverParameters &parameters) {
  const auto &p = parameters;
  const std::array<double, 17> values{
      ego.x, ego.y, goal.x, goal.y, goal.yaw_rad,
      grid.origin_x, grid.origin_y, grid.origin_yaw_rad,
      grid.resolution_m, p.wheelbase_m,
      p.maximum_steering_angle_rad, p.rrt_step_m,
      p.velocity_mps, p.front_m, p.rear_m, p.half_width_m,
      p.vehicle_shape_margin_m};
  for (const double value : values)
    if (!std::isfinite(value))
      throw std::invalid_argument("RRT* 주차 입력이 유효하지 않습니다");
  if (grid.width == 0 || grid.height == 0 ||
      grid.width > 2000 || grid.height > 2000 ||
      grid.values.size() != grid.width * grid.height ||
      grid.resolution_m <= 0.0 || p.wheelbase_m <= 0.0 ||
      p.maximum_steering_angle_rad <= 0.0 ||
      p.maximum_steering_angle_rad >= std::acos(-1.0) / 2.0 ||
      p.rrt_step_m <= 0.0 || p.rrt_iterations <= 0 ||
      !std::isfinite(p.rrt_time_limit_ms) ||
      p.rrt_time_limit_ms < 0.0 ||
      !std::isfinite(p.final_straight_distance_m) ||
      p.final_straight_distance_m < 0.0 ||
      p.velocity_mps <= 0.0 || p.front_m < 0.0 ||
      p.rear_m < 0.0 || p.half_width_m <= 0.0 ||
      p.vehicle_shape_margin_m < 0.0 ||
      p.obstacle_threshold < 1 || p.obstacle_threshold > 100)
    throw std::invalid_argument("RRT* 주차 설정이 유효하지 않습니다");
  if (!p.use_reverse) return std::nullopt;
  const auto final_goal = goal;
  if (!p.use_backward_goal_approach) {
    goal.x -= p.final_straight_distance_m *
        std::cos(goal.yaw_rad);
    goal.y -= p.final_straight_distance_m *
        std::sin(goal.yaw_rad);
  }
  const double ego_yaw = std::atan2(
      2.0 * (ego.orientation_w * ego.orientation_z +
             ego.orientation_x * ego.orientation_y),
      1.0 - 2.0 * (ego.orientation_y * ego.orientation_y +
                   ego.orientation_z * ego.orientation_z));
  if (!detail::footprintFree(grid, p, ego.x, ego.y, ego_yaw) ||
      !detail::footprintFree(grid, p, goal.x, goal.y, goal.yaw_rad) ||
      !detail::footprintFree(grid, p, final_goal.x,
                             final_goal.y, final_goal.yaw_rad))
    return std::nullopt;
  namespace rrt = autoware::freespace_planning_algorithms::rrtstar_core;
  const double cosine = std::cos(grid.origin_yaw_rad);
  const double sine = std::sin(grid.origin_yaw_rad);
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -min_x;
  double max_y = -min_y;
  for (int ix : {0, static_cast<int>(grid.width)})
    for (int iy : {0, static_cast<int>(grid.height)}) {
      const double local_x = ix * grid.resolution_m;
      const double local_y = iy * grid.resolution_m;
      const double x = grid.origin_x +
          local_x * cosine - local_y * sine;
      const double y = grid.origin_y +
          local_x * sine + local_y * cosine;
      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);
    }
  const double radius = p.wheelbase_m /
      std::tan(p.maximum_steering_angle_rad);
  const auto free_pose = [&](rrt::Pose pose) {
    return detail::footprintFree(
        grid, p, pose.x, pose.y, pose.yaw);
  };
  rrt::CSpace space(
      {min_x, min_y, 0.0}, {max_x, max_y, 2.0 * std::acos(-1.0)},
      radius, free_pose, p.rrt_seed);
  rrt::RRTStar search(
      {ego.x, ego.y, ego_yaw},
      {goal.x, goal.y, goal.yaw_rad},
      p.rrt_step_m,
      std::min(0.25, grid.resolution_m * 0.5),
      true, space);
  const auto start_time = std::chrono::steady_clock::now();
  for (int iteration = 0; iteration < p.rrt_iterations; ++iteration) {
    if (p.rrt_time_limit_ms > 0.0 &&
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start_time).count() >=
            p.rrt_time_limit_ms)
      break;
    search.extend();
  }
  if (!search.isSolutionFound()) return std::nullopt;
  const auto waypoints = search.sampleSolutionWaypoints();
  if (waypoints.size() < 2) return std::nullopt;
  Trajectory path;
  path.reserve(waypoints.size());
  std::vector<int> directions;
  directions.reserve(waypoints.size());
  for (std::size_t index = 0; index < waypoints.size(); ++index) {
    const auto &pose = waypoints[index];
    if (!path.empty() &&
        std::hypot(path.back().x - pose.x,
                   path.back().y - pose.y) < 1.0e-5)
      continue;
    TrajectoryPoint point = ego;
    point.x = pose.x;
    point.y = pose.y;
    point.orientation_z = std::sin(pose.yaw / 2.0);
    point.orientation_w = std::cos(pose.yaw / 2.0);
    if (path.empty()) {
      point.longitudinal_velocity_mps = 0.0F;
      directions.push_back(0);
    } else {
      const double along =
          (pose.x - path.back().x) * std::cos(pose.yaw) +
          (pose.y - path.back().y) * std::sin(pose.yaw);
      const int direction = along >= 0.0 ? 1 : -1;
      point.longitudinal_velocity_mps = static_cast<float>(
          direction * p.velocity_mps);
      directions.push_back(direction);
    }
    path.push_back(point);
  }
  if (path.size() < 2) return std::nullopt;
  std::vector<std::size_t> phase_ends;
  for (std::size_t index = 2; index < path.size(); ++index)
    if (directions[index] != directions[index - 1]) {
      path[index - 1].longitudinal_velocity_mps = 0.0F;
      phase_ends.push_back(index - 1);
    }
  const auto search_end_index = path.size() - 1;
  path.back().longitudinal_velocity_mps = 0.0F;
  if (!p.use_backward_goal_approach &&
      p.final_straight_distance_m > 0.0) {
    const int samples = std::max(1, static_cast<int>(std::ceil(
        p.final_straight_distance_m / 0.5)));
    for (int index = 1; index <= samples; ++index) {
      const double ratio = static_cast<double>(index) / samples;
      if (!detail::footprintFree(
              grid, p,
              goal.x + ratio * (final_goal.x - goal.x),
              goal.y + ratio * (final_goal.y - goal.y),
              goal.yaw_rad)) return std::nullopt;
    }
    if (directions.back() < 0)
      phase_ends.push_back(search_end_index);
    else path.back().longitudinal_velocity_mps =
        static_cast<float>(p.velocity_mps);
    for (int index = 1; index <= samples; ++index) {
      const double ratio = static_cast<double>(index) / samples;
      auto point = path.back();
      point.x = goal.x + ratio * (final_goal.x - goal.x);
      point.y = goal.y + ratio * (final_goal.y - goal.y);
      point.orientation_z = std::sin(goal.yaw_rad / 2.0);
      point.orientation_w = std::cos(goal.yaw_rad / 2.0);
      point.longitudinal_velocity_mps = index == samples ?
          0.0F : static_cast<float>(p.velocity_mps);
      path.push_back(point);
    }
  }
  phase_ends.push_back(path.size() - 1);
  ShiftPullOverCandidate result;
  result.shift_end_index = search_end_index;
  result.lateral_shift_m = final_goal.y - ego.y;
  result.phase_end_indices = std::move(phase_ends);
  result.path = std::move(path);
  return result;
}

}  // planning 네임스페이스
