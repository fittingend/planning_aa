// SPDX-License-Identifier: Apache-2.0
// 점유 격자와 차량 원호 운동으로 자유공간 주차 경로를 검색합니다.

#include "planning/behavior_pull_over_core.hpp"
#include "behavior_freespace_grid.hpp"
#include "autoware/freespace_planning_algorithms/reeds_shepp.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace planning {
namespace {

struct SearchNode {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double cost = 0.0;
  int parent = -1;
  int direction = 0;
  int steering = 0;
  double direction_distance = 0.0;
};

struct QueueEntry {
  double score = 0.0;
  int index = 0;
  bool operator<(const QueueEntry &other) const {
    return score > other.score;
  }
};

double yawOf(const TrajectoryPoint &point) {
  return std::atan2(
      2.0 * (point.orientation_w * point.orientation_z +
             point.orientation_x * point.orientation_y),
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z));
}

using detail::cellOf;
using detail::footprintFree;

struct ObstacleEdt {
  double distance = std::numeric_limits<double>::infinity();
  double angle = 0.0;
};

// Match the source planner's row scan and column minimum for obstacle distance.
std::vector<ObstacleEdt> sourceObstacleEdt(
    const CrosswalkOccupancyGrid &grid, int obstacle_threshold) {
  const auto width = static_cast<int>(grid.width);
  const auto height = static_cast<int>(grid.height);
  const double resolution = grid.resolution_m;
  std::vector<double> rows(
      grid.width * grid.height,
      std::numeric_limits<double>::infinity());
  std::vector<double> row_offsets(rows.size(), 0.0);
  for (int y = 0; y < height; ++y) {
    double distance = resolution;
    bool found_obstacle = false;
    for (int x = 0; x < width; ++x) {
      auto &value = rows[static_cast<std::size_t>(y) * grid.width + x];
      const int occupied = grid.values[static_cast<std::size_t>(y) * grid.width + x];
      if (occupied < 0 || occupied >= obstacle_threshold) {
        value = 0.0;
        distance = resolution;
        found_obstacle = true;
      } else if (found_obstacle) {
        value = distance;
        row_offsets[static_cast<std::size_t>(y) * grid.width + x] = -distance;
        distance += resolution;
      }
    }
    distance = resolution;
    found_obstacle = false;
    for (int x = width - 1; x >= 0; --x) {
      auto &value = rows[static_cast<std::size_t>(y) * grid.width + x];
      const int occupied = grid.values[static_cast<std::size_t>(y) * grid.width + x];
      if (occupied < 0 || occupied >= obstacle_threshold) {
        distance = resolution;
        found_obstacle = true;
      } else if (found_obstacle && value > distance) {
        value = distance;
        row_offsets[static_cast<std::size_t>(y) * grid.width + x] = distance;
        distance += resolution;
      }
    }
  }
  std::vector<ObstacleEdt> output(rows.size());
  std::vector<int> sites(height);
  std::vector<double> boundaries(height);
  const double resolution_squared = resolution * resolution;
  for (int x = 0; x < width; ++x) {
    int count = 0;
    for (int k = 0; k < height; ++k) {
      const double row = rows[static_cast<std::size_t>(k) * grid.width + x];
      if (!std::isfinite(row)) continue;
      double crossing = -std::numeric_limits<double>::infinity();
      while (count > 0) {
        const int previous = sites[count - 1];
        const double previous_row =
            rows[static_cast<std::size_t>(previous) * grid.width + x];
        crossing = (row * row - previous_row * previous_row +
                    resolution_squared * (k * k - previous * previous)) /
                   (2.0 * resolution_squared * (k - previous));
        if (count == 1 || crossing > boundaries[count - 1]) break;
        --count;
      }
      sites[count] = k;
      boundaries[count] = count == 0 ?
          -std::numeric_limits<double>::infinity() : crossing;
      ++count;
    }
    if (count == 0) continue;
    int best_site = 0;
    for (int y = 0; y < height; ++y) {
      const auto index = static_cast<std::size_t>(y) * grid.width + x;
      while (best_site + 1 < count && boundaries[best_site + 1] <= y)
        ++best_site;
      const int nearest = sites[best_site];
      const double row = rows[static_cast<std::size_t>(nearest) * grid.width + x];
      output[index] = {
          std::hypot(row, resolution * (y - nearest)),
          std::atan2(resolution * std::abs(y - nearest),
                     row_offsets[static_cast<std::size_t>(nearest) * grid.width + x])};
    }
  }
  return output;
}

std::vector<double> obstacleDistanceField(
    const CrosswalkOccupancyGrid &grid,
    const FreespacePullOverParameters &p,
    const std::vector<ObstacleEdt> &obstacle_edt,
    double goal_x, double goal_y) {
  std::vector<double> distances(
      grid.width * grid.height,
      std::numeric_limits<double>::infinity());
  const auto goal_cell = cellOf(grid, goal_x, goal_y);
  if (!goal_cell) return distances;
  const int goal_index = goal_cell->second *
      static_cast<int>(grid.width) + goal_cell->first;
  distances[goal_index] = 0.0;
  std::priority_queue<QueueEntry> queue;
  queue.push({0.0, goal_index});
  while (!queue.empty()) {
    const auto current = queue.top();
    queue.pop();
    if (current.score > distances[current.index] + 1.0e-9) continue;
    const int x = current.index % static_cast<int>(grid.width);
    const int y = current.index / static_cast<int>(grid.width);
    for (int dy = -1; dy <= 1; ++dy)
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;
        const int nx = x + dx;
        const int ny = y + dy;
        if (nx < 0 || ny < 0 ||
            nx >= static_cast<int>(grid.width) ||
            ny >= static_cast<int>(grid.height)) continue;
        const int next_index = ny * static_cast<int>(grid.width) + nx;
        const auto occupied = grid.values[next_index];
        if (occupied < 0 || occupied >= p.obstacle_threshold) continue;
        if (!obstacle_edt.empty() &&
            obstacle_edt[next_index].distance <
                p.half_width_m +
                    p.vehicle_shape_margin_m * 0.5)
          continue;
        const double distance = current.score +
            grid.resolution_m * std::hypot(dx, dy);
        if (distance >= distances[next_index]) continue;
        distances[next_index] = distance;
        queue.push({distance, next_index});
      }
  }
  return distances;
}

std::uint64_t stateKey(const CrosswalkOccupancyGrid &grid,
                       const FreespacePullOverParameters &p,
                       double x, double y, double yaw,
                       int direction) {
  const auto cell = cellOf(grid, x, y);
  if (!cell) return std::numeric_limits<std::uint64_t>::max();
  const double full_turn = 2.0 * std::acos(-1.0);
  const double angle = std::remainder(yaw, full_turn) +
      std::acos(-1.0);
  const double source_angle = std::fmod(
      yaw - grid.origin_yaw_rad + full_turn, full_turn);
  // Source yaw quantization preserves both synthetic paths; source XY rounding
  // changes the obstacle path's first maneuver until collision cells align.
  const int theta = p.use_source_obstacle_edt ?
      static_cast<int>(std::round(
          source_angle / full_turn * p.theta_bins)) % p.theta_bins :
      std::clamp(static_cast<int>(std::floor(
          angle / full_turn * p.theta_bins)),
          0, p.theta_bins - 1);
  if (p.use_source_obstacle_edt)
    return (static_cast<std::uint64_t>(cell->second) * grid.width +
            static_cast<std::uint64_t>(cell->first)) *
               static_cast<std::uint64_t>(p.theta_bins) + theta;
  return (((static_cast<std::uint64_t>(cell->second) * grid.width +
             static_cast<std::uint64_t>(cell->first)) *
            static_cast<std::uint64_t>(p.theta_bins) + theta) * 3U +
          static_cast<std::uint64_t>(direction + 1));
}

bool goalConnectionFree(const CrosswalkOccupancyGrid &grid,
                        const FreespacePullOverParameters &p,
                        const SearchNode &current,
                        const BehaviorGoalPose &goal) {
  const double distance = std::hypot(
      goal.x - current.x, goal.y - current.y);
  const double yaw_change = std::remainder(
      goal.yaw_rad - current.yaw, 2.0 * std::acos(-1.0));
  const int samples = std::max(1, static_cast<int>(std::ceil(
      distance / std::min(0.1, grid.resolution_m * 0.25))));
  for (int sample = 1; sample <= samples; ++sample) {
    const double ratio = static_cast<double>(sample) / samples;
    if (!footprintFree(grid, p,
            current.x + ratio * (goal.x - current.x),
            current.y + ratio * (goal.y - current.y),
            current.yaw + ratio * yaw_change))
      return false;
  }
  return true;
}

}  // 익명 네임스페이스

std::optional<ShiftPullOverCandidate> generateFreespacePullOverCandidate(
    const TrajectoryPoint &ego, BehaviorGoalPose goal,
    const CrosswalkOccupancyGrid &grid,
    const FreespacePullOverParameters &p) {
  if (p.use_rrt_star)
    return generateRrtStarPullOverCandidate(ego, goal, grid, p);
  const std::array<double, 17> values{
      ego.x, ego.y, goal.x, goal.y, goal.yaw_rad,
      grid.origin_x, grid.origin_y, grid.origin_yaw_rad,
      grid.resolution_m, p.wheelbase_m,
      p.maximum_steering_angle_rad, p.expansion_distance_m,
      p.velocity_mps, p.front_m, p.rear_m, p.half_width_m,
      p.vehicle_shape_margin_m};
  for (const double value : values)
    if (!std::isfinite(value))
      throw std::invalid_argument(
          "자유공간 주차 좌표와 차량 값이 유효하지 않습니다");
  if (!std::isfinite(p.goal_position_tolerance_m) ||
      !std::isfinite(p.goal_yaw_tolerance_rad) ||
      !std::isfinite(p.final_straight_distance_m) ||
      !std::isfinite(p.reverse_weight) ||
      !std::isfinite(p.curve_weight) ||
      !std::isfinite(p.smoothness_weight) ||
      !std::isfinite(p.near_goal_distance_m) ||
      !std::isfinite(p.goal_lateral_distance_weight) ||
      !std::isfinite(p.direction_change_weight) ||
      grid.width == 0 || grid.height == 0 ||
      grid.width > 2000 || grid.height > 2000 ||
      grid.values.size() != grid.width * grid.height ||
      grid.resolution_m <= 0.0 || p.wheelbase_m <= 0.0 ||
      p.maximum_steering_angle_rad <= 0.0 ||
      p.maximum_steering_angle_rad >= std::acos(-1.0) / 2.0 ||
      p.expansion_distance_m <= 0.0 || p.velocity_mps <= 0.0 ||
      p.front_m < 0.0 || p.rear_m < 0.0 ||
      p.half_width_m <= 0.0 ||
      p.vehicle_shape_margin_m < 0.0 ||
      p.goal_position_tolerance_m <= 0.0 ||
      p.goal_yaw_tolerance_rad <= 0.0 ||
      p.final_straight_distance_m < 0.0 ||
      p.reverse_weight < 0.0 ||
      p.curve_weight < 0.0 || p.smoothness_weight < 0.0 ||
      p.near_goal_distance_m < 0.0 ||
      p.goal_lateral_distance_weight < 0.0 ||
      p.direction_change_weight < 1.0 ||
      p.theta_bins < 8 || p.theta_bins > 360 ||
      p.obstacle_threshold < 1 || p.obstacle_threshold > 100 ||
      p.maximum_expansions <= 0)
    throw std::invalid_argument("자유공간 주차 설정이 유효하지 않습니다");
  const auto final_goal = goal;
  if (!p.use_backward_goal_approach) {
    goal.x -= p.final_straight_distance_m *
        std::cos(goal.yaw_rad);
    goal.y -= p.final_straight_distance_m *
        std::sin(goal.yaw_rad);
  }
  const double ego_yaw = yawOf(ego);
  if (!footprintFree(grid, p, ego.x, ego.y, ego_yaw) ||
      !footprintFree(grid, p, goal.x, goal.y, goal.yaw_rad) ||
      !footprintFree(grid, p, final_goal.x,
                     final_goal.y, final_goal.yaw_rad))
    return std::nullopt;
  std::vector<SearchNode> nodes{{ego.x, ego.y, ego_yaw,
                                  0.0, -1, 0, 0}};
  const autoware::freespace_planning_algorithms::ReedsSheppStateSpace
      reeds_shepp(p.wheelbase_m /
          std::tan(p.maximum_steering_angle_rad));
  const autoware::freespace_planning_algorithms::ReedsSheppStateSpace::StateXYT
      goal_state{goal.x, goal.y, goal.yaw_rad};
  const auto obstacle_edt = p.use_source_obstacle_edt ?
      sourceObstacleEdt(grid, p.obstacle_threshold) :
      std::vector<ObstacleEdt>{};
  const auto obstacle_distances = p.use_obstacle_distance_heuristic ?
      obstacleDistanceField(grid, p, obstacle_edt,
                            goal.x, goal.y) :
      std::vector<double>{};
  const auto edtAt = [&](double x, double y) {
    if (obstacle_edt.empty()) return ObstacleEdt{};
    const double dx = x - grid.origin_x;
    const double dy = y - grid.origin_y;
    const double cosine = std::cos(grid.origin_yaw_rad);
    const double sine = std::sin(grid.origin_yaw_rad);
    const auto ix = static_cast<int>(std::floor(
        (cosine * dx + sine * dy) / grid.resolution_m));
    const auto iy = static_cast<int>(std::floor(
        (-sine * dx + cosine * dy) / grid.resolution_m));
    if (ix < 0 || iy < 0 || ix >= static_cast<int>(grid.width) ||
        iy >= static_cast<int>(grid.height)) return ObstacleEdt{};
    return obstacle_edt[static_cast<std::size_t>(iy) * grid.width + ix];
  };
  const auto sourceIndexEdtAt = [&](double x, double y) {
    if (obstacle_edt.empty()) return ObstacleEdt{};
    const double dx = x - grid.origin_x;
    const double dy = y - grid.origin_y;
    const double cosine = std::cos(grid.origin_yaw_rad);
    const double sine = std::sin(grid.origin_yaw_rad);
    const int ix = static_cast<int>(std::round(
        (cosine * dx + sine * dy) / grid.resolution_m));
    const int iy = static_cast<int>(std::round(
        (-sine * dx + cosine * dy) / grid.resolution_m));
    if (ix < 0 || iy < 0 || ix >= static_cast<int>(grid.width) ||
        iy >= static_cast<int>(grid.height)) return ObstacleEdt{};
    return obstacle_edt[static_cast<std::size_t>(iy) * grid.width + ix];
  };
  const auto footprintFreeWithEdt = [&](double x, double y, double yaw) {
    if (!obstacle_edt.empty()) {
      const double dx = x - grid.origin_x;
      const double dy = y - grid.origin_y;
      const double cosine = std::cos(grid.origin_yaw_rad);
      const double sine = std::sin(grid.origin_yaw_rad);
      const double gx = cosine * dx + sine * dy;
      const double gy = -sine * dx + cosine * dy;
      const double shape_margin = p.vehicle_shape_margin_m * 0.5;
      const double max_dimension = std::hypot(
          std::max(p.front_m, p.rear_m) + shape_margin,
          p.half_width_m + shape_margin);
      const double clearance = max_dimension + 2.0 * grid.resolution_m;
      if (std::min({gx, gy, grid.width * grid.resolution_m - gx,
                    grid.height * grid.resolution_m - gy}) > clearance &&
          edtAt(x, y).distance > clearance)
        return true;
    }
    return footprintFree(grid, p, x, y, yaw);
  };
  const auto heuristic = [&](double x, double y, double yaw) {
    double distance = std::hypot(goal.x - x, goal.y - y);
    if (!obstacle_distances.empty()) {
      const auto cell = cellOf(grid, x, y);
      if (cell) {
        const double obstacle_distance = obstacle_distances[
            static_cast<std::size_t>(cell->second) * grid.width +
            static_cast<std::size_t>(cell->first)];
        if (std::isfinite(obstacle_distance))
          distance = std::max(distance, obstacle_distance);
      }
    }
    if (!p.use_reeds_shepp_heuristic) return 2.0 * distance;
    const double reeds_shepp_distance = reeds_shepp.distance(
        {x, y, yaw}, goal_state);
    return 2.0 * std::max(distance, reeds_shepp_distance);
  };
  std::priority_queue<QueueEntry> queue;
  queue.push({heuristic(ego.x, ego.y, ego_yaw), 0});
  std::unordered_map<std::uint64_t, double> best;
  best[stateKey(grid, p, ego.x, ego.y, ego_yaw, 0)] = 0.0;
  int goal_index = -1;
  int expanded = 0;
  while (!queue.empty() && expanded < p.maximum_expansions) {
    const auto entry = queue.top();
    queue.pop();
    const auto current = nodes[entry.index];
    const auto key = stateKey(grid, p, current.x, current.y,
                              current.yaw, current.direction);
    const auto known = best.find(key);
    if (known == best.end() || current.cost > known->second + 1.0e-9)
      continue;
    ++expanded;
    const double goal_distance = std::hypot(
        goal.x - current.x, goal.y - current.y);
    const double goal_angle = std::abs(std::remainder(
        goal.yaw_rad - current.yaw, 2.0 * std::acos(-1.0)));
    if (goal_distance <= p.goal_position_tolerance_m &&
        goal_angle <= p.goal_yaw_tolerance_rad &&
        goalConnectionFree(grid, p, current, goal)) {
      goal_index = entry.index;
      break;
    }
    const double obstacle_expansion = edtAt(current.x, current.y).distance * 0.3;
    const double expansion_distance = p.adapt_expansion_distance ?
        std::clamp(std::min(0.15 * goal_distance, obstacle_expansion),
                   std::max(p.expansion_distance_m, 1.5 * grid.resolution_m),
                   std::max(0.5 * p.wheelbase_m,
                            std::max(p.expansion_distance_m,
                                     1.5 * grid.resolution_m))) :
        p.expansion_distance_m;
    for (int direction : {1, -1}) {
      if (direction < 0 && !p.use_reverse) continue;
      for (int steering = -1; steering <= 1; ++steering) {
        if (current.parent >= 0 &&
            direction != current.direction &&
            steering == current.steering)
          continue;
        const double steering_angle =
            steering * p.maximum_steering_angle_rad;
        const double yaw_change = direction *
            expansion_distance / p.wheelbase_m *
            std::tan(steering_angle);
        SearchNode next;
        if (steering == 0) {
          next.x = current.x + direction * expansion_distance *
              std::cos(current.yaw);
          next.y = current.y + direction * expansion_distance *
              std::sin(current.yaw);
        } else {
          const double radius = p.wheelbase_m / std::tan(steering_angle);
          next.x = current.x + radius *
              (std::sin(current.yaw + yaw_change) -
               std::sin(current.yaw));
          next.y = current.y + radius *
              (std::cos(current.yaw) -
               std::cos(current.yaw + yaw_change));
        }
        next.yaw = std::remainder(current.yaw + yaw_change,
                                   2.0 * std::acos(-1.0));
        next.direction = direction;
        next.steering = steering;
        next.parent = entry.index;
        const int collision_samples = std::max(2,
            static_cast<int>(std::ceil(expansion_distance /
                std::min(0.25, grid.resolution_m * 0.5))));
        bool clear = true;
        for (int sample = 1; sample <= collision_samples; ++sample) {
          const double ratio = static_cast<double>(sample) /
              collision_samples;
          if (!footprintFreeWithEdt(
                  current.x + ratio * (next.x - current.x),
                  current.y + ratio * (next.y - current.y),
                  current.yaw + ratio * yaw_change)) {
            clear = false;
            break;
          }
        }
        if (!clear) continue;
        const auto next_key = stateKey(
            grid, p, next.x, next.y, next.yaw, direction);
        if (next_key == std::numeric_limits<std::uint64_t>::max())
          continue;
        const double direction_weight = direction < 0 ?
            1.0 + p.reverse_weight : 1.0;
        const bool direction_switch = current.direction != 0 &&
            current.direction != direction;
        next.cost = current.cost + expansion_distance *
            (1.0 + p.curve_weight * std::abs(steering)) *
            direction_weight +
            0.5 * p.smoothness_weight *
                std::abs(steering - current.steering);
        if (p.obstacle_distance_weight > 0.0 && !obstacle_edt.empty()) {
          const auto edt = sourceIndexEdtAt(next.x, next.y);
          const double margin = p.vehicle_shape_margin_m * 0.5;
          const double front = p.front_m + margin;
          const double rear = p.rear_m + margin;
          const double half_width = p.half_width_m + margin;
          const double max_dimension =
              std::hypot(std::max(front, rear), half_width);
          if (edt.distance <= max_dimension + 1.0) {
            const double full_turn = 2.0 * std::acos(-1.0);
            double angle = std::fmod(
                next.yaw - grid.origin_yaw_rad, full_turn);
            if (angle < 0.0) angle += full_turn;
            const int theta = static_cast<int>(std::round(
                angle / full_turn * p.theta_bins)) % p.theta_bins;
            const double yaw = theta * full_turn / p.theta_bins;
            const double relative = std::abs(std::remainder(
                yaw - edt.angle, full_turn));
            const double front_corner = std::atan(half_width / front);
            const double rear_corner =
                std::acos(-1.0) * 0.5 + std::atan(rear / half_width);
            double vehicle_radius = 0.0;
            if (relative < front_corner)
              vehicle_radius = front / std::cos(relative);
            else if (relative < std::acos(-1.0) * 0.5)
              vehicle_radius = half_width / std::sin(relative);
            else if (relative < rear_corner)
              vehicle_radius = half_width /
                  std::cos(relative - std::acos(-1.0) * 0.5);
            else
              vehicle_radius = rear /
                  std::cos(std::acos(-1.0) - relative);
            const double clearance =
                std::max(edt.distance - vehicle_radius, 0.0);
            next.cost += p.obstacle_distance_weight *
                std::max(1.0 - clearance, 0.0);
          }
        }
        if (direction_switch)
          next.cost += p.direction_change_weight *
              (1.0 + 1.0 / (1.0 +
                            current.direction_distance));
        next.direction_distance = expansion_distance +
            (direction_switch ? 0.0 :
                                current.direction_distance);
        if (p.near_goal_distance_m > 0.0 &&
            std::hypot(goal.x - next.x, goal.y - next.y) <=
                p.near_goal_distance_m) {
          const double goal_lateral =
              -(next.x - goal.x) * std::sin(goal.yaw_rad) +
              (next.y - goal.y) * std::cos(goal.yaw_rad);
          next.cost += p.goal_lateral_distance_weight *
              std::abs(goal_lateral);
        }
        const auto previous = best.find(next_key);
        if (previous != best.end() &&
            previous->second <= next.cost) continue;
        best[next_key] = next.cost;
        const int next_index = static_cast<int>(nodes.size());
        nodes.push_back(next);
        queue.push({next.cost +
            heuristic(next.x, next.y, next.yaw),
            next_index});
      }
    }
  }
  if (goal_index < 0) return std::nullopt;
  std::vector<int> indices;
  for (int index = goal_index; index >= 0;
       index = nodes[index].parent)
    indices.push_back(index);
  std::reverse(indices.begin(), indices.end());
  Trajectory path;
  path.reserve(indices.size() * 2 + 1);
  std::vector<std::size_t> node_path_indices;
  node_path_indices.reserve(indices.size());
  for (std::size_t k = 0; k < indices.size(); ++k) {
    const auto index = indices[k];
    const auto &state = nodes[index];
    if (k > 0 && p.adapt_expansion_distance) {
      const auto &parent = nodes[indices[k - 1]];
      const double distance = std::hypot(
          state.x - parent.x, state.y - parent.y);
      const double minimum_distance = std::max(
          p.expansion_distance_m, 1.5 * grid.resolution_m);
      const int segments = static_cast<int>(distance / minimum_distance);
        const double yaw_delta = std::remainder(
          state.yaw - parent.yaw, 2.0 * std::acos(-1.0));
      for (int step = 1; step < segments; ++step) {
        const double ratio = static_cast<double>(step) / segments;
        auto intermediate = ego;
        const double step_distance = ratio * distance;
        const double yaw = parent.yaw + ratio * yaw_delta;
        if (state.steering == 0) {
          intermediate.x = parent.x + state.direction * step_distance *
              std::cos(parent.yaw);
          intermediate.y = parent.y + state.direction * step_distance *
              std::sin(parent.yaw);
        } else {
          const double radius = p.wheelbase_m /
              std::tan(state.steering * p.maximum_steering_angle_rad);
          intermediate.x = parent.x + radius *
              (std::sin(yaw) - std::sin(parent.yaw));
          intermediate.y = parent.y + radius *
              (std::cos(parent.yaw) - std::cos(yaw));
        }
        intermediate.orientation_z = std::sin(yaw / 2.0);
        intermediate.orientation_w = std::cos(yaw / 2.0);
        intermediate.longitudinal_velocity_mps = static_cast<float>(
            state.direction * p.velocity_mps);
        path.push_back(intermediate);
      }
    }
    TrajectoryPoint point = ego;
    point.x = state.x;
    point.y = state.y;
    point.orientation_z = std::sin(state.yaw / 2.0);
    point.orientation_w = std::cos(state.yaw / 2.0);
    point.longitudinal_velocity_mps = static_cast<float>(
        state.direction * p.velocity_mps);
    path.push_back(point);
    node_path_indices.push_back(path.size() - 1);
  }
  std::vector<std::size_t> phase_ends;
  for (std::size_t index = 2; index < indices.size(); ++index) {
    if (nodes[indices[index]].direction !=
        nodes[indices[index - 1]].direction) {
      if (p.stop_at_phase_changes) {
        path[node_path_indices[index - 1]].longitudinal_velocity_mps = 0.0F;
        phase_ends.push_back(node_path_indices[index - 1]);
      }
    }
  }
  TrajectoryPoint final = path.back();
  final.x = goal.x;
  final.y = goal.y;
  final.orientation_z = std::sin(goal.yaw_rad / 2.0);
  final.orientation_w = std::cos(goal.yaw_rad / 2.0);
  final.longitudinal_velocity_mps = 0.0F;
  bool append_goal = p.append_exact_goal;
  if (p.source_goal_shift_policy) {
    const auto &terminal = nodes[goal_index];
    const double two_pi = 2.0 * std::acos(-1.0);
    const auto indexed_angle = [&](double radians) {
      const double normalized = std::fmod(std::fmod(radians, two_pi) +
                                         two_pi, two_pi);
      return static_cast<int>(std::lround(normalized / two_pi *
                                         p.theta_bins)) % p.theta_bins;
    };
    const bool same_goal_index =
        std::lround((terminal.x - grid.origin_x) / grid.resolution_m) ==
            std::lround((goal.x - grid.origin_x) / grid.resolution_m) &&
        std::lround((terminal.y - grid.origin_y) / grid.resolution_m) ==
            std::lround((goal.y - grid.origin_y) / grid.resolution_m) &&
        indexed_angle(terminal.yaw) == indexed_angle(goal.yaw_rad);
    const double lateral_offset =
        -(terminal.x - goal.x) * std::sin(goal.yaw_rad) +
        (terminal.y - goal.y) * std::cos(goal.yaw_rad);
    const double longitudinal_offset =
        (terminal.x - goal.x) * std::cos(goal.yaw_rad) +
        (terminal.y - goal.y) * std::sin(goal.yaw_rad);
    const bool behind_goal = longitudinal_offset <= 0.0;
    const bool backing = terminal.direction < 0;
    append_goal = !same_goal_index && (behind_goal != backing);
    if (append_goal) {
      final.x -= lateral_offset * std::sin(goal.yaw_rad);
      final.y += lateral_offset * std::cos(goal.yaw_rad);
      final.longitudinal_velocity_mps = static_cast<float>(
          terminal.direction * p.velocity_mps);
    }
  }
  if (append_goal) path.push_back(final);
  else path.back().longitudinal_velocity_mps = 0.0F;
  if (!p.use_backward_goal_approach &&
      p.final_straight_distance_m > 0.0) {
    const int samples = std::max(1, static_cast<int>(std::ceil(
        p.final_straight_distance_m / 0.5)));
    for (int index = 1; index <= samples; ++index) {
      const double ratio = static_cast<double>(index) / samples;
      const double x = goal.x + ratio *
          (final_goal.x - goal.x);
      const double y = goal.y + ratio *
          (final_goal.y - goal.y);
      if (!footprintFree(grid, p, x, y, goal.yaw_rad))
        return std::nullopt;
    }
    if (nodes[indices.back()].direction < 0)
      phase_ends.push_back(path.size() - 1);
    else path.back().longitudinal_velocity_mps =
        static_cast<float>(p.velocity_mps);
    for (int index = 1; index <= samples; ++index) {
      const double ratio = static_cast<double>(index) / samples;
      auto point = path.back();
      point.x = goal.x + ratio * (final_goal.x - goal.x);
      point.y = goal.y + ratio * (final_goal.y - goal.y);
      point.longitudinal_velocity_mps = index == samples ?
          0.0F : static_cast<float>(p.velocity_mps);
      path.push_back(point);
    }
  }
  phase_ends.push_back(path.size() - 1);
  ShiftPullOverCandidate result;
  result.path = std::move(path);
  result.shift_end_index = node_path_indices.back();
  result.lateral_shift_m = final_goal.y - ego.y;
  result.phase_end_indices = std::move(phase_ends);
  return result;
}

}  // planning 네임스페이스
