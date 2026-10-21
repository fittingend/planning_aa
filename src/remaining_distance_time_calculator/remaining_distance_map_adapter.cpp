// SPDX-License-Identifier: Apache-2.0

#include "planning/remaining_distance_map_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace planning {
namespace {
double length(const std::vector<CrosswalkPoint> &line) {
  double result = 0.0;
  for (std::size_t i = 1; i < line.size(); ++i)
    result += std::hypot(line[i].x - line[i - 1].x,
                         line[i].y - line[i - 1].y);
  return result;
}

std::optional<double> arc(const std::vector<CrosswalkPoint> &line,
                          RouteCheckpoint pose) {
  if (line.size() < 2) return std::nullopt;
  double best_distance = std::numeric_limits<double>::infinity();
  double best_arc = 0.0;
  double traveled = 0.0;
  for (std::size_t i = 1; i < line.size(); ++i) {
    const double dx = line[i].x - line[i - 1].x;
    const double dy = line[i].y - line[i - 1].y;
    const double segment_length = std::hypot(dx, dy);
    if (segment_length < 1.0e-9) continue;
    const double ratio = std::clamp(
        ((pose.x - line[i - 1].x) * dx +
         (pose.y - line[i - 1].y) * dy) /
            (segment_length * segment_length),
        0.0, 1.0);
    const double distance = std::hypot(
        pose.x - line[i - 1].x - ratio * dx,
        pose.y - line[i - 1].y - ratio * dy);
    if (distance < best_distance) {
      best_distance = distance;
      best_arc = traveled + ratio * segment_length;
    }
    traveled += segment_length;
  }
  return best_arc;
}
}  // namespace

RemainingDistanceMapAdapter::RemainingDistanceMapAdapter(
    const std::vector<std::uint8_t> &map_data)
    : owned_map_(std::make_unique<LaneletBinAdapter>(map_data)),
      map_(owned_map_.get()),
      core_([this](RouteCheckpoint ego, RouteCheckpoint goal) {
        return routeDistance(ego, goal);
      }) {}

RemainingDistanceMapAdapter::RemainingDistanceMapAdapter(
    const LaneletBinAdapter &map)
    : map_(&map),
      core_([this](RouteCheckpoint ego, RouteCheckpoint goal) {
        return routeDistance(ego, goal);
      }) {}

std::optional<double> RemainingDistanceMapAdapter::routeDistance(
    RouteCheckpoint ego, RouteCheckpoint goal) const {
  const auto route = active_route_ ? active_route_ : map_->routeBetweenCheckpoints(ego, goal);
  if (!route || route->sections.empty()) return std::nullopt;
  const auto first = map_->laneCenterline(
      route->sections.front().preferred_lanelet_id);
  const auto last = map_->laneCenterline(
      route->sections.back().preferred_lanelet_id);
  const auto ego_arc = arc(first, ego);
  const auto goal_arc = arc(last, goal);
  if (!ego_arc || !goal_arc) return std::nullopt;
  if (route->sections.size() == 1)
    return *goal_arc - *ego_arc;
  double distance = length(first) - *ego_arc + *goal_arc;
  for (std::size_t i = 1; i + 1 < route->sections.size(); ++i)
    distance += length(map_->laneCenterline(
        route->sections[i].preferred_lanelet_id));
  return distance;
}

}  // namespace planning
