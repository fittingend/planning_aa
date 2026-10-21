// SPDX-License-Identifier: Apache-2.0
#include "behavior_signal_output.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace adaptive {
namespace {
double length(const std::vector<planning::CrosswalkPoint> &line) {
  double distance = 0.0;
  for (std::size_t i = 1; i < line.size(); ++i)
    distance += std::hypot(line[i].x - line[i - 1].x,
                           line[i].y - line[i - 1].y);
  return distance;
}

double projectedArc(const std::vector<planning::CrosswalkPoint> &line,
                    planning::RouteCheckpoint ego) {
  double best_distance = std::numeric_limits<double>::infinity();
  double best_arc = 0.0;
  double traveled = 0.0;
  for (std::size_t i = 1; i < line.size(); ++i) {
    const double dx = line[i].x - line[i - 1].x;
    const double dy = line[i].y - line[i - 1].y;
    const double segment = std::hypot(dx, dy);
    if (segment < 1.0e-9) continue;
    const double t = std::clamp(((ego.x - line[i - 1].x) * dx +
                                 (ego.y - line[i - 1].y) * dy) /
                                    (segment * segment), 0.0, 1.0);
    const double error = std::hypot(ego.x - line[i - 1].x - t * dx,
                                    ego.y - line[i - 1].y - t * dy);
    if (error < best_distance) {
      best_distance = error;
      best_arc = traveled + t * segment;
    }
    traveled += segment;
  }
  return best_arc;
}
}  // namespace

BehaviorSignalDecision decideBehaviorSignals(
    const planning::LaneletBinAdapter &map, const planning::MissionRoute &route,
    planning::RouteCheckpoint ego, double speed_mps,
    const std::optional<planning::BehaviorRouteActionResult> &action) {
  BehaviorSignalDecision result;
  if (route.sections.empty()) return result;
  std::vector<std::int64_t> ids;
  ids.reserve(route.sections.size());
  for (const auto &section : route.sections)
    ids.push_back(section.preferred_lanelet_id);
  const auto current = map.closestRouteRoadLaneletId(
      ego.x, ego.y, ego.yaw_rad, ids, 10.0);
  if (!current) return result;
  const auto found = std::find(ids.begin(), ids.end(), *current);
  if (found == ids.end()) return result;

  // The original turn signal decider gives hazard priority during a reverse
  // pull out phase. The route action path contains the signed target velocity.
  if (action && action->kind == planning::BehaviorRouteActionKind::StartPullOut &&
      !action->path.points.empty() &&
      action->path.points.front().longitudinal_velocity_mps < -0.01) {
    result.turn = 1;
    result.hazard = 2;
    return result;
  }

  // A route lane change has an explicit target lane in the action path.
  if (action && action->kind == planning::BehaviorRouteActionKind::LaneChange &&
      !action->path.lane_ids.empty()) {
    for (const auto &point_ids : action->path.lane_ids) {
      for (const auto target : point_ids) {
        if (target == *current) continue;
        if (map.adjacentRoadLaneletId(*current, true) == target) {
          result.turn = 2;
          return result;
        }
        if (map.adjacentRoadLaneletId(*current, false) == target) {
          result.turn = 3;
          return result;
        }
      }
    }
  }

  const double search = 30.0 + 3.0 * std::max(0.0, speed_mps);
  double to_start = -projectedArc(map.laneCenterline(*current), ego);
  for (auto it = found; it != ids.end(); ++it) {
    const auto direction = map.laneTurnDirection(*it);
    const auto centerline = map.laneCenterline(*it);
    if (to_start <= search && to_start + length(centerline) > 0.0) {
      if (direction == planning::LaneTurnDirection::Left) result.turn = 2;
      if (direction == planning::LaneTurnDirection::Right) result.turn = 3;
      if (result.turn != 0) return result;
    }
    to_start += length(centerline);
    if (to_start > search) break;
  }
  return result;
}
}  // namespace adaptive
