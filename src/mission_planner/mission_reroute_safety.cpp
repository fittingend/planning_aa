// SPDX-License-Identifier: Apache-2.0
#include "planning/mission_reroute_safety.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace planning {
namespace {

bool samePrimitives(const MissionRouteSection &a,
                    const MissionRouteSection &b) {
  if (a.primitive_lanelet_ids.size() != b.primitive_lanelet_ids.size())
    return false;
  return std::all_of(a.primitive_lanelet_ids.begin(),
                     a.primitive_lanelet_ids.end(), [&](std::int64_t id) {
    return std::find(b.primitive_lanelet_ids.begin(),
                     b.primitive_lanelet_ids.end(), id) !=
           b.primitive_lanelet_ids.end();
  });
}

struct LaneProjection {
  double arc = 0.0;
  double length = 0.0;
  double distance = std::numeric_limits<double>::infinity();
};

LaneProjection project(const std::vector<CrosswalkPoint> &line,
                       RouteCheckpoint pose) {
  LaneProjection result;
  double traveled = 0.0;
  for (std::size_t i = 1; i < line.size(); ++i) {
    const double dx = line[i].x - line[i - 1].x;
    const double dy = line[i].y - line[i - 1].y;
    const double segment = std::hypot(dx, dy);
    if (segment < 1e-9) continue;
    const double ratio = std::clamp(
        ((pose.x - line[i - 1].x) * dx +
         (pose.y - line[i - 1].y) * dy) / (segment * segment),
        0.0, 1.0);
    const double error = std::hypot(
        pose.x - line[i - 1].x - ratio * dx,
        pose.y - line[i - 1].y - ratio * dy);
    if (error < result.distance) {
      result.distance = error;
      result.arc = traveled + ratio * segment;
    }
    traveled += segment;
  }
  result.length = traveled;
  return result;
}

std::optional<LaneProjection> closest(
    const MissionRouteSection &section, RouteCheckpoint pose,
    const MissionRerouteGeometry &geometry) {
  std::optional<LaneProjection> result;
  for (const auto id : section.primitive_lanelet_ids) {
    const auto line = geometry.centerline(id);
    const auto candidate = project(line, pose);
    if (!std::isfinite(candidate.distance)) continue;
    if (!result || candidate.distance < result->distance)
      result = candidate;
  }
  return result;
}

}  // namespace

bool checkMissionRerouteSafety(
    const RouteSelectorRoute &original,
    const RouteSelectorRoute &target,
    const MissionPlannerEgo &ego,
    const MissionRerouteGeometry &geometry,
    double reroute_time_threshold_s,
    double minimum_reroute_length_m) {
  const auto &old_sections = original.route.sections;
  const auto &new_sections = target.route.sections;
  if (old_sections.empty() || new_sections.empty() ||
      !geometry.centerline || !geometry.contains ||
      !std::isfinite(ego.speed_mps) ||
      !std::isfinite(reroute_time_threshold_s) ||
      !std::isfinite(minimum_reroute_length_m) ||
      reroute_time_threshold_s < 0.0 || minimum_reroute_length_m < 0.0)
    return false;
  if (ego.speed_mps < 0.01) return true;

  std::optional<std::pair<std::size_t, std::size_t>> start;
  for (std::size_t old_index = 0; old_index < old_sections.size() &&
       !start; ++old_index) {
    for (std::size_t new_index = 0; new_index < new_sections.size();
         ++new_index) {
      if (samePrimitives(old_sections[old_index],
                         new_sections[new_index])) {
        start = std::make_pair(old_index, new_index);
        break;
      }
    }
  }
  if (!start) return false;
  const auto [old_start, new_start] = *start;
  std::size_t old_end = old_start;
  std::size_t new_end = new_start;
  for (std::size_t offset = 1;
       old_start + offset < old_sections.size() &&
       new_start + offset < new_sections.size(); ++offset) {
    if (!samePrimitives(old_sections[old_start + offset],
                        new_sections[new_start + offset])) break;
    old_end = old_start + offset;
    new_end = new_start + offset;
  }

  const bool on_first_target = std::any_of(
      new_sections.front().primitive_lanelet_ids.begin(),
      new_sections.front().primitive_lanelet_ids.end(),
      [&](std::int64_t id) { return geometry.contains(id, target.start); });
  if (!on_first_target) return false;

  const auto first_old_index = new_start != 0 && old_start > 1 ?
      old_start - 1 : old_start;
  const auto first = closest(old_sections[first_old_index],
                             target.start, geometry);
  if (!first) return false;
  double common_length = first->length - first->arc;
  for (std::size_t i = old_start + 1; i <= old_end; ++i) {
    const auto &section = old_sections[i];
    if (section.primitive_lanelet_ids.empty()) break;
    double minimum = std::numeric_limits<double>::infinity();
    for (const auto id : section.primitive_lanelet_ids) {
      const auto length = project(geometry.centerline(id), target.start).length;
      if (length > 0.0) minimum = std::min(minimum, length);
    }
    if (!std::isfinite(minimum)) return false;
    common_length += minimum;
  }
  for (const auto id : new_sections[new_end].primitive_lanelet_ids) {
    if (!geometry.contains(id, target.goal)) continue;
    const auto end = project(geometry.centerline(id), target.goal);
    if (!std::isfinite(end.distance)) return false;
    common_length = std::max(common_length -
                             (end.length - end.arc), 0.0);
    break;
  }
  return common_length > std::max(
      ego.speed_mps * reroute_time_threshold_s,
      minimum_reroute_length_m);
}

}  // namespace planning
