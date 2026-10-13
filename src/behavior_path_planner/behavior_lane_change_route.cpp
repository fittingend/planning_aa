// SPDX-License-Identifier: Apache-2.0
// 지도 의존 부분에서 전달한 차로별 중심선을 후보 기준선으로 바꿉니다.

#include "planning/behavior_lane_change_route.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {

double distance(const TrajectoryPoint &a, const TrajectoryPoint &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

struct JoinedCenterline {
  Trajectory path;
  std::vector<std::vector<std::int64_t>> lane_ids;
  std::vector<double> arcs;
};

std::optional<JoinedCenterline> joinCenterlines(
    const std::vector<LaneChangeCenterline> &centerlines) {
  JoinedCenterline joined;
  for (const auto &lane : centerlines) {
    if (lane.points.size() < 2 || lane.lane_id == 0) return std::nullopt;
    for (const auto &point : lane.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) return std::nullopt;
      if (!joined.path.empty()) {
        const double gap = distance(joined.path.back(), point);
        if (gap < 1.0e-6) {
          auto &ids = joined.lane_ids.back();
          if (std::find(ids.begin(), ids.end(), lane.lane_id) == ids.end()) {
            ids.push_back(lane.lane_id);
          }
          continue;
        }
        if (&point == &lane.points.front() && gap > 2.0) {
          return std::nullopt;
        }
        joined.arcs.push_back(joined.arcs.back() + gap);
      } else {
        joined.arcs.push_back(0.0);
      }
      joined.path.push_back(point);
      joined.lane_ids.push_back({lane.lane_id});
    }
  }
  if (joined.path.size() < 2) return std::nullopt;
  return joined;
}

double projectArc(const JoinedCenterline &line,
                  BehaviorPathShiftAnchor point) {
  double nearest_squared = std::numeric_limits<double>::infinity();
  double nearest_arc = 0.0;
  for (std::size_t i = 0; i + 1 < line.path.size(); ++i) {
    const auto &a = line.path[i];
    const auto &b = line.path[i + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared < 1.0e-12) continue;
    const double ratio = std::clamp(
        ((point.x - a.x) * dx + (point.y - a.y) * dy) /
            length_squared,
        0.0, 1.0);
    const double error_x = point.x - a.x - ratio * dx;
    const double error_y = point.y - a.y - ratio * dy;
    const double squared = error_x * error_x + error_y * error_y;
    if (squared < nearest_squared) {
      nearest_squared = squared;
      nearest_arc = line.arcs[i] + ratio * std::sqrt(length_squared);
    }
  }
  return nearest_arc;
}

void appendSample(LaneChangeMappedReference &result,
                  const JoinedCenterline &line, double arc_m,
                  double velocity_mps) {
  const auto upper = std::upper_bound(
      line.arcs.begin(), line.arcs.end(), arc_m);
  const std::size_t index = std::min(
      static_cast<std::size_t>(upper - line.arcs.begin() - 1),
      line.path.size() - 2);
  const auto &a = line.path[index];
  const auto &b = line.path[index + 1];
  const double length = line.arcs[index + 1] - line.arcs[index];
  const double ratio = length > 0.0 ?
      (arc_m - line.arcs[index]) / length : 0.0;
  TrajectoryPoint point = a;
  point.x = a.x + ratio * (b.x - a.x);
  point.y = a.y + ratio * (b.y - a.y);
  point.z = a.z + ratio * (b.z - a.z);
  const double yaw = std::atan2(b.y - a.y, b.x - a.x);
  point.orientation_x = 0.0;
  point.orientation_y = 0.0;
  point.orientation_z = std::sin(yaw / 2.0);
  point.orientation_w = std::cos(yaw / 2.0);
  point.longitudinal_velocity_mps = static_cast<float>(velocity_mps);
  result.path.push_back(point);
  result.lane_ids.push_back(line.lane_ids[index]);
}

}  // 익명 네임스페이스

std::optional<LaneChangeMappedReference> buildLaneChangeMappedReference(
    const std::vector<LaneChangeCenterline> &centerlines,
    BehaviorPathShiftAnchor change_start, double change_length_m,
    double change_velocity_mps, double forward_path_length_m,
    double next_change_buffer_m,
    std::optional<BehaviorPathShiftAnchor> goal) {
  if (!std::isfinite(change_start.x) || !std::isfinite(change_start.y) ||
      (goal && (!std::isfinite(goal->x) || !std::isfinite(goal->y)))) {
    throw std::invalid_argument("차로 변경 기준 자세가 유한하지 않습니다");
  }
  const auto joined = joinCenterlines(centerlines);
  if (!joined) return std::nullopt;
  const double start_arc = projectArc(*joined, change_start);
  const std::optional<double> goal_arc = goal ?
      std::optional<double>{projectArc(*joined, *goal)} : std::nullopt;
  const auto range = laneChangeTargetReferenceArcRange(
      start_arc, change_length_m, forward_path_length_m,
      joined->arcs.back(), next_change_buffer_m, goal_arc);
  if (!range || range->end_m <= range->start_m) return std::nullopt;
  const double interval = laneChangeTargetReferenceInterval(
      change_length_m, change_velocity_mps);
  if (interval <= 0.0 ||
      (range->end_m - range->start_m) / interval > 100000.0) {
    return std::nullopt;
  }
  LaneChangeMappedReference result;
  result.start_arc_m = range->start_m;
  result.end_arc_m = range->end_m;
  for (double arc = range->start_m; arc < range->end_m; arc += interval) {
    appendSample(result, *joined, arc, change_velocity_mps);
  }
  appendSample(result, *joined, range->end_m, change_velocity_mps);
  return result;
}

std::optional<double> laneChangeNextRegulatoryDistance(
    const LaneChangeRegulatoryDistances &distances,
    bool regulate_intersection, bool regulate_crosswalk,
    bool regulate_traffic_light) {
  std::optional<double> nearest;
  const auto consider = [&](std::optional<double> value, bool enabled) {
    if (!enabled || !value) return;
    if (!std::isfinite(*value) || *value < 0.0) {
      throw std::invalid_argument("규제 요소 거리가 유효하지 않습니다");
    }
    nearest = nearest ? std::min(*nearest, *value) : *value;
  };
  consider(distances.intersection_m, regulate_intersection);
  consider(distances.crosswalk_m, regulate_crosswalk);
  consider(distances.traffic_light_m, regulate_traffic_light);
  return nearest;
}

}  // planning 네임스페이스
