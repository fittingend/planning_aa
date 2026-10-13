// Copyright 2024 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 경로상 차량 외곽과 교차 차로의 첫 접촉점에서 정체 정지점을 구합니다.

#include "planning/intersection_stuck_core.hpp"

#include "planning/jerk_resample.hpp"
#include "planning/stop_line_core.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace planning {
namespace {

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using LineString = bg::model::linestring<Point>;

Polygon makePolygon(const CrosswalkBounds &bounds) {
  Polygon shape;
  for (const auto &point : bounds.left) {
    bg::append(shape.outer(), Point(point.x, point.y));
  }
  for (auto it = bounds.right.rbegin(); it != bounds.right.rend(); ++it) {
    bg::append(shape.outer(), Point(it->x, it->y));
  }
  bg::correct(shape);
  return shape;
}

Polygon footprint(double x, double y, double yaw,
                  const IntersectionStuckParameters &settings) {
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  Polygon shape;
  for (const auto &pair : {
           std::pair<double, double>{settings.front_m, settings.width_m / 2.0},
           {settings.front_m, -settings.width_m / 2.0},
           {-settings.rear_m, -settings.width_m / 2.0},
           {-settings.rear_m, settings.width_m / 2.0}}) {
    bg::append(shape.outer(), Point(
        x + cosine * pair.first - sine * pair.second,
        y + sine * pair.first + cosine * pair.second));
  }
  bg::correct(shape);
  return shape;
}

std::vector<double> arcs(const Trajectory &path) {
  std::vector<double> result(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double step = std::hypot(path[i].x - path[i - 1].x,
                                   path[i].y - path[i - 1].y);
    if (step <= 1.0e-9) throw std::invalid_argument("경로에 중복 좌표가 있습니다");
    result[i] = result[i - 1] + step;
  }
  return result;
}

struct Projection {
  double arc = 0.0;
  double lateral = std::numeric_limits<double>::infinity();
};

Projection project(const Trajectory &path, const std::vector<double> &arc,
                   CrosswalkPoint point) {
  Projection result;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x - path[i].x;
    const double dy = path[i + 1].y - path[i].y;
    const double length2 = dx * dx + dy * dy;
    const double ratio = std::clamp(
        ((point.x - path[i].x) * dx + (point.y - path[i].y) * dy) / length2,
        0.0, 1.0);
    const double lateral = std::hypot(
        point.x - path[i].x - ratio * dx,
        point.y - path[i].y - ratio * dy);
    if (lateral < result.lateral) {
      result = {arc[i] + ratio * std::sqrt(length2), lateral};
    }
  }
  return result;
}

double judgeDistance(double velocity, double acceleration) {
  if (velocity <= 0.0) return 0.0;
  constexpr double max_acceleration = -2.8;
  constexpr double max_jerk = -5.0;
  const double delay_distance = velocity * 0.5;
  const double velocity_after_jerk = velocity +
      (max_acceleration * max_acceleration - acceleration * acceleration) /
          (2.0 * max_jerk);
  if (velocity_after_jerk <= 0.0) {
    const double time = -(max_acceleration + std::sqrt(
        acceleration * acceleration - 2.0 * max_jerk * velocity)) /
        max_jerk;
    return std::max(0.0, delay_distance + velocity * time +
        acceleration * time * time / 2.0 +
        max_jerk * time * time * time / 6.0);
  }
  const double time = (max_acceleration - acceleration) / max_jerk;
  const double jerk_distance = velocity * time +
      acceleration * time * time / 2.0 +
      max_jerk * time * time * time / 6.0;
  const double final_distance = -velocity_after_jerk * velocity_after_jerk /
      (2.0 * max_acceleration);
  return std::max(0.0, delay_distance + jerk_distance + final_distance);
}

Trajectory insertControlPoints(
    const Trajectory &path, const Trajectory &sampled,
    const std::vector<double> &sampled_arcs, std::size_t lane_start_index,
    std::size_t conflict_index, const CrosswalkBounds &first_conflict,
    const std::vector<CrosswalkPoint> &first_centerline,
    double ego_x, double ego_y, const IntersectionStuckParameters &settings) {
  if (!settings.insert_control_points || sampled.empty()) return path;
  const double ds = settings.sample_interval_m;
  std::size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < sampled.size(); ++i) {
    const double distance = std::hypot(
        sampled[i].x - ego_x, sampled[i].y - ego_y);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = i;
    }
  }
  LineString centerline;
  if (!first_centerline.empty()) {
    for (const auto &point : first_centerline) {
      bg::append(centerline, Point(point.x, point.y));
    }
  } else {
    const auto count = std::min(first_conflict.left.size(),
                                first_conflict.right.size());
    for (std::size_t i = 0; i < count; ++i) {
      bg::append(centerline, Point(
          0.5 * (first_conflict.left[i].x + first_conflict.right[i].x),
          0.5 * (first_conflict.left[i].y + first_conflict.right[i].y)));
    }
  }
  std::size_t centerline_index = conflict_index;
  if (centerline.size() >= 2) {
    for (std::size_t i = lane_start_index; i < sampled.size(); ++i) {
      const double yaw = 2.0 * std::atan2(
          sampled[i].orientation_z, sampled[i].orientation_w);
      if (bg::intersects(footprint(
              sampled[i].x, sampled[i].y, yaw, settings), centerline)) {
        centerline_index = i;
        break;
      }
    }
  }
  const auto judge_steps = static_cast<std::size_t>(std::ceil(
      judgeDistance(settings.ego_velocity_mps,
                    settings.ego_acceleration_mps2) / ds));
  const auto margin_steps = static_cast<std::size_t>(std::ceil(
      settings.stop_margin_m / ds));
  const auto backward = [](std::size_t index, std::size_t steps) {
    return index > steps ? index - steps : 0;
  };
  std::set<std::size_t> control_indices{
      closest_index,
      backward(conflict_index, judge_steps),
      backward(centerline_index, judge_steps),
      backward(conflict_index, margin_steps),
      conflict_index};
  if (conflict_index >= 5) control_indices.insert(conflict_index - 5);
  Trajectory enriched = path;
  for (const auto index : control_indices) {
    if (index >= sampled.size()) continue;
    const auto &sample = sampled[index];
    const bool duplicate = std::any_of(
        enriched.begin(), enriched.end(), [&](const auto &point) {
          return std::hypot(point.x - sample.x,
                            point.y - sample.y) < 0.001;
        });
    if (duplicate) continue;
    const auto enriched_arcs = arcs(enriched);
    const auto upper = std::upper_bound(
        enriched_arcs.begin(), enriched_arcs.end(), sampled_arcs[index]);
    const auto insertion = static_cast<std::size_t>(
        std::distance(enriched_arcs.begin(), upper));
    const auto prior = insertion == 0 ? 0 : insertion - 1;
    auto point = enriched[prior];
    point.x = sample.x;
    point.y = sample.y;
    point.z = sample.z;
    point.orientation_x = sample.orientation_x;
    point.orientation_y = sample.orientation_y;
    point.orientation_z = sample.orientation_z;
    point.orientation_w = sample.orientation_w;
    enriched.insert(enriched.begin() + std::min(insertion, enriched.size()), point);
  }
  return enriched;
}

}  // 익명 네임스페이스

static IntersectionStuckResult calculateIntersectionStop(
    const Trajectory &path,
    const std::vector<CrosswalkBounds> &conflicting_lanes,
    const std::vector<IntersectionStuckObject> &objects,
    double ego_x, double ego_y,
    IntersectionStuckParameters settings, bool stop_required) {
  const double values[] = {
      settings.front_m, settings.rear_m, settings.width_m,
      settings.stop_margin_m, settings.sample_interval_m,
      settings.velocity_threshold_mps, settings.path_lateral_tolerance_m,
      settings.minimum_conflict_arc_m,
      settings.ego_velocity_mps, settings.ego_acceleration_mps2,
      ego_x, ego_y};
  for (double value : values) {
    if (!std::isfinite(value)) throw std::invalid_argument("교차로 입력에는 유한한 값이 필요합니다");
  }
  if (settings.front_m < 0.0 || settings.rear_m < 0.0 ||
      settings.width_m <= 0.0 || settings.stop_margin_m < 0.0 ||
      settings.sample_interval_m <= 0.0 ||
      settings.velocity_threshold_mps < 0.0 ||
      settings.path_lateral_tolerance_m < 0.0 ||
      settings.minimum_conflict_arc_m < 0.0) {
    throw std::invalid_argument("교차로 정체 차량 설정이 잘못되었습니다");
  }
  if (settings.lane_entry_arc_m &&
      (!std::isfinite(*settings.lane_entry_arc_m) ||
       *settings.lane_entry_arc_m < 0.0)) {
    throw std::invalid_argument("교차로 차로 시작 호길이가 잘못되었습니다");
  }
  IntersectionStuckResult result;
  result.trajectory = path;
  if (path.size() < 2 || conflicting_lanes.empty()) return result;
  const auto arc = arcs(path);
  std::vector<Polygon> conflict_polygons;
  std::vector<std::size_t> conflict_indices;
  for (std::size_t index = 0; index < conflicting_lanes.size(); ++index) {
    const auto &bounds = conflicting_lanes[index];
    if (bounds.left.size() >= 2 && bounds.right.size() >= 2) {
      conflict_polygons.push_back(makePolygon(bounds));
      conflict_indices.push_back(index);
    }
  }
  std::vector<double> sampled_arcs;
  for (double distance = 0.0; distance < arc.back();
       distance += settings.sample_interval_m) {
    sampled_arcs.push_back(distance);
  }
  if (sampled_arcs.empty()) sampled_arcs.push_back(0.0);
  if (arc.back() - sampled_arcs.back() < 0.1) {
    sampled_arcs.back() = arc.back();
  } else {
    sampled_arcs.push_back(arc.back());
  }
  // 원본 resamplePath는 복수 차로 ID를 가진 입력점으로 가까운 표본을 옮깁니다.
  for (const double anchor : settings.multi_lane_anchor_arcs_m) {
    if (!std::isfinite(anchor) || anchor < 0.0 || anchor > arc.back()) {
      throw std::invalid_argument("교차로 차로 경계 호길이가 잘못되었습니다");
    }
    for (std::size_t index = 1; index < sampled_arcs.size(); ++index) {
      if (sampled_arcs[index - 1] <= anchor &&
          anchor < sampled_arcs[index]) {
        if (anchor - sampled_arcs[index - 1] < 0.1) {
          sampled_arcs[index - 1] = anchor;
        } else if (sampled_arcs[index] - anchor < 0.1) {
          sampled_arcs[index] = anchor;
        } else {
          sampled_arcs.insert(sampled_arcs.begin() + index, anchor);
        }
        break;
      }
    }
  }
  auto sampled = resampleJerkAtArcs(path, sampled_arcs);
  // 원본 resamplePath는 재표본화한 점의 진행 방향으로 자세를 다시 계산합니다.
  // 이 방향은 차량 외곽과 주목 차로 중심선의 첫 교차 인덱스를 결정합니다.
  for (std::size_t index = 0; index + 1 < sampled.size(); ++index) {
    const auto &next = sampled[index + 1];
    auto &point = sampled[index];
    const double dx = next.x - point.x;
    const double dy = next.y - point.y;
    const double yaw = std::atan2(dy, dx);
    const double pitch = std::atan2(next.z - point.z, std::hypot(dx, dy));
    point.orientation_x = -std::sin(pitch / 2.0) * std::sin(yaw / 2.0);
    point.orientation_y = std::sin(pitch / 2.0) * std::cos(yaw / 2.0);
    point.orientation_z = std::cos(pitch / 2.0) * std::sin(yaw / 2.0);
    point.orientation_w = std::cos(pitch / 2.0) * std::cos(yaw / 2.0);
  }
  if (sampled.size() >= 2) {
    sampled.back().orientation_x = sampled[sampled.size() - 2].orientation_x;
    sampled.back().orientation_y = sampled[sampled.size() - 2].orientation_y;
    sampled.back().orientation_z = sampled[sampled.size() - 2].orientation_z;
    sampled.back().orientation_w = sampled[sampled.size() - 2].orientation_w;
  }
  std::size_t conflict_polygon_index = 0;
  std::size_t conflict_sample_index = 0;
  std::size_t lane_start_index = 0;
  std::size_t search_start_index = 0;
  if (settings.lane_entry_arc_m) {
    const auto entry = std::lower_bound(
        sampled_arcs.begin(), sampled_arcs.end(),
        *settings.lane_entry_arc_m - 1.0e-6);
    const auto entry_index = static_cast<std::size_t>(
        std::distance(sampled_arcs.begin(), entry));
    lane_start_index = entry_index > 0 ? entry_index - 1 : 0;
    const auto vehicle_steps = static_cast<std::size_t>(
        settings.front_m / settings.sample_interval_m);
    search_start_index = lane_start_index > vehicle_steps
        ? lane_start_index - vehicle_steps : 0;
  }
  for (std::size_t index = search_start_index; index < sampled.size(); ++index) {
    const double s = sampled_arcs[index];
    if (s < settings.minimum_conflict_arc_m) continue;
    const double x = sampled[index].x;
    const double y = sampled[index].y;
    const double yaw = 2.0 * std::atan2(
        sampled[index].orientation_z, sampled[index].orientation_w);
    const auto vehicle = footprint(x, y, yaw, settings);
    for (std::size_t polygon_index = 0;
         polygon_index < conflict_polygons.size(); ++polygon_index) {
      if (bg::intersects(vehicle, conflict_polygons[polygon_index])) {
        result.first_conflict_arc_m = s;
        conflict_polygon_index = polygon_index;
        conflict_sample_index = index;
        break;
      }
    }
    if (result.first_conflict_arc_m) break;
  }
  if (!result.first_conflict_arc_m) return result;
  const auto bounds_index = conflict_indices[conflict_polygon_index];
  result.trajectory = insertControlPoints(
      path, sampled, sampled_arcs, lane_start_index, conflict_sample_index,
      conflicting_lanes[bounds_index],
      bounds_index < settings.conflict_centerlines.size()
          ? settings.conflict_centerlines[bounds_index]
          : std::vector<CrosswalkPoint>{},
      ego_x, ego_y, settings);
  const double ego_arc = project(path, arc, {ego_x, ego_y}).arc;
  std::vector<Polygon> detection_polygons;
  for (const auto &bounds : settings.detection_lanes) {
    if (bounds.left.size() >= 2 && bounds.right.size() >= 2) {
      detection_polygons.push_back(makePolygon(bounds));
    }
  }
  for (const auto &object : objects) {
    if (!object.target_vehicle || object.classification < 1 ||
        object.classification > 4 ||
        !std::isfinite(object.position.x) ||
        !std::isfinite(object.position.y) ||
        !std::isfinite(object.speed_mps) ||
        std::abs(object.speed_mps) > settings.velocity_threshold_mps) continue;
    const auto location = project(path, arc, object.position);
    const bool in_detection_area = detection_polygons.empty()
        ? location.lateral <= settings.path_lateral_tolerance_m
        : std::any_of(
              detection_polygons.begin(), detection_polygons.end(),
              [&object](const Polygon &area) {
                return bg::within(
                    Point(object.position.x, object.position.y), area);
              });
    if (in_detection_area &&
        location.arc >= *result.first_conflict_arc_m &&
        location.arc >= ego_arc &&
        (!settings.maximum_detection_arc_m ||
         location.arc <= *settings.maximum_detection_arc_m)) {
      result.stuck_detected = true;
      break;
    }
  }
  if (!result.stuck_detected && !stop_required) return result;
  const double stop_arc = *result.first_conflict_arc_m - settings.stop_margin_m;
  if (stop_arc < ego_arc - 0.5 || stop_arc < 0.0 ||
      stop_arc + settings.front_m > arc.back()) return result;
  const double crossing_arc = stop_arc + settings.front_m;
  const auto upper = std::upper_bound(arc.begin(), arc.end(), crossing_arc);
  const std::size_t next = std::min<std::size_t>(
      std::distance(arc.begin(), upper), path.size() - 1);
  const std::size_t prev = next - 1;
  const double ratio = (crossing_arc - arc[prev]) / (arc[next] - arc[prev]);
  const double x = path[prev].x + ratio * (path[next].x - path[prev].x);
  const double y = path[prev].y + ratio * (path[next].y - path[prev].y);
  const double yaw = std::atan2(path[next].y - path[prev].y,
                                path[next].x - path[prev].x);
  const double normal_x = -std::sin(yaw);
  const double normal_y = std::cos(yaw);
  const StopLineSegment line{
      x - 10.0 * normal_x, y - 10.0 * normal_y,
      x + 10.0 * normal_x, y + 10.0 * normal_y};
  StopLineParameters stop_settings;
  stop_settings.front_offset_m = settings.front_m;
  stop_settings.line_extension_m = 0.0;
  StopLineCore stop_core(line, stop_settings);
  const auto stopped = stop_core.apply(
      result.trajectory, ego_x, ego_y, false, 0.0);
  result.trajectory = stopped.trajectory;
  result.stop_arc_m = stopped.stop_arc_m;
  return result;
}

IntersectionStuckResult planIntersectionStuckStop(
    const Trajectory &path,
    const std::vector<CrosswalkBounds> &conflicting_lanes,
    const std::vector<IntersectionStuckObject> &objects,
    double ego_x, double ego_y,
    IntersectionStuckParameters parameters) {
  return calculateIntersectionStop(
      path, conflicting_lanes, objects, ego_x, ego_y, parameters, false);
}

IntersectionStuckResult planIntersectionConflictStop(
    const Trajectory &path,
    const std::vector<CrosswalkBounds> &conflicting_lanes,
    double ego_x, double ego_y,
    IntersectionStuckParameters parameters) {
  return calculateIntersectionStop(
      path, conflicting_lanes, {}, ego_x, ego_y, parameters, true);
}

}  // planning 네임스페이스
