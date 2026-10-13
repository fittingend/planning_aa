// Copyright 2024 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 점유 격자의 가림 영역과 횡단보도 접근 감속을 ROS 없이 계산합니다.

#include "planning/crosswalk_occlusion.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;

Polygon toPolygon(const std::vector<CrosswalkPoint> &points) {
  Polygon polygon;
  for (const auto &point : points) {
    polygon.outer().emplace_back(point.x, point.y);
  }
  bg::correct(polygon);
  return polygon;
}

CrosswalkPoint gridCellCenter(
    const CrosswalkOccupancyGrid &grid, std::size_t row, std::size_t col) {
  const double c = std::cos(grid.origin_yaw_rad);
  const double s = std::sin(grid.origin_yaw_rad);
  const double local_x = (col + 0.5) * grid.resolution_m;
  const double local_y = (row + 0.5) * grid.resolution_m;
  return {grid.origin_x + c * local_x - s * local_y,
          grid.origin_y + s * local_x + c * local_y};
}

double distanceToPath(
    CrosswalkPoint point, const std::vector<CrosswalkPoint> &path) {
  double nearest = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto &start = path[i - 1];
    const auto &end = path[i];
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double squared = dx * dx + dy * dy;
    const double ratio = squared > 0.0
        ? std::clamp(((point.x - start.x) * dx +
                      (point.y - start.y) * dy) / squared, 0.0, 1.0)
        : 0.0;
    nearest = std::min(nearest, std::hypot(
        point.x - start.x - ratio * dx,
        point.y - start.y - ratio * dy));
  }
  return nearest;
}

CrosswalkPoint extend(CrosswalkPoint before, CrosswalkPoint end, double distance) {
  const double dx = end.x - before.x;
  const double dy = end.y - before.y;
  const double length = std::hypot(dx, dy);
  if (length <= 1.0e-9) return end;
  return {end.x + distance * dx / length,
          end.y + distance * dy / length};
}

std::vector<double> pathArcs(const Trajectory &path) {
  std::vector<double> arcs(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    arcs[i] = arcs[i - 1] + std::hypot(
        path[i].x - path[i - 1].x,
        path[i].y - path[i - 1].y);
  }
  return arcs;
}

Trajectory capVelocityAtArc(
    const Trajectory &path, double start_arc, double velocity) {
  Trajectory output = path;
  if (path.size() < 2) return output;
  const auto arcs = pathArcs(path);
  if (start_arc < 0.0 || start_arc > arcs.back()) return output;
  const auto upper = std::upper_bound(arcs.begin(), arcs.end(), start_arc);
  const std::size_t next = std::distance(arcs.begin(), upper);
  std::size_t first = next;
  if (next < path.size() && next > 0) {
    const double ratio = (start_arc - arcs[next - 1]) /
                         (arcs[next] - arcs[next - 1]);
    const double previous_distance = start_arc - arcs[next - 1];
    const double next_distance = arcs[next] - start_arc;
    if (previous_distance <= 0.05) {
      first = next - 1;
    } else if (next_distance > 0.05) {
      auto point = path[next - 1];
      point.x += ratio * (path[next].x - point.x);
      point.y += ratio * (path[next].y - point.y);
      point.z += ratio * (path[next].z - point.z);
      point.longitudinal_velocity_mps = static_cast<float>(velocity);
      output.insert(output.begin() + next, point);
    }
  }
  for (std::size_t i = first; i < output.size(); ++i) {
    output[i].longitudinal_velocity_mps = std::min(
        output[i].longitudinal_velocity_mps,
        static_cast<float>(velocity));
  }
  return output;
}

}  // 익명 네임스페이스

std::vector<std::vector<CrosswalkPoint>> crosswalkDetectionAreas(
    const std::vector<CrosswalkPoint> &left_bound,
    const std::vector<CrosswalkPoint> &right_bound,
    CrosswalkPoint crossing, double detection_range_m) {
  if (left_bound.size() < 2 || right_bound.size() < 2 ||
      !std::isfinite(detection_range_m) || detection_range_m < 0.0) {
    throw std::invalid_argument("횡단보도 가림 감지 경계가 잘못되었습니다");
  }
  std::vector<std::vector<CrosswalkPoint>> areas;
  auto polygon = left_bound;
  polygon.insert(polygon.end(), right_bound.rbegin(), right_bound.rend());
  areas.push_back(std::move(polygon));
  const auto add_end = [&](CrosswalkPoint left_before, CrosswalkPoint left_end,
                           CrosswalkPoint right_before, CrosswalkPoint right_end) {
    const CrosswalkPoint center{
        (left_end.x + right_end.x) / 2.0,
        (left_end.y + right_end.y) / 2.0};
    const double distance = std::hypot(
        crossing.x - center.x, crossing.y - center.y);
    if (distance >= detection_range_m) return;
    const double extension = detection_range_m - distance;
    areas.push_back({left_end, extend(left_before, left_end, extension),
                     extend(right_before, right_end, extension), right_end});
  };
  add_end(left_bound[1], left_bound[0],
          right_bound[1], right_bound[0]);
  add_end(left_bound[left_bound.size() - 2], left_bound.back(),
          right_bound[right_bound.size() - 2], right_bound.back());
  return areas;
}

double crosswalkDetectionRange(
    double assumed_object_speed_mps,
    double distance_to_crosswalk_m, double ego_velocity_mps) {
  const double time = distance_to_crosswalk_m /
                      std::max(1.0, ego_velocity_mps);
  return time > 0.0 ? time * assumed_object_speed_mps : 20.0;
}

bool isCrosswalkOccluded(
    const CrosswalkOccupancyGrid &grid,
    const std::vector<std::vector<CrosswalkPoint>> &detection_areas,
    CrosswalkOcclusionParameters parameters,
    const std::vector<CrosswalkOcclusionMaskObject> &objects,
    const std::vector<CrosswalkPoint> &path) {
  if (grid.resolution_m <= 0.0 || grid.width == 0 || grid.height == 0 ||
      grid.values.size() != grid.width * grid.height ||
      parameters.minimum_size_m <= 0.0) {
    throw std::invalid_argument("횡단보도 점유 격자가 잘못되었습니다");
  }
  std::vector<Polygon> polygons;
  for (const auto &area : detection_areas) {
    if (area.size() >= 3) polygons.push_back(toPolygon(area));
  }
  if (polygons.empty()) return false;
  auto values = grid.values;
  if (parameters.ignore_behind_dynamic_objects) {
    const double grid_cos = std::cos(grid.origin_yaw_rad);
    const double grid_sin = std::sin(grid.origin_yaw_rad);
    const double half_width_m = grid.width * grid.resolution_m / 2.0;
    const double half_height_m = grid.height * grid.resolution_m / 2.0;
    const CrosswalkPoint center{
        grid.origin_x + grid_cos * half_width_m - grid_sin * half_height_m,
        grid.origin_y + grid_sin * half_width_m + grid_cos * half_height_m};
    const double range = std::max(grid.width, grid.height) *
                         grid.resolution_m / 2.0;
    for (const auto &object : objects) {
      if (object.longitudinal_speed_mps < object.minimum_mask_speed_mps ||
          object.length_m <= 0.0 || object.width_m <= 0.0 ||
          std::hypot(center.x - object.pose.x,
                     center.y - object.pose.y) >= range) continue;
      const double half_length =
          (object.length_m + parameters.object_inflate_size_m) / 2.0;
      const double half_width =
          (object.width_m + parameters.object_inflate_size_m) / 2.0;
      const double c = std::cos(object.pose.yaw_rad);
      const double s = std::sin(object.pose.yaw_rad);
      std::vector<CrosswalkPoint> corners;
      for (double longitudinal : {-half_length, half_length}) {
        for (double lateral : {-half_width, half_width}) {
          corners.push_back({
              object.pose.x + longitudinal * c - lateral * s,
              object.pose.y + longitudinal * s + lateral * c});
        }
      }
      std::sort(corners.begin(), corners.end(),
          [&](CrosswalkPoint a, CrosswalkPoint b) {
            return std::atan2(a.y - center.y, a.x - center.x) <
                   std::atan2(b.y - center.y, b.x - center.x);
          });
      const auto shadow = toPolygon({
          corners.front(), extend(center, corners.front(), 10.0 * range),
          extend(center, corners.back(), 10.0 * range), corners.back()});
      for (std::size_t row = 0; row < grid.height; ++row) {
        for (std::size_t col = 0; col < grid.width; ++col) {
          const auto cell = gridCellCenter(grid, row, col);
          if (bg::covered_by(Point(cell.x, cell.y), shadow)) {
            values[row * grid.width + col] = 0;
          }
        }
      }
    }
  }
  const std::size_t side = static_cast<std::size_t>(
      std::ceil(parameters.minimum_size_m / grid.resolution_m));
  for (std::size_t row = 0; row < grid.height; ++row) {
    for (std::size_t col = 0; col < grid.width; ++col) {
      const auto world = gridCellCenter(grid, row, col);
      const Point center(world.x, world.y);
      if (parameters.maximum_distance_to_path_m > 0.0 &&
          !path.empty() && distanceToPath(world, path) >
                               parameters.maximum_distance_to_path_m) continue;
      if (std::none_of(polygons.begin(), polygons.end(),
                      [&](const Polygon &polygon) {
                        return bg::covered_by(center, polygon);
                      })) continue;
      bool unknown_square = true;
      for (std::size_t dy = 0; dy < side && unknown_square; ++dy) {
        for (std::size_t dx = 0; dx < side; ++dx) {
          if (row + dy >= grid.height || col + dx >= grid.width) {
            unknown_square = false;
            break;
          }
          const auto square_cell = gridCellCenter(grid, row + dy, col + dx);
          if (std::none_of(polygons.begin(), polygons.end(),
                           [&](const Polygon &polygon) {
                             return bg::covered_by(
                                 Point(square_cell.x, square_cell.y), polygon);
                           })) {
            unknown_square = false;
            break;
          }
          const int value = values[(row + dy) * grid.width + col + dx];
          if (value < parameters.free_space_max ||
              value > parameters.occupied_min) {
            unknown_square = false;
            break;
          }
        }
      }
      if (unknown_square) return true;
    }
  }
  return false;
}

double crosswalkFeasibleSlowVelocity(
    double jerk_mps3, double acceleration_limit_mps2,
    double ego_acceleration_mps2, double ego_velocity_mps,
    double distance_to_target_m) {
  if (jerk_mps3 > 0.0 || acceleration_limit_mps2 > 0.0) {
    throw std::invalid_argument("횡단보도 감속 제한값이 잘못되었습니다");
  }
  if (distance_to_target_m <= 0.0) return ego_velocity_mps;
  const auto distance = [&](double time) {
    return jerk_mps3 * time * time * time / 6.0 +
           ego_acceleration_mps2 * time * time / 2.0 +
           ego_velocity_mps * time;
  };
  const auto velocity = [&](double time, double jerk, double acceleration,
                            double initial_velocity) {
    return jerk * time * time / 2.0 +
           acceleration * time + initial_velocity;
  };
  const double jerk_time =
      (acceleration_limit_mps2 - ego_acceleration_mps2) / jerk_mps3;
  const double remaining = distance_to_target_m - distance(jerk_time);
  if (remaining < 0.0) {
    double lower = 0.0;
    double upper = jerk_time;
    for (int i = 0; i < 80; ++i) {
      const double middle = (lower + upper) / 2.0;
      if (distance(middle) > distance_to_target_m) upper = middle;
      else lower = middle;
    }
    return velocity((lower + upper) / 2.0, jerk_mps3,
                    ego_acceleration_mps2, ego_velocity_mps);
  }
  const double after_jerk_velocity = velocity(
      jerk_time, jerk_mps3, ego_acceleration_mps2,
      ego_velocity_mps);
  const double discriminant = 2.0 * acceleration_limit_mps2 * remaining +
                              after_jerk_velocity * after_jerk_velocity;
  if (discriminant <= 0.0) return 0.0;
  const double constant_time =
      (-after_jerk_velocity + std::sqrt(discriminant)) /
      acceleration_limit_mps2;
  return velocity(constant_time, 0.0, acceleration_limit_mps2,
                  after_jerk_velocity);
}

CrosswalkOcclusionPlanner::CrosswalkOcclusionPlanner(
    CrosswalkOcclusionParameters parameters)
    : parameters_(parameters) {}

bool CrosswalkOcclusionPlanner::update(
    double now_s, bool occluded, bool enabled,
    bool crosswalk_has_traffic_light, bool map_skips_occlusion,
    bool ego_on_crosswalk) {
  if (!std::isfinite(now_s)) {
    throw std::invalid_argument("횡단보도 가림 판정 시각이 잘못되었습니다");
  }
  if (!enabled || map_skips_occlusion || ego_on_crosswalk ||
      (parameters_.ignore_with_traffic_light &&
       crosswalk_has_traffic_light)) return false;
  const double buffer = parameters_.time_buffer_s;
  if (occluded) {
    if (!initial_occlusion_s_) initial_occlusion_s_ = now_s;
    if (now_s - *initial_occlusion_s_ >= buffer) {
      most_recent_occlusion_s_ = now_s;
    }
  } else if (!most_recent_occlusion_s_ ||
             now_s - *most_recent_occlusion_s_ > buffer) {
    initial_occlusion_s_.reset();
  }
  if (most_recent_occlusion_s_ &&
      now_s - *most_recent_occlusion_s_ <= buffer) return true;
  most_recent_occlusion_s_.reset();
  return false;
}

Trajectory CrosswalkOcclusionPlanner::applySlowdown(
    const Trajectory &path, double entry_arc_m, double exit_arc_m,
    double ego_arc_m, double ego_velocity_mps,
    double ego_acceleration_mps2) {
  if (path.size() < 2) return path;
  const double feasible = ego_velocity_mps < parameters_.no_relax_speed_mps
      ? 0.0 : crosswalkFeasibleSlowVelocity(
          parameters_.maximum_slow_jerk_mps3,
          parameters_.maximum_slow_acceleration_mps2,
          ego_acceleration_mps2, ego_velocity_mps,
          entry_arc_m - ego_arc_m);
  const double target = std::max(
      feasible > 0.5 ? feasible : 0.0,
      parameters_.minimum_slow_speed_mps);
  if (!passed_slow_point_) {
    const double slow_arc = entry_arc_m - parameters_.vehicle_front_m -
                            parameters_.safety_slow_distance_m;
    const auto output = capVelocityAtArc(path, slow_arc, target);
    if (slow_arc < ego_arc_m) passed_slow_point_ = true;
    return output;
  }
  if (exit_arc_m > ego_arc_m) {
    Trajectory output = path;
    for (auto &point : output) {
      point.longitudinal_velocity_mps = std::min(
          point.longitudinal_velocity_mps,
          static_cast<float>(target));
    }
    return output;
  }
  return path;
}

}  // planning 네임스페이스
