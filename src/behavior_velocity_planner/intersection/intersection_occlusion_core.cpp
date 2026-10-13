// Copyright 2024 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 교차로 가림 격자의 크기·거리 조건을 독립 계산 코어로 묶습니다.

#include "planning/intersection_occlusion_core.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace planning {
namespace {

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using Line = bg::model::linestring<Point>;

CrosswalkPoint pointAtArc(
    const std::vector<CrosswalkPoint> &line, double arc);

CrosswalkPoint cellCenter(
    const CrosswalkOccupancyGrid &grid, std::size_t row, std::size_t col) {
  const double x = (col + 0.5) * grid.resolution_m;
  const double y = (row + 0.5) * grid.resolution_m;
  const double cosine = std::cos(grid.origin_yaw_rad);
  const double sine = std::sin(grid.origin_yaw_rad);
  return {grid.origin_x + cosine * x - sine * y,
          grid.origin_y + sine * x + cosine * y};
}

std::optional<std::size_t> cellIndex(
    const CrosswalkOccupancyGrid &grid, CrosswalkPoint point) {
  const double x = point.x - grid.origin_x;
  const double y = point.y - grid.origin_y;
  const double cosine = std::cos(grid.origin_yaw_rad);
  const double sine = std::sin(grid.origin_yaw_rad);
  const double local_x = cosine * x + sine * y;
  const double local_y = -sine * x + cosine * y;
  if (local_x < 0.0 || local_y < 0.0) return std::nullopt;
  const auto col = static_cast<std::size_t>(local_x / grid.resolution_m);
  const auto row = static_cast<std::size_t>(local_y / grid.resolution_m);
  if (col >= grid.width || row >= grid.height) return std::nullopt;
  return row * grid.width + col;
}

std::vector<std::uint8_t> occlusionCells(
    const CrosswalkOccupancyGrid &grid,
    const std::vector<std::vector<CrosswalkPoint>> &attention_areas,
    double minimum_square_m,
    const std::vector<std::vector<CrosswalkPoint>> &lane_divisions,
    const std::vector<IntersectionOcclusionBlocker> &parked_vehicles,
    double parked_speed_threshold_mps) {
  std::vector<Polygon> areas;
  for (const auto &area : attention_areas) {
    if (area.size() < 3) continue;
    Polygon polygon;
    for (const auto &point : area) {
      bg::append(polygon.outer(), Point(point.x, point.y));
    }
    bg::correct(polygon);
    areas.push_back(std::move(polygon));
  }
  std::vector<std::uint8_t> eligible(grid.values.size(), 0);
  for (std::size_t row = 0; row < grid.height; ++row) {
    for (std::size_t col = 0; col < grid.width; ++col) {
      const auto point = cellCenter(grid, row, col);
      eligible[row * grid.width + col] =
          grid.values[row * grid.width + col] >= 43 &&
          grid.values[row * grid.width + col] < 58 &&
          std::any_of(areas.begin(), areas.end(), [&](const Polygon &area) {
            return bg::covered_by(Point(point.x, point.y), area);
          });
    }
  }
  const auto unblocked_eligible = eligible;
  std::vector<Polygon> blockers;
  for (const auto &vehicle : parked_vehicles) {
    if (std::abs(vehicle.speed_mps) >= parked_speed_threshold_mps) continue;
    const double cosine = std::cos(vehicle.pose.yaw_rad);
    const double sine = std::sin(vehicle.pose.yaw_rad);
    Polygon polygon;
    for (const auto &[forward, lateral] : {
             std::pair{-vehicle.length_m / 2.0, -vehicle.width_m / 2.0},
             std::pair{vehicle.length_m / 2.0, -vehicle.width_m / 2.0},
             std::pair{vehicle.length_m / 2.0, vehicle.width_m / 2.0},
             std::pair{-vehicle.length_m / 2.0, vehicle.width_m / 2.0}}) {
      bg::append(polygon.outer(), Point(
          vehicle.pose.x + cosine * forward - sine * lateral,
          vehicle.pose.y + sine * forward + cosine * lateral));
    }
    bg::correct(polygon);
    blockers.push_back(std::move(polygon));
  }
  // 원본과 같이 탐색선을 교차로 쪽부터 훑고 차량 뒤쪽 격자를 지웁니다.
  if (!blockers.empty()) {
    for (const auto &division : lane_divisions) {
      if (division.size() < 2) continue;
      double length = 0.0;
      double maximum_turn = 0.0;
      for (std::size_t index = 1; index < division.size(); ++index) {
        const double dx = division[index].x - division[index - 1].x;
        const double dy = division[index].y - division[index - 1].y;
        length += std::hypot(dx, dy);
        if (index > 1) {
          const double previous_dx =
              division[index - 1].x - division[index - 2].x;
          const double previous_dy =
              division[index - 1].y - division[index - 2].y;
          maximum_turn = std::max(maximum_turn, std::abs(std::atan2(
              previous_dx * dy - previous_dy * dx,
              previous_dx * dx + previous_dy * dy)));
        }
      }
      // 원본의 곡선 차로 제외를 보수적으로 근사해 큰 꺾임에서는 차폐하지 않습니다.
      if (maximum_turn > 0.25) continue;
      bool blocked = false;
      for (double arc = 0.0; arc <= length; arc += grid.resolution_m) {
        const auto point = pointAtArc(division, arc);
        if (!blocked) {
          blocked = std::any_of(blockers.begin(), blockers.end(),
              [&](const Polygon &polygon) {
                return bg::covered_by(Point(point.x, point.y), polygon);
              });
        }
        if (blocked) {
          const auto index = cellIndex(grid, point);
          if (index) eligible[*index] = 0;
        }
      }
    }
  }
  const auto side = static_cast<std::size_t>(
      std::ceil(minimum_square_m / grid.resolution_m));
  std::vector<std::uint8_t> mask(grid.values.size(), 0);
  if (side > grid.width || side > grid.height) return mask;
  for (std::size_t row = 0; row + side <= grid.height; ++row) {
    for (std::size_t col = 0; col + side <= grid.width; ++col) {
      bool valid = true;
      for (std::size_t dy = 0; dy < side && valid; ++dy) {
        for (std::size_t dx = 0; dx < side; ++dx) {
          if (!unblocked_eligible[(row + dy) * grid.width + col + dx]) {
            valid = false;
            break;
          }
        }
      }
      if (!valid) continue;
      for (std::size_t dy = 0; dy < side; ++dy) {
        for (std::size_t dx = 0; dx < side; ++dx) {
          mask[(row + dy) * grid.width + col + dx] = 1;
        }
      }
    }
  }
  // 차폐 뒤에도 원본의 최소 차량 크기를 담는 미확인 영역만 유지합니다.
  std::vector<std::uint8_t> visited(mask.size(), 0);
  std::vector<std::uint8_t> surviving(mask.size(), 0);
  for (std::size_t start = 0; start < mask.size(); ++start) {
    if (!mask[start] || !eligible[start] || visited[start]) continue;
    std::vector<std::size_t> component{start};
    visited[start] = 1;
    std::size_t min_row = grid.height, max_row = 0;
    std::size_t min_col = grid.width, max_col = 0;
    for (std::size_t position = 0; position < component.size(); ++position) {
      const auto current = component[position];
      const auto row = current / grid.width;
      const auto col = current % grid.width;
      min_row = std::min(min_row, row);
      max_row = std::max(max_row, row);
      min_col = std::min(min_col, col);
      max_col = std::max(max_col, col);
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (!dx && !dy) continue;
          const auto next_row = static_cast<std::ptrdiff_t>(row) + dy;
          const auto next_col = static_cast<std::ptrdiff_t>(col) + dx;
          if (next_row < 0 || next_col < 0 ||
              next_row >= static_cast<std::ptrdiff_t>(grid.height) ||
              next_col >= static_cast<std::ptrdiff_t>(grid.width)) continue;
          const auto next = static_cast<std::size_t>(next_row) * grid.width +
                            static_cast<std::size_t>(next_col);
          if (mask[next] && eligible[next] && !visited[next]) {
            visited[next] = 1;
            component.push_back(next);
          }
        }
      }
    }
    const double extent_x = (max_col - min_col + 1) * grid.resolution_m;
    const double extent_y = (max_row - min_row + 1) * grid.resolution_m;
    const double area = component.size() *
                        grid.resolution_m * grid.resolution_m;
    if (area < 1.9 * 2.5 || std::min(extent_x, extent_y) < 1.9 ||
        std::max(extent_x, extent_y) < 2.5) continue;
    for (const auto index : component) surviving[index] = 1;
  }
  return surviving;
}

double projectedArc(const std::vector<CrosswalkPoint> &line,
                    CrosswalkPoint point) {
  double best = std::numeric_limits<double>::infinity();
  double selected = 0.0;
  double accumulated = 0.0;
  for (std::size_t index = 1; index < line.size(); ++index) {
    const double dx = line[index].x - line[index - 1].x;
    const double dy = line[index].y - line[index - 1].y;
    const double length2 = dx * dx + dy * dy;
    if (length2 <= 1.0e-12) continue;
    const double ratio = std::clamp(
        ((point.x - line[index - 1].x) * dx +
         (point.y - line[index - 1].y) * dy) / length2, 0.0, 1.0);
    const double lateral = std::hypot(
        point.x - line[index - 1].x - ratio * dx,
        point.y - line[index - 1].y - ratio * dy);
    if (lateral < best) {
      best = lateral;
      selected = accumulated + ratio * std::sqrt(length2);
    }
    accumulated += std::sqrt(length2);
  }
  return selected;
}

CrosswalkPoint pointAtArc(
    const std::vector<CrosswalkPoint> &line, double arc) {
  double accumulated = 0.0;
  for (std::size_t index = 1; index < line.size(); ++index) {
    const double dx = line[index].x - line[index - 1].x;
    const double dy = line[index].y - line[index - 1].y;
    const double length = std::hypot(dx, dy);
    if (accumulated + length >= arc) {
      const double ratio = length > 1.0e-9
          ? (arc - accumulated) / length : 0.0;
      return {line[index - 1].x + ratio * dx,
              line[index - 1].y + ratio * dy};
    }
    accumulated += length;
  }
  return line.back();
}

struct OcclusionInterval {
  double distance_m = std::numeric_limits<double>::infinity();
  CrosswalkPoint first;
  CrosswalkPoint visible_end;
};

OcclusionInterval nearestOcclusionAlongDivision(
    const CrosswalkOccupancyGrid &grid,
    const std::vector<std::uint8_t> &mask,
    const std::vector<std::vector<CrosswalkPoint>> &divisions,
    const Trajectory &ego_path, double limit_m) {
  Line path_line;
  for (const auto &point : ego_path) {
    bg::append(path_line, Point(point.x, point.y));
  }
  OcclusionInterval nearest;
  for (const auto &division : divisions) {
    if (division.size() < 2) continue;
    Line division_line;
    double length = 0.0;
    for (const auto &point : division) {
      bg::append(division_line, Point(point.x, point.y));
    }
    for (std::size_t index = 1; index < division.size(); ++index) {
      length += std::hypot(
          division[index].x - division[index - 1].x,
          division[index].y - division[index - 1].y);
    }
    std::vector<Point> intersections;
    bg::intersection(division_line, path_line, intersections);
    if (intersections.empty()) continue;
    const double crossing = projectedArc(
        division, {intersections.front().x(), intersections.front().y()});
    const auto projection = pointAtArc(division, crossing);
    const double step = grid.resolution_m / 2.0;
    bool contiguous = false;
    for (double distance = 0.0;
         distance <= limit_m && crossing + distance <= length;
         distance += step) {
      const auto sample = pointAtArc(division, crossing + distance);
      const auto index = cellIndex(grid, sample);
      const bool occluded = index && mask[*index];
      if (occluded && distance < nearest.distance_m) {
        nearest = {distance, sample, projection};
        contiguous = true;
      } else if (occluded && contiguous) {
        nearest.visible_end = sample;
      } else if (!occluded) {
        contiguous = false;
      }
    }
  }
  return nearest;
}

Polygon vehiclePolygon(const IntersectionOcclusionBlocker &vehicle) {
  Polygon polygon;
  const double cosine = std::cos(vehicle.pose.yaw_rad);
  const double sine = std::sin(vehicle.pose.yaw_rad);
  for (const auto &[forward, lateral] : {
           std::pair{-vehicle.length_m / 2.0, -vehicle.width_m / 2.0},
           std::pair{vehicle.length_m / 2.0, -vehicle.width_m / 2.0},
           std::pair{vehicle.length_m / 2.0, vehicle.width_m / 2.0},
           std::pair{-vehicle.length_m / 2.0, vehicle.width_m / 2.0}}) {
    bg::append(polygon.outer(), Point(
        vehicle.pose.x + cosine * forward - sine * lateral,
        vehicle.pose.y + sine * forward + cosine * lateral));
  }
  bg::correct(polygon);
  return polygon;
}

}  // 익명 네임스페이스

IntersectionOcclusionReleaseState::IntersectionOcclusionReleaseState(
    IntersectionOcclusionReleaseParameters parameters)
    : parameters_(parameters) {
  if (!std::isfinite(parameters.static_release_delay_s) ||
      parameters.static_release_delay_s < 0.0) {
    throw std::invalid_argument("교차로 가림 해제 시간 설정이 잘못되었습니다");
  }
}

bool IntersectionOcclusionReleaseState::update(
    IntersectionOcclusionStatus status, bool has_traffic_light,
    bool collision, bool stopped_at_default_line,
    bool stopped_at_peeking_line, double now_s) {
  if (!std::isfinite(now_s)) {
    throw std::invalid_argument("교차로 가림 시각이 잘못되었습니다");
  }
  if (!has_traffic_light) return false;
  if (stopped_at_default_line) passed_default_line_ = true;
  if (collision) {
    released_ = false;
    static_wait_started_s_.reset();
  } else if (!released_ && passed_default_line_ &&
             stopped_at_peeking_line &&
             status == IntersectionOcclusionStatus::Static) {
    if (!static_wait_started_s_ || now_s < *static_wait_started_s_) {
      static_wait_started_s_ = now_s;
    } else if (now_s - *static_wait_started_s_ >
               parameters_.static_release_delay_s) {
      released_ = true;
      static_wait_started_s_.reset();
    }
  }
  return released_ && !collision;
}

void IntersectionOcclusionReleaseState::reset() {
  passed_default_line_ = false;
  released_ = false;
  static_wait_started_s_.reset();
}

IntersectionOcclusionStatus classifyIntersectionOcclusion(
    const CrosswalkOccupancyGrid &grid,
    const std::vector<std::vector<CrosswalkPoint>> &attention_areas,
    const Trajectory &ego_path, CrosswalkPoint ego_position,
    IntersectionOcclusionParameters parameters,
    const std::vector<std::vector<CrosswalkPoint>> &lane_divisions,
    const std::vector<IntersectionOcclusionBlocker> &attention_vehicles) {
  if (!std::isfinite(parameters.minimum_unknown_square_m) ||
      parameters.minimum_unknown_square_m <= 0.0 ||
      !std::isfinite(parameters.maximum_distance_to_path_m) ||
      parameters.maximum_distance_to_path_m <= 0.0 ||
      !std::isfinite(parameters.maximum_clearance_distance_m) ||
      parameters.maximum_clearance_distance_m <= 0.0 ||
      !std::isfinite(parameters.parked_vehicle_speed_threshold_mps) ||
      parameters.parked_vehicle_speed_threshold_mps <= 0.0 ||
      grid.resolution_m <= 0.0 || grid.width == 0 || grid.height == 0 ||
      grid.values.size() != grid.width * grid.height) {
    throw std::invalid_argument("교차로 가림 입력이 잘못되었습니다");
  }
  if (ego_path.size() < 2) return IntersectionOcclusionStatus::Clear;
  if (!std::isfinite(ego_position.x) || !std::isfinite(ego_position.y)) {
    throw std::invalid_argument("교차로 자차 위치가 잘못되었습니다");
  }
  std::vector<IntersectionOcclusionBlocker> parked_vehicles;
  for (const auto &vehicle : attention_vehicles) {
    if (!std::isfinite(vehicle.pose.x) ||
        !std::isfinite(vehicle.pose.y) ||
        !std::isfinite(vehicle.pose.yaw_rad) ||
        !std::isfinite(vehicle.speed_mps) ||
        !std::isfinite(vehicle.length_m) || vehicle.length_m <= 0.0 ||
        !std::isfinite(vehicle.width_m) || vehicle.width_m <= 0.0) {
      throw std::invalid_argument("교차로 주목 차량 입력이 잘못되었습니다");
    }
    if (std::abs(vehicle.speed_mps) <
        parameters.parked_vehicle_speed_threshold_mps) {
      parked_vehicles.push_back(vehicle);
    }
  }
  if (!lane_divisions.empty()) {
    const auto mask = occlusionCells(
        grid, attention_areas, parameters.minimum_unknown_square_m,
        lane_divisions, parked_vehicles,
        parameters.parked_vehicle_speed_threshold_mps);
    const auto nearest = nearestOcclusionAlongDivision(
        grid, mask, lane_divisions, ego_path,
        parameters.maximum_clearance_distance_m);
    if (!std::isfinite(nearest.distance_m)) {
      return IntersectionOcclusionStatus::Clear;
    }
    Polygon triangle;
    bg::append(triangle.outer(), Point(ego_position.x, ego_position.y));
    bg::append(triangle.outer(), Point(nearest.first.x, nearest.first.y));
    bg::append(triangle.outer(), Point(nearest.visible_end.x,
                                       nearest.visible_end.y));
    bg::correct(triangle);
    for (const auto &vehicle : attention_vehicles) {
      if (bg::intersects(vehiclePolygon(vehicle), triangle)) {
        return IntersectionOcclusionStatus::Dynamic;
      }
    }
    return IntersectionOcclusionStatus::Static;
  }
  CrosswalkOcclusionParameters occupancy_parameters;
  occupancy_parameters.minimum_size_m = parameters.minimum_unknown_square_m;
  occupancy_parameters.maximum_distance_to_path_m =
      parameters.maximum_distance_to_path_m;
  occupancy_parameters.ignore_behind_dynamic_objects = false;
  std::vector<CrosswalkPoint> path;
  path.reserve(ego_path.size());
  for (const auto &point : ego_path) path.push_back({point.x, point.y});
  return isCrosswalkOccluded(grid, attention_areas, occupancy_parameters,
                             {}, path)
             ? IntersectionOcclusionStatus::Static
             : IntersectionOcclusionStatus::Clear;
}

bool detectIntersectionOcclusion(
    const CrosswalkOccupancyGrid &grid,
    const std::vector<std::vector<CrosswalkPoint>> &attention_areas,
    const Trajectory &ego_path, IntersectionOcclusionParameters parameters,
    const std::vector<std::vector<CrosswalkPoint>> &lane_divisions,
    const std::vector<IntersectionOcclusionBlocker> &parked_vehicles) {
  const CrosswalkPoint ego_position = ego_path.empty()
      ? CrosswalkPoint{} : CrosswalkPoint{ego_path.front().x,
                                         ego_path.front().y};
  return classifyIntersectionOcclusion(
             grid, attention_areas, ego_path, ego_position, parameters,
             lane_divisions, parked_vehicles) !=
         IntersectionOcclusionStatus::Clear;
}

}  // planning 네임스페이스
