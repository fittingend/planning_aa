// SPDX-License-Identifier: Apache-2.0

#include "planning/behavior_dynamic_drivable_area.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace planning {
namespace {
struct LocalPoint { double arc = 0.0; double lateral = 0.0; };

LocalPoint project(const Trajectory &reference, double x, double y) {
  LocalPoint selected;
  double traveled = 0.0;
  double nearest = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < reference.size(); ++i) {
    const auto &a = reference[i - 1];
    const auto &b = reference[i];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length = std::hypot(dx, dy);
    if (length < 1e-9) continue;
    const double ratio = std::clamp(
        ((x - a.x) * dx + (y - a.y) * dy) / (length * length),
        0.0, 1.0);
    const double error = std::hypot(
        x - a.x - ratio * dx, y - a.y - ratio * dy);
    if (error < nearest) {
      nearest = error;
      selected = {traveled + ratio * length,
                  (dx * (y - a.y) - dy * (x - a.x)) / length};
    }
    traveled += length;
  }
  return selected;
}

struct PathFrame {
  double x = 0.0;
  double y = 0.0;
  double normal_x = 0.0;
  double normal_y = 1.0;
};

PathFrame pathFrame(const Trajectory &path, double arc) {
  double traveled = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double dx = path[i].x - path[i - 1].x;
    const double dy = path[i].y - path[i - 1].y;
    const double length = std::hypot(dx, dy);
    if (length < 1e-9) continue;
    if (arc <= traveled + length || i + 1 == path.size()) {
      const double ratio = std::clamp(
          (arc - traveled) / length, 0.0, 1.0);
      return {path[i - 1].x + ratio * dx,
              path[i - 1].y + ratio * dy,
              -dy / length, dx / length};
    }
    traveled += length;
  }
  return {path.front().x, path.front().y, 0.0, 1.0};
}

bool forwardBound(const std::vector<LocalPoint> &points) {
  for (std::size_t i = 1; i < points.size(); ++i)
    if (points[i].arc + 1e-3 < points[i - 1].arc) return false;
  return true;
}

double boundLateral(const std::vector<LocalPoint> &points, double arc) {
  if (arc <= points.front().arc) return points.front().lateral;
  for (std::size_t i = 1; i < points.size(); ++i) {
    if (arc > points[i].arc) continue;
    const double length = points[i].arc - points[i - 1].arc;
    if (length < 1e-9) return points[i].lateral;
    const double ratio = (arc - points[i - 1].arc) / length;
    return points[i - 1].lateral +
           ratio * (points[i].lateral - points[i - 1].lateral);
  }
  return points.back().lateral;
}

std::optional<std::pair<double, double>> polygonSlice(
    const BehaviorSafetyPolygon &polygon, const PathFrame &frame) {
  std::vector<double> lateral;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto &a = polygon[i];
    const auto &b = polygon[(i + 1) % polygon.size()];
    const double edge_x = b.first - a.first;
    const double edge_y = b.second - a.second;
    const double from_x = a.first - frame.x;
    const double from_y = a.second - frame.y;
    const double denominator =
        frame.normal_x * edge_y - frame.normal_y * edge_x;
    if (std::abs(denominator) < 1e-9) {
      // The edge lies on the station line, so both endpoints bound the slice.
      const double separation =
          from_x * frame.normal_y - from_y * frame.normal_x;
      if (std::abs(separation) < 1e-7) {
        lateral.push_back(from_x * frame.normal_x +
                          from_y * frame.normal_y);
        lateral.push_back((b.first - frame.x) * frame.normal_x +
                          (b.second - frame.y) * frame.normal_y);
      }
      continue;
    }
    const double edge_ratio =
        (from_x * frame.normal_y - from_y * frame.normal_x) /
        denominator;
    if (edge_ratio < -1e-8 || edge_ratio > 1.0 + 1e-8)
      continue;
    const double distance =
        (from_x * edge_y - from_y * edge_x) / denominator;
    lateral.push_back(distance);
  }
  if (lateral.empty()) return std::nullopt;
  const auto [minimum, maximum] =
      std::minmax_element(lateral.begin(), lateral.end());
  return std::pair<double, double>{*minimum, *maximum};
}

struct LocalPolygon {
  std::vector<LocalPoint> points;
  BehaviorSafetyPolygon world;
  bool left = false;
};

std::vector<PathOptimizerPoint2D> extractBound(
    const Trajectory &reference,
    const std::vector<PathOptimizerPoint2D> &original,
    const std::vector<LocalPoint> &local_bound,
    const std::vector<LocalPolygon> &polygons, bool left) {
  std::vector<double> stations;
  for (const auto &point : local_bound) stations.push_back(point.arc);
  bool has_side_polygon = false;
  for (const auto &polygon : polygons) {
    if (polygon.left != left) continue;
    has_side_polygon = true;
    for (const auto &point : polygon.points) {
      stations.push_back(point.arc);
      stations.push_back(std::max(local_bound.front().arc,
                                  point.arc - 0.05));
      stations.push_back(std::min(local_bound.back().arc,
                                  point.arc + 0.05));
    }
  }
  if (has_side_polygon) {
    double reference_arc = 0.0;
    for (std::size_t i = 1; i < reference.size(); ++i) {
      reference_arc += std::hypot(reference[i].x - reference[i - 1].x,
                                  reference[i].y - reference[i - 1].y);
      if (reference_arc >= local_bound.front().arc &&
          reference_arc <= local_bound.back().arc)
        stations.push_back(reference_arc);
    }
  }
  std::sort(stations.begin(), stations.end());
  stations.erase(std::unique(stations.begin(), stations.end(),
                             [](double a, double b) {
                               return std::abs(a - b) < 1e-4;
                             }), stations.end());
  std::vector<PathOptimizerPoint2D> result;
  result.reserve(stations.size());
  for (const double arc : stations) {
    if (arc + 1e-6 < local_bound.front().arc ||
        arc - 1e-6 > local_bound.back().arc) continue;
    const double original_lateral = boundLateral(local_bound, arc);
    double lateral = original_lateral;
    const auto frame = pathFrame(reference, arc);
    for (const auto &polygon : polygons) {
      if (polygon.left != left) continue;
      const auto slice = polygonSlice(polygon.world, frame);
      if (!slice) continue;
      if (left && slice->first < lateral)
        lateral = slice->first;
      if (!left && slice->second > lateral)
        lateral = slice->second;
    }
    PathOptimizerPoint2D base = original.front();
    if (arc >= local_bound.back().arc) {
      base = original.back();
    } else {
      for (std::size_t i = 1; i < local_bound.size(); ++i) {
        if (arc > local_bound[i].arc) continue;
        const double length = local_bound[i].arc - local_bound[i - 1].arc;
        const double ratio = length < 1e-9 ? 0.0 :
            std::clamp((arc - local_bound[i - 1].arc) / length,
                       0.0, 1.0);
        base = {original[i - 1].x + ratio *
                    (original[i].x - original[i - 1].x),
                original[i - 1].y + ratio *
                    (original[i].y - original[i - 1].y)};
        break;
      }
    }
    result.push_back({base.x + frame.normal_x *
                                  (lateral - original_lateral),
                      base.y + frame.normal_y *
                                  (lateral - original_lateral)});
  }
  return result.size() >= 2 ? result : original;
}
}  // namespace

BehaviorDynamicDrivableArea extractDynamicDrivableArea(
    const Trajectory &reference,
    const std::vector<PathOptimizerPoint2D> &left_bound,
    const std::vector<PathOptimizerPoint2D> &right_bound,
    const std::vector<BehaviorSafetyPolygon> &exclusion_polygons,
    const std::vector<bool> &collision_on_left) {
  BehaviorDynamicDrivableArea result{left_bound, right_bound, 0};
  if (reference.size() < 2 || left_bound.size() < 2 ||
      right_bound.size() < 2 || exclusion_polygons.empty()) return result;
  std::vector<LocalPoint> left, right;
  for (const auto &point : left_bound)
    left.push_back(project(reference, point.x, point.y));
  for (const auto &point : right_bound)
    right.push_back(project(reference, point.x, point.y));
  if (!forwardBound(left) || !forwardBound(right)) return result;

  std::vector<LocalPolygon> polygons;
  for (std::size_t index = 0; index < exclusion_polygons.size(); ++index) {
    const auto &world = exclusion_polygons[index];
    if (world.size() < 3) continue;
    LocalPolygon polygon;
    polygon.world = world;
    double lateral_sum = 0.0;
    for (const auto &[x, y] : world) {
      polygon.points.push_back(project(reference, x, y));
      lateral_sum += polygon.points.back().lateral;
    }
    polygon.left = collision_on_left.size() == exclusion_polygons.size() ?
        collision_on_left[index] : lateral_sum > 0.0;
    polygons.push_back(std::move(polygon));
  }
  if (polygons.empty()) return result;
  result.left_bound = extractBound(
      reference, left_bound, left, polygons, true);
  result.right_bound = extractBound(
      reference, right_bound, right, polygons, false);
  result.applied_obstacles = polygons.size();
  return result;
}

}  // namespace planning
