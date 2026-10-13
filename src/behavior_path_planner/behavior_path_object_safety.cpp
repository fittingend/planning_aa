// SPDX-License-Identifier: Apache-2.0
// 원본 행동 경로 안전 검사의 직사각형 RSS 분기를 계산합니다.

#include "planning/behavior_path_object_safety.hpp"

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

Polygon rectangle(const BehaviorSafetyPose &pose, double front, double rear,
                  double left, double right) {
  Polygon polygon;
  const double cosine = std::cos(pose.yaw_rad);
  const double sine = std::sin(pose.yaw_rad);
  for (const auto &[along, across] : {
           std::pair{front, left}, std::pair{front, -right},
           std::pair{-rear, -right}, std::pair{-rear, left}}) {
    polygon.outer().emplace_back(
        pose.x + along * cosine - across * sine,
        pose.y + along * sine + across * cosine);
  }
  bg::correct(polygon);
  return polygon;
}

Polygon alongPathRectangle(
    const BehaviorSafetyPose &pose, const BehaviorSafetyFootprint &vehicle,
    double longitude, double lateral, bool stopped,
    const std::vector<BehaviorSafetyPose> &path) {
  const double front = vehicle.front_m +
      (stopped ? longitude * 0.5 : longitude);
  const double rear = vehicle.rear_m +
      (stopped ? longitude * 0.5 : 0.0);
  const double half_width = vehicle.half_width_m + lateral;
  const auto fallback = [&]() {
    return rectangle(pose, front, rear, half_width, half_width);
  };
  if (path.size() < 2) return fallback();

  std::vector<double> arcs(path.size(), 0.0);
  for (std::size_t index = 1; index < path.size(); ++index) {
    const double segment = std::hypot(
        path[index].x - path[index - 1].x,
        path[index].y - path[index - 1].y);
    if (!std::isfinite(segment) || segment < 1.0e-9) return fallback();
    arcs[index] = arcs[index - 1] + segment;
  }
  double nearest_error = std::numeric_limits<double>::infinity();
  double start_arc = 0.0;
  std::size_t start_index = 0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const double dx = path[index + 1].x - path[index].x;
    const double dy = path[index + 1].y - path[index].y;
    const double length = arcs[index + 1] - arcs[index];
    const double fraction = std::clamp(
        ((pose.x - path[index].x) * dx +
         (pose.y - path[index].y) * dy) / (length * length),
        0.0, 1.0);
    const double error = std::hypot(
        pose.x - path[index].x - fraction * dx,
        pose.y - path[index].y - fraction * dy);
    if (error < nearest_error) {
      nearest_error = error;
      start_arc = arcs[index] + fraction * length;
      start_index = index;
    }
  }
  const double end_arc = start_arc + longitude;
  if (!std::isfinite(end_arc) || end_arc > arcs.back()) return fallback();
  std::size_t end_index = start_index;
  while (end_index + 1 < path.size() && arcs[end_index + 1] < end_arc) {
    ++end_index;
  }
  if (end_index + 1 >= path.size()) end_index = path.size() - 2;
  const double fraction = std::clamp(
      (end_arc - arcs[end_index]) /
          (arcs[end_index + 1] - arcs[end_index]),
      0.0, 1.0);
  const double yaw_difference = std::remainder(
      path[end_index + 1].yaw_rad - path[end_index].yaw_rad,
      2.0 * std::acos(-1.0));
  const BehaviorSafetyPose end_pose{
      0.0,
      path[end_index].x + fraction *
          (path[end_index + 1].x - path[end_index].x),
      path[end_index].y + fraction *
          (path[end_index + 1].y - path[end_index].y),
      path[end_index].yaw_rad + fraction * yaw_difference, 0.0};
  const auto offset = [](const BehaviorSafetyPose &at,
                         double along, double across) {
    const double cosine = std::cos(at.yaw_rad);
    const double sine = std::sin(at.yaw_rad);
    return Point{at.x + along * cosine - across * sine,
                 at.y + along * sine + across * cosine};
  };
  Polygon polygon;
  polygon.outer().push_back(offset(pose, -rear, half_width));
  for (std::size_t index = start_index + 1; index <= end_index; ++index) {
    polygon.outer().push_back(offset(path[index], 0.0, half_width));
  }
  polygon.outer().push_back(offset(end_pose, vehicle.front_m, half_width));
  polygon.outer().push_back(offset(end_pose, vehicle.front_m, -half_width));
  for (std::size_t index = end_index; index > start_index; --index) {
    polygon.outer().push_back(offset(path[index], 0.0, -half_width));
  }
  polygon.outer().push_back(offset(pose, -rear, -half_width));
  bg::correct(polygon);
  return bg::is_valid(polygon) ? polygon : fallback();
}

Polygon objectPolygon(const BehaviorSafetyPose &pose,
                      const BehaviorSafetyObject &object) {
  if (object.shape_type < 0 || object.shape_type > 2 ||
      (object.shape_type == 0 &&
       (object.length_m <= 0.0 || object.width_m <= 0.0)) ||
      (object.shape_type == 1 && object.length_m <= 0.0) ||
      (object.shape_type == 2 && object.footprint.empty()) ||
      !std::isfinite(object.length_m) || !std::isfinite(object.width_m) ||
      !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
      !std::isfinite(pose.yaw_rad)) {
    throw std::invalid_argument("객체 외형 매개변수가 유효하지 않습니다");
  }
  if (object.shape_type != 2) {
    if (object.shape_type == 1) {
      Polygon circle;
      constexpr int sides = 6;
      for (int index = 0; index < sides; ++index) {
        const double angle = 2.0 * std::acos(-1.0) * index / sides +
                             std::acos(-1.0) / sides;
        circle.outer().emplace_back(
            pose.x + std::cos(angle) * object.length_m * 0.5,
            pose.y + std::sin(angle) * object.length_m * 0.5);
      }
      bg::correct(circle);
      return circle;
    }
    return rectangle(pose, object.length_m * 0.5, object.length_m * 0.5,
                     object.width_m * 0.5, object.width_m * 0.5);
  }
  Polygon polygon;
  const double cosine = std::cos(pose.yaw_rad);
  const double sine = std::sin(pose.yaw_rad);
  for (const auto &[along, across] : object.footprint) {
    if (!std::isfinite(along) || !std::isfinite(across)) {
      throw std::invalid_argument("객체 외형 다각형이 유효하지 않습니다");
    }
    polygon.outer().emplace_back(
        pose.x + along * cosine - across * sine,
        pose.y + along * sine + across * cosine);
  }
  bg::correct(polygon);
  if (!bg::is_valid(polygon)) {
    throw std::invalid_argument("객체 외형 다각형이 유효하지 않습니다");
  }
  return polygon;
}

std::pair<std::pair<double, double>, std::pair<double, double>>
objectBounds(const Polygon &polygon, const BehaviorSafetyPose &pose) {
  const double cosine = std::cos(pose.yaw_rad);
  const double sine = std::sin(pose.yaw_rad);
  double minimum_x = std::numeric_limits<double>::infinity();
  double maximum_x = -minimum_x;
  double minimum_y = minimum_x;
  double maximum_y = -minimum_x;
  for (const auto &point : polygon.outer()) {
    const double dx = point.x() - pose.x;
    const double dy = point.y() - pose.y;
    const double x = dx * cosine + dy * sine;
    const double y = -dx * sine + dy * cosine;
    minimum_x = std::min(minimum_x, x);
    maximum_x = std::max(maximum_x, x);
    minimum_y = std::min(minimum_y, y);
    maximum_y = std::max(maximum_y, y);
  }
  return {{minimum_x, maximum_x}, {minimum_y, maximum_y}};
}

std::optional<BehaviorSafetyPose> interpolate(
    const std::vector<BehaviorSafetyPose> &path, double time) {
  if (path.size() < 2 || time < 0.0) return std::nullopt;
  for (std::size_t index = 1; index < path.size(); ++index) {
    if (time > path[index].time_s + 1.0e-6) continue;
    const auto &first = path[index - 1];
    const auto &last = path[index];
    const double duration = last.time_s - first.time_s;
    const double ratio = duration > 1.0e-9 ?
        std::clamp((time - first.time_s) / duration, 0.0, 1.0) : 0.0;
    const double difference = std::remainder(
        last.yaw_rad - first.yaw_rad, 2.0 * std::acos(-1.0));
    return BehaviorSafetyPose{
        time,
        first.x + ratio * (last.x - first.x),
        first.y + ratio * (last.y - first.y),
        first.yaw_rad + ratio * difference,
        first.velocity_mps + ratio *
            (last.velocity_mps - first.velocity_mps)};
  }
  return std::nullopt;
}

double stoppingDistance(double velocity, double deceleration) {
  const double limited = deceleration < -1.0e-3 ? deceleration : -1.0;
  return -velocity * velocity / (2.0 * limited);
}

}  // 익명 네임스페이스

BehaviorSafetyPolygon behaviorSafetyWorldPolygon(
    const BehaviorSafetyPose &pose, const BehaviorSafetyObject &object) {
  const auto polygon = objectPolygon(pose, object);
  BehaviorSafetyPolygon result;
  result.reserve(polygon.outer().size());
  for (const auto &point : polygon.outer()) {
    result.emplace_back(point.x(), point.y());
  }
  return result;
}

bool behaviorFootprintCollidesWithObjects(
    const BehaviorSafetyPose &ego_pose,
    const BehaviorSafetyFootprint &ego_vehicle,
    const std::vector<BehaviorSafetyPolygon> &object_polygons,
    double margin_m) {
  if (!std::isfinite(ego_pose.x) || !std::isfinite(ego_pose.y) ||
      !std::isfinite(ego_pose.yaw_rad) ||
      !std::isfinite(ego_vehicle.front_m) ||
      !std::isfinite(ego_vehicle.rear_m) ||
      !std::isfinite(ego_vehicle.half_width_m) ||
      ego_vehicle.front_m <= 0.0 || ego_vehicle.rear_m < 0.0 ||
      ego_vehicle.half_width_m <= 0.0 ||
      !std::isfinite(margin_m) || margin_m < 0.0) {
    throw std::invalid_argument("차량 외곽 충돌 입력이 유효하지 않습니다");
  }
  const auto footprint = rectangle(
      ego_pose, ego_vehicle.front_m, ego_vehicle.rear_m,
      ego_vehicle.half_width_m, ego_vehicle.half_width_m);
  for (const auto &points : object_polygons) {
    if (points.size() < 3) {
      throw std::invalid_argument("정지 객체 다각형이 유효하지 않습니다");
    }
    Polygon object;
    for (const auto &[x, y] : points) {
      if (!std::isfinite(x) || !std::isfinite(y)) {
        throw std::invalid_argument("정지 객체 좌표가 유효하지 않습니다");
      }
      object.outer().emplace_back(x, y);
    }
    bg::correct(object);
    if (!bg::is_valid(object)) {
      throw std::invalid_argument("정지 객체 다각형이 유효하지 않습니다");
    }
    if (bg::distance(object, footprint.outer()) < margin_m) return true;
  }
  return false;
}

std::optional<BehaviorSafetyCollision> firstBehaviorPathObjectCollision(
    const std::vector<BehaviorSafetyPose> &ego_path,
    const BehaviorSafetyFootprint &ego_vehicle,
    const BehaviorSafetyObject &object,
    const BehaviorSafetyRssParameters &parameters,
    const BehaviorSafetyRegions &collision_regions,
    const std::vector<BehaviorSafetyPose> &planned_path) {
  if (ego_vehicle.front_m <= 0.0 || ego_vehicle.rear_m < 0.0 ||
      ego_vehicle.half_width_m <= 0.0 ||
      (object.shape_type == 0 &&
       (object.length_m <= 0.0 || object.width_m <= 0.0)) ||
      (object.shape_type == 1 && object.length_m <= 0.0) ||
      (object.shape_type == 2 && object.footprint.empty()) ||
      parameters.hysteresis_factor <= 0.0 ||
      object.shape_type < 0 || object.shape_type > 2 ||
      parameters.reaction_time_s < 0.0 ||
      parameters.safety_time_margin_s < 0.0 ||
      parameters.lateral_margin_m < 0.0 ||
      parameters.minimum_longitudinal_m < 0.0 ||
      parameters.yaw_difference_threshold_rad < 0.0 ||
      parameters.ego_velocity_limit_mps < 0.0 ||
      std::isnan(parameters.ego_velocity_limit_mps)) {
    throw std::invalid_argument("행동 경로 객체 안전 매개변수가 유효하지 않습니다");
  }
  std::vector<Polygon> regions;
  regions.reserve(collision_regions.size());
  for (const auto &points : collision_regions) {
    if (points.size() < 3) {
      throw std::invalid_argument("충돌 검사 차로 다각형이 유효하지 않습니다");
    }
    Polygon region;
    for (const auto &[x, y] : points) {
      if (!std::isfinite(x) || !std::isfinite(y)) {
        throw std::invalid_argument("충돌 검사 차로 좌표가 유효하지 않습니다");
      }
      region.outer().emplace_back(x, y);
    }
    bg::correct(region);
    if (!bg::is_valid(region)) {
      throw std::invalid_argument("충돌 검사 차로 다각형이 유효하지 않습니다");
    }
    regions.push_back(std::move(region));
  }
  const auto in_collision_regions = [&](const Polygon &polygon) {
    return regions.empty() || std::any_of(
        regions.begin(), regions.end(), [&](const Polygon &region) {
          return bg::intersects(polygon, region);
        });
  };
  if (parameters.extension_policy == BehaviorSafetyExtensionPolicy::AlongPath) {
    for (const auto &point : planned_path) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.yaw_rad)) {
        throw std::invalid_argument("경로를 따라 확장할 기준 자세가 유효하지 않습니다");
      }
    }
  }
  for (std::size_t index = 0; index < object.predicted.size(); ++index) {
    const auto &target = object.predicted[index];
    const auto ego = interpolate(ego_path, target.time_s);
    if (!ego) continue;
    const double yaw_difference = std::remainder(
        ego->yaw_rad - target.yaw_rad, 2.0 * std::acos(-1.0));
    if (std::fabs(yaw_difference) >
        parameters.yaw_difference_threshold_rad) continue;
    const auto ego_polygon = rectangle(
        *ego, ego_vehicle.front_m, ego_vehicle.rear_m,
        ego_vehicle.half_width_m, ego_vehicle.half_width_m);
    const auto target_polygon = objectPolygon(target, object);
    if (bg::intersects(ego_polygon, target_polygon)) {
      if (!in_collision_regions(target_polygon)) continue;
      return BehaviorSafetyCollision{
          BehaviorSafetyCollisionKind::DirectOverlap, index};
    }
    const double front_x = ego->x +
        ego_vehicle.front_m * std::cos(ego->yaw_rad);
    const double front_y = ego->y +
        ego_vehicle.front_m * std::sin(ego->yaw_rad);
    bool object_front = false;
    for (const auto &point : target_polygon.outer()) {
      const double longitudinal =
          (point.x() - front_x) * std::cos(ego->yaw_rad) +
          (point.y() - front_y) * std::sin(ego->yaw_rad);
      object_front |= longitudinal > 0.0;
    }
    const double ego_velocity = std::min(
        ego->velocity_mps, parameters.ego_velocity_limit_mps);
    const double front_velocity =
        object_front ? target.velocity_mps : ego_velocity;
    const double rear_velocity =
        object_front ? ego_velocity : target.velocity_mps;
    const double reaction_time = parameters.reaction_time_s +
                                 parameters.safety_time_margin_s;
    const double rss_distance =
        rear_velocity * reaction_time +
        stoppingDistance(rear_velocity,
                         parameters.rear_deceleration_mps2) -
        stoppingDistance(front_velocity,
                         parameters.front_deceleration_mps2);
    const double minimum_length =
        parameters.longitudinal_velocity_delta_time_s *
            std::abs(std::max(front_velocity, rear_velocity)) +
        parameters.minimum_longitudinal_m;
    const double longitude =
        std::max(rss_distance, minimum_length) *
        parameters.hysteresis_factor;
    const double lateral = parameters.lateral_margin_m *
                           parameters.hysteresis_factor;
    const bool stopped = target.velocity_mps < 0.3;
    Polygon extended;
    if (object_front) {
      extended = parameters.extension_policy ==
          BehaviorSafetyExtensionPolicy::AlongPath ?
          alongPathRectangle(
              *ego, ego_vehicle, longitude, lateral, stopped,
              planned_path) :
          rectangle(
              *ego,
              ego_vehicle.front_m + (stopped ? longitude * 0.5 : longitude),
              ego_vehicle.rear_m + (stopped ? longitude * 0.5 : 0.0),
              ego_vehicle.half_width_m + lateral,
              ego_vehicle.half_width_m + lateral);
    } else {
      const auto bounds = objectBounds(target_polygon, target);
      extended = rectangle(
          target,
          bounds.first.second +
              (stopped ? longitude * 0.5 : longitude),
          -bounds.first.first +
              (stopped ? longitude * 0.5 : 0.0),
          bounds.second.second + lateral,
          -bounds.second.first + lateral);
    }
    if (bg::intersects(object_front ? extended : ego_polygon,
                       object_front ? target_polygon : extended)) {
      if (!in_collision_regions(target_polygon)) continue;
      return BehaviorSafetyCollision{
          BehaviorSafetyCollisionKind::ExtendedOverlap,
          index, rss_distance, longitude, lateral, object_front};
    }
  }
  return std::nullopt;
}

std::optional<BehaviorSafetyCollision> firstBehaviorPathObjectCollisionAcrossPaths(
    const std::vector<BehaviorSafetyPose> &ego_path,
    const BehaviorSafetyFootprint &ego_vehicle,
    const BehaviorSafetyObject &object,
    const std::vector<BehaviorSafetyPredictedPath> &predicted_paths,
    bool use_all_predicted_paths,
    const BehaviorSafetyRssParameters &parameters,
    const BehaviorSafetyRegions &collision_regions,
    const std::vector<BehaviorSafetyPose> &planned_path) {
  if (predicted_paths.empty()) return std::nullopt;
  const auto check = [&](std::size_t index) {
    auto selected = object;
    selected.predicted = predicted_paths[index].poses;
    auto collision = firstBehaviorPathObjectCollision(
        ego_path, ego_vehicle, selected, parameters, collision_regions,
        planned_path);
    if (collision) collision->predicted_path_index = index;
    return collision;
  };
  if (!use_all_predicted_paths) {
    const auto highest = std::max_element(
        predicted_paths.begin(), predicted_paths.end(),
        [](const auto &first, const auto &second) {
          return first.confidence < second.confidence;
        });
    return check(static_cast<std::size_t>(highest - predicted_paths.begin()));
  }
  for (std::size_t index = 0; index < predicted_paths.size(); ++index) {
    if (auto collision = check(index)) return collision;
  }
  return std::nullopt;
}

}  // planning 네임스페이스
