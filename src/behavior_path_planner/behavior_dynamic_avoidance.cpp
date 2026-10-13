// SPDX-License-Identifier: Apache-2.0
// 원본 동적 회피 플러그인의 대상 진입·이탈 카운터를 옮깁니다.

#include "planning/behavior_dynamic_avoidance.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <utility>

namespace planning {
namespace {
namespace bg = boost::geometry;
using BgPoint = bg::model::d2::point_xy<double>;
using BgPolygon = bg::model::polygon<BgPoint>;
using BgMultiPolygon = bg::model::multi_polygon<BgPolygon>;
using BgMultiPoint = bg::model::multi_point<BgPoint>;
}  // 익명 네임스페이스

bool behaviorDynamicPathForward(const Trajectory &path) {
  if (path.size() < 2) return false;
  const auto &first = path.front();
  const double yaw = std::atan2(
      2.0 * (first.orientation_w * first.orientation_z +
             first.orientation_x * first.orientation_y),
      1.0 - 2.0 * (first.orientation_y * first.orientation_y +
                   first.orientation_z * first.orientation_z));
  const double dx = path[1].x - first.x;
  const double dy = path[1].y - first.y;
  return std::cos(yaw) * dx + std::sin(yaw) * dy > 0.0;
}

BehaviorDynamicObjectType behaviorDynamicObjectType(
    std::uint8_t classification,
    const BehaviorDynamicAvoidanceParameters &parameters) {
  if (classification >= parameters.avoid_types.size() ||
      !parameters.avoid_types[classification])
    return BehaviorDynamicObjectType::OutOfScope;
  if (classification == 0 || classification == 6 ||
      classification == 7)
    return BehaviorDynamicObjectType::Unregulated;
  return BehaviorDynamicObjectType::Regulated;
}

bool behaviorDynamicRoughAdmission(
    const BehaviorDynamicRoughAdmissionInput &input,
    const BehaviorDynamicAvoidanceParameters &parameters) {
  const auto type = behaviorDynamicObjectType(
      input.classification, parameters);
  if (type == BehaviorDynamicObjectType::OutOfScope ||
      !std::isfinite(input.tangent_speed_mps) ||
      !std::isfinite(input.normal_speed_mps) ||
      !std::isfinite(input.speed_norm_mps) ||
      !std::isfinite(input.yaw_difference_rad) ||
      !std::isfinite(input.center_distance_to_path_m) ||
      !std::isfinite(input.object_maximum_radius_m) ||
      !std::isfinite(input.ego_half_width_m))
    return false;
  if (type == BehaviorDynamicObjectType::Unregulated) {
    return input.speed_norm_mps >= parameters.minimum_speed_mps &&
        input.speed_norm_mps <= parameters.maximum_speed_mps &&
        std::abs(input.normal_speed_mps) <=
            parameters.maximum_pedestrian_lateral_speed_mps;
  }
  if (std::abs(input.tangent_speed_mps) <
          parameters.minimum_speed_mps ||
      std::abs(input.tangent_speed_mps) >
          parameters.maximum_speed_mps)
    return false;
  const double angle = std::abs(std::remainder(
      input.yaw_difference_rad, 2.0 * std::acos(-1.0)));
  const bool overtaking = input.tangent_speed_mps >= 0.0;
  const double maximum_crossing_angle = overtaking ?
      parameters.maximum_overtaking_crossing_angle_rad :
      parameters.maximum_oncoming_crossing_angle_rad;
  const double minimum_crossing_speed = overtaking ?
      parameters.minimum_overtaking_crossing_speed_mps :
      parameters.minimum_oncoming_crossing_speed_mps;
  if (minimum_crossing_speed < input.speed_norm_mps &&
      maximum_crossing_angle < angle &&
      maximum_crossing_angle < std::acos(-1.0) - angle)
    return false;
  const double outside_distance = std::max(
      0.0, input.center_distance_to_path_m -
               input.ego_half_width_m -
               input.object_maximum_radius_m);
  return outside_distance <=
      parameters.maximum_object_lateral_distance_m;
}

BehaviorDynamicCollisionTiming behaviorDynamicCollisionTiming(
    const BehaviorDynamicCollisionTimingInput &input) {
  const double relative = input.ego_speed_mps -
      input.object_path_speed_mps;
  const double ego_front_to_object_back =
      input.ego_to_object_path_m +
      input.object_longitudinal_min_m - input.ego_front_m;
  const double object_front_to_ego_back =
      -input.ego_to_object_path_m -
      input.object_longitudinal_max_m - input.ego_rear_m;
  double start = 0.0;
  if (ego_front_to_object_back > 0.0)
    start = ego_front_to_object_back / relative;
  else if (object_front_to_ego_back > 0.0)
    start = object_front_to_ego_back / -relative;
  const double ego_back_to_object_front =
      input.ego_to_object_path_m +
      input.object_longitudinal_max_m + input.ego_rear_m;
  const double object_back_to_ego_front =
      -input.ego_to_object_path_m -
      input.object_longitudinal_min_m + input.ego_front_m;
  const double end = relative > 0.0 ?
      ego_back_to_object_front / relative :
      object_back_to_ego_front / -relative;
  const double infinity = std::numeric_limits<double>::max();
  return {start < 0.0 ? infinity : start,
          end < 0.0 ? infinity : end};
}

bool behaviorDynamicObjectWillCutIn(
    const BehaviorDynamicCutInInput &input) {
  if (!input.predicted_path_enters_ego_corridor ||
      input.object_path_speed_mps <
          input.minimum_object_speed_mps)
    return false;
  const double relative = input.ego_speed_mps -
      input.object_path_speed_mps;
  return input.ego_to_object_back_m >= std::max(
      input.minimum_longitudinal_distance_m,
      relative * input.minimum_start_time_s);
}

bool behaviorDynamicObjectWillCutOut(
    const BehaviorDynamicCutOutInput &input) {
  if (input.object_path_speed_mps <
          input.minimum_object_speed_mps ||
      !input.time_since_inside_ego_path_s ||
      *input.time_since_inside_ego_path_s >
          input.maximum_time_since_inside_s)
    return false;
  return input.object_on_left ?
      input.object_lateral_speed_mps >
          input.minimum_lateral_speed_mps :
      input.object_lateral_speed_mps <
          -input.minimum_lateral_speed_mps;
}

std::optional<BehaviorSafetyPolygon> behaviorDynamicPredictedPolygon(
    const BehaviorDynamicPredictedPolygonInput &input) {
  if (!std::isfinite(input.object_length_m) ||
      !std::isfinite(input.object_width_m) ||
      !std::isfinite(input.end_time_s) ||
      !std::isfinite(input.minimum_confidence) ||
      !std::isfinite(input.expansion_margin_m) ||
      input.object_length_m <= 0.0 || input.object_width_m <= 0.0 ||
      input.end_time_s < 0.0 || input.expansion_margin_m < 0.0)
    return std::nullopt;
  BgMultiPoint footprint_points;
  BehaviorSafetyObject shape;
  shape.length_m = input.object_length_m;
  shape.width_m = input.object_width_m;
  for (const auto &path : input.paths) {
    if (!std::isfinite(path.confidence) ||
        !std::isfinite(path.time_step_s) ||
        path.time_step_s <= 0.0 ||
        path.confidence < input.minimum_confidence)
      continue;
    double time = 0.0;
    for (const auto &pose : path.poses) {
      if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
          !std::isfinite(pose.yaw_rad)) continue;
      const auto outline = behaviorSafetyWorldPolygon(
          pose, shape);
      for (const auto &[x, y] : outline)
        footprint_points.emplace_back(x, y);
      time += path.time_step_s;
      if (time > input.end_time_s + 1.0e-3) break;
    }
  }
  if (footprint_points.size() < 3) return std::nullopt;
  BgPolygon hull;
  bg::convex_hull(footprint_points, hull);
  BgMultiPolygon expanded;
  if (input.expansion_margin_m > 0.0) {
    namespace strategy = boost::geometry::strategy::buffer;
    bg::buffer(hull, expanded,
               strategy::distance_symmetric<double>(
                   input.expansion_margin_m),
               strategy::side_straight(), strategy::join_round(16),
               strategy::end_flat(), strategy::point_circle(16));
  } else {
    expanded.push_back(hull);
  }
  if (expanded.size() != 1) return std::nullopt;
  BgMultiPolygon result = expanded;
  if (input.ego_reserve_polygon.size() >= 3) {
    BgPolygon reserved;
    for (const auto &[x, y] : input.ego_reserve_polygon)
      reserved.outer().emplace_back(x, y);
    bg::correct(reserved);
    BgMultiPolygon clipped;
    bg::difference(expanded.front(), reserved, clipped);
    if (clipped.size() != 1) return std::nullopt;
    result = std::move(clipped);
  }
  BehaviorSafetyPolygon polygon;
  for (const auto &point : result.front().outer())
    polygon.emplace_back(bg::get<0>(point), bg::get<1>(point));
  return polygon.size() >= 4 ?
      std::optional<BehaviorSafetyPolygon>{std::move(polygon)} :
      std::nullopt;
}

std::optional<BehaviorSafetyPolygon> behaviorDynamicEgoReservePolygon(
    const Trajectory &reference, bool object_on_left,
    double vehicle_half_width_m,
    double maximum_lateral_avoidance_m) {
  if (reference.size() < 2 ||
      !std::isfinite(vehicle_half_width_m) ||
      !std::isfinite(maximum_lateral_avoidance_m) ||
      vehicle_half_width_m <= 0.0 ||
      maximum_lateral_avoidance_m < 0.0 ||
      maximum_lateral_avoidance_m > vehicle_half_width_m)
    return std::nullopt;
  bg::model::linestring<BgPoint> line;
  for (const auto &point : reference) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y))
      return std::nullopt;
    line.emplace_back(point.x, point.y);
  }
  const double near_width = vehicle_half_width_m -
      maximum_lateral_avoidance_m;
  namespace strategy = boost::geometry::strategy::buffer;
  BgMultiPolygon buffered;
  bg::buffer(line, buffered,
             object_on_left ?
                 strategy::distance_asymmetric<double>(
                     vehicle_half_width_m, near_width) :
                 strategy::distance_asymmetric<double>(
                     near_width, vehicle_half_width_m),
             strategy::side_straight(), strategy::join_round(16),
             strategy::end_flat(), strategy::point_circle(16));
  if (buffered.size() != 1) return std::nullopt;
  BehaviorSafetyPolygon polygon;
  for (const auto &point : buffered.front().outer())
    polygon.emplace_back(bg::get<0>(point), bg::get<1>(point));
  return polygon.size() >= 4 ?
      std::optional<BehaviorSafetyPolygon>{std::move(polygon)} :
      std::nullopt;
}

std::optional<BehaviorSafetyPolygon> behaviorDynamicObjectPathPolygon(
    const std::vector<BehaviorSafetyPose> &predicted_poses,
    double object_length_m, double object_width_m,
    double minimum_longitudinal_margin_m,
    double lateral_margin_m) {
  if (predicted_poses.size() < 2 ||
      !std::isfinite(object_length_m) ||
      !std::isfinite(object_width_m) ||
      !std::isfinite(minimum_longitudinal_margin_m) ||
      !std::isfinite(lateral_margin_m) ||
      object_length_m <= 0.0 || object_width_m <= 0.0 ||
      minimum_longitudinal_margin_m < 0.0 ||
      lateral_margin_m < 0.0)
    return std::nullopt;
  double path_length = 0.0;
  for (std::size_t index = 1; index < predicted_poses.size();
       ++index) {
    const auto &a = predicted_poses[index - 1];
    const auto &b = predicted_poses[index];
    if (!std::isfinite(a.x) || !std::isfinite(a.y) ||
        !std::isfinite(a.yaw_rad) ||
        !std::isfinite(b.x) || !std::isfinite(b.y) ||
        !std::isfinite(b.yaw_rad)) return std::nullopt;
    path_length += std::hypot(b.x - a.x, b.y - a.y);
  }
  std::vector<std::pair<double, double>> left, right;
  for (std::size_t index = 0; index < predicted_poses.size();
       ++index) {
    const auto &pose = predicted_poses[index];
    double longitudinal = 0.0;
    if (index == 0)
      longitudinal = -object_length_m * 0.5 -
          std::max(minimum_longitudinal_margin_m,
                   lateral_margin_m);
    else if (index + 1 == predicted_poses.size())
      longitudinal = object_length_m * 0.5 +
          std::max(minimum_longitudinal_margin_m - path_length,
                   lateral_margin_m);
    const double cosine = std::cos(pose.yaw_rad);
    const double sine = std::sin(pose.yaw_rad);
    const double half_width = object_width_m * 0.5 +
        lateral_margin_m;
    left.emplace_back(
        pose.x + longitudinal * cosine - half_width * sine,
        pose.y + longitudinal * sine + half_width * cosine);
    right.emplace_back(
        pose.x + longitudinal * cosine + half_width * sine,
        pose.y + longitudinal * sine - half_width * cosine);
  }
  BehaviorSafetyPolygon polygon = std::move(right);
  polygon.insert(polygon.end(), left.rbegin(), left.rend());
  polygon.push_back(polygon.front());
  return polygon;
}

std::optional<BehaviorDynamicLateralBounds>
behaviorDynamicUnregulatedLateralBounds(
    const BehaviorDynamicUnregulatedLateralInput &input) {
  const double values[]{input.object_min_lateral_m,
                        input.object_max_lateral_m,
                        input.object_lateral_speed_mps,
                        input.prediction_time_s,
                        input.obstacle_margin_m,
                        input.vehicle_half_width_m,
                        input.maximum_lateral_avoidance_m,
                        input.lowpass_gain};
  for (const double value : values)
    if (!std::isfinite(value)) return std::nullopt;
  if (input.object_min_lateral_m >
          input.object_max_lateral_m ||
      input.prediction_time_s < 0.0 ||
      input.obstacle_margin_m < 0.0 ||
      input.vehicle_half_width_m <= 0.0 ||
      input.maximum_lateral_avoidance_m < 0.0 ||
      input.lowpass_gain < 0.0 || input.lowpass_gain > 1.0)
    return std::nullopt;
  const double displacement = input.object_lateral_speed_mps *
      input.prediction_time_s;
  const double minimum = std::min(
      input.object_min_lateral_m,
      input.object_min_lateral_m + displacement);
  const double maximum = std::max(
      input.object_max_lateral_m,
      input.object_max_lateral_m + displacement);
  if (minimum * maximum < 0.0 ||
      maximum - minimum < 1.0e-3)
    return std::nullopt;
  double near = minimum - input.obstacle_margin_m;
  double far = maximum + input.obstacle_margin_m;
  if (std::abs(far) < std::abs(near))
    std::swap(near, far);
  const double near_limit = input.vehicle_half_width_m -
      input.maximum_lateral_avoidance_m;
  near = far > 0.0 ? std::max(near, near_limit) :
                      std::min(near, -near_limit);
  if (input.previous_near_m && input.use_lowpass &&
      std::isfinite(*input.previous_near_m))
    near = input.lowpass_gain * *input.previous_near_m +
        (1.0 - input.lowpass_gain) * near;
  return BehaviorDynamicLateralBounds{near, far};
}

std::optional<BehaviorDynamicLateralBounds>
behaviorDynamicRegulatedLateralBounds(
    const BehaviorDynamicRegulatedLateralInput &input) {
  const double values[]{input.object_min_lateral_m,
                        input.object_max_lateral_m,
                        input.object_path_speed_mps,
                        input.object_lateral_speed_mps,
                        input.minimum_front_object_speed_mps,
                        input.maximum_ego_path_cover_ratio,
                        input.obstacle_margin_m,
                        input.lateral_prediction_time_s,
                        input.vehicle_half_width_m,
                        input.maximum_lateral_avoidance_m,
                        input.lowpass_gain};
  for (const double value : values)
    if (!std::isfinite(value)) return std::nullopt;
  if (input.object_min_lateral_m >
          input.object_max_lateral_m ||
      input.obstacle_margin_m < 0.0 ||
      input.lateral_prediction_time_s < 0.0 ||
      input.vehicle_half_width_m <= 0.0 ||
      input.maximum_ego_path_cover_ratio < 0.0 ||
      input.lowpass_gain < 0.0 || input.lowpass_gain > 1.0)
    return std::nullopt;
  if (input.object_path_speed_mps >
      input.minimum_front_object_speed_mps) {
    const double covered = std::min(
        input.object_max_lateral_m,
        input.vehicle_half_width_m) -
        std::max(input.object_min_lateral_m,
                 -input.vehicle_half_width_m);
    if (2.0 * input.vehicle_half_width_m *
            input.maximum_ego_path_cover_ratio < covered)
      return std::nullopt;
  }
  const double sign = input.collision_on_left ? 1.0 : -1.0;
  const double expected_shift = std::max(
      0.0, -sign * input.object_lateral_speed_mps) *
      input.lateral_prediction_time_s;
  const double inner = input.collision_on_left ?
      input.object_min_lateral_m :
      input.object_max_lateral_m;
  const double outer = input.collision_on_left ?
      input.object_max_lateral_m :
      input.object_min_lateral_m;
  const double raw_near = inner - sign *
      (input.obstacle_margin_m + expected_shift);
  const double limit = input.vehicle_half_width_m -
      input.maximum_lateral_avoidance_m;
  double near = input.collision_on_left ?
      std::max(raw_near, limit) :
      std::min(raw_near, -limit);
  const double far = outer + sign * input.obstacle_margin_m;
  if (input.previous_near_m && input.use_lowpass &&
      std::isfinite(*input.previous_near_m))
    near = input.lowpass_gain * *input.previous_near_m +
        (1.0 - input.lowpass_gain) * near;
  return BehaviorDynamicLateralBounds{near, far};
}

std::optional<std::pair<double, double>>
behaviorDynamicLongitudinalBounds(
    const BehaviorDynamicLongitudinalBoundsInput &input) {
  const double values[]{input.object_min_longitudinal_m,
                        input.object_max_longitudinal_m,
                        input.object_path_speed_mps,
                        input.ego_speed_mps,
                        input.collision_time.start_s,
                        input.collision_time.end_s,
                        input.valid_predicted_path_length_m,
                        input.maximum_stopped_object_speed_mps,
                        input.overtaking_start_duration_s,
                        input.overtaking_end_duration_s,
                        input.oncoming_start_duration_s,
                        input.oncoming_end_duration_s};
  for (const double value : values)
    if (!std::isfinite(value)) return std::nullopt;
  if (input.object_min_longitudinal_m >
          input.object_max_longitudinal_m ||
      input.valid_predicted_path_length_m < 0.0 ||
      input.maximum_stopped_object_speed_mps < 0.0)
    return std::nullopt;
  const double object_speed = input.object_path_speed_mps;
  const double relative_speed = input.ego_speed_mps - object_speed;
  double before = 0.0;
  double after = 0.0;
  if (object_speed < input.maximum_stopped_object_speed_mps) {
    before = std::min(input.collision_time.start_s, 3.5) *
        std::abs(object_speed) +
        std::abs(relative_speed) *
            input.oncoming_start_duration_s;
    after = std::abs(relative_speed) *
        input.oncoming_end_duration_s;
  } else {
    const double decelerated = std::max(object_speed - 0.5, 0.0);
    const double object_moving_distance =
        (decelerated * decelerated - object_speed *
            object_speed) / -1.0;
    before = std::max(
        0.0, input.ego_speed_mps - object_moving_distance) +
        std::abs(relative_speed) *
            input.overtaking_start_duration_s;
    after = std::min(input.collision_time.end_s, 3.0) *
        object_speed + std::abs(relative_speed) *
            input.overtaking_end_duration_s;
  }
  const double valid = input.valid_predicted_path_length_m;
  if (object_speed < -input.maximum_stopped_object_speed_mps)
    return std::pair<double, double>{
        input.object_min_longitudinal_m +
            std::max(-before, -valid),
        input.object_max_longitudinal_m + after};
  if (object_speed > input.maximum_stopped_object_speed_mps)
    return std::pair<double, double>{
        input.object_min_longitudinal_m - before,
        input.object_max_longitudinal_m +
            std::min(after, valid)};
  return std::pair<double, double>{
      input.object_min_longitudinal_m - before,
      input.object_max_longitudinal_m + after};
}

std::optional<BehaviorDynamicPathOffsets> behaviorDynamicPathOffsets(
    const Trajectory &reference, double object_x, double object_y,
    const BehaviorSafetyPolygon &object_polygon) {
  if (reference.size() < 2 || object_polygon.empty() ||
      !std::isfinite(object_x) || !std::isfinite(object_y))
    return std::nullopt;
  std::vector<double> arcs{0.0};
  for (std::size_t index = 1; index < reference.size(); ++index) {
    const auto &a = reference[index - 1];
    const auto &b = reference[index];
    const double segment = std::hypot(b.x - a.x,
                                      b.y - a.y);
    if (!std::isfinite(segment) || segment <= 1.0e-8)
      return std::nullopt;
    arcs.push_back(arcs.back() + segment);
  }
  const auto nearest_segment = [&](double x, double y) {
    std::size_t nearest = 0;
    double minimum = std::numeric_limits<double>::infinity();
    double ratio_at_nearest = 0.0;
    for (std::size_t index = 0; index + 1 < reference.size();
         ++index) {
      const auto &a = reference[index];
      const auto &b = reference[index + 1];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double length_squared = dx * dx + dy * dy;
      const double ratio = std::clamp(
          ((x - a.x) * dx + (y - a.y) * dy) /
              length_squared, 0.0, 1.0);
      const double distance = std::hypot(
          x - a.x - ratio * dx,
          y - a.y - ratio * dy);
      if (distance < minimum) {
        nearest = index;
        minimum = distance;
        ratio_at_nearest = ratio;
      }
    }
    return std::pair<std::size_t, double>{nearest,
                                          ratio_at_nearest};
  };
  const auto [object_segment, object_ratio] =
      nearest_segment(object_x, object_y);
  BehaviorDynamicPathOffsets offsets;
  offsets.object_arc_m = arcs[object_segment] +
      object_ratio * (arcs[object_segment + 1] -
                      arcs[object_segment]);
  offsets.minimum_lateral_m =
      std::numeric_limits<double>::infinity();
  offsets.maximum_lateral_m =
      -std::numeric_limits<double>::infinity();
  offsets.minimum_longitudinal_m =
      std::numeric_limits<double>::infinity();
  offsets.maximum_longitudinal_m =
      -std::numeric_limits<double>::infinity();
  const auto &object_start = reference[object_segment];
  const auto &object_end = reference[object_segment + 1];
  const double object_yaw = std::atan2(
      object_end.y - object_start.y,
      object_end.x - object_start.x);
  for (const auto &[x, y] : object_polygon) {
    if (!std::isfinite(x) || !std::isfinite(y))
      return std::nullopt;
    const auto [segment, ratio] = nearest_segment(x, y);
    const auto &a = reference[segment];
    const auto &b = reference[segment + 1];
    const double yaw = std::atan2(b.y - a.y, b.x - a.x);
    const double lateral = -(x - a.x) * std::sin(yaw) +
        (y - a.y) * std::cos(yaw);
    const double longitudinal =
        (x - object_start.x) * std::cos(object_yaw) +
        (y - object_start.y) * std::sin(object_yaw) -
        object_ratio * (arcs[object_segment + 1] -
                        arcs[object_segment]);
    offsets.minimum_lateral_m = std::min(
        offsets.minimum_lateral_m, lateral);
    offsets.maximum_lateral_m = std::max(
        offsets.maximum_lateral_m, lateral);
    offsets.minimum_longitudinal_m = std::min(
        offsets.minimum_longitudinal_m, longitudinal);
    offsets.maximum_longitudinal_m = std::max(
        offsets.maximum_longitudinal_m, longitudinal);
    (void)ratio;
  }
  return offsets;
}

std::optional<BehaviorDynamicRegulatedPlan>
planBehaviorDynamicRegulatedObject(
    const BehaviorDynamicRegulatedPlanInput &input) {
  const auto object = behaviorDynamicPathOffsets(
      input.reference, input.object_x, input.object_y,
      input.object_polygon);
  const auto ego = behaviorDynamicPathOffsets(
      input.reference, input.ego_x, input.ego_y,
      {{input.ego_x, input.ego_y}});
  if (!object || !ego) return std::nullopt;
  BehaviorDynamicRegulatedLateralInput lateral;
  lateral.object_min_lateral_m = object->minimum_lateral_m;
  lateral.object_max_lateral_m = object->maximum_lateral_m;
  lateral.object_path_speed_mps =
      input.object_path_speed_mps;
  lateral.object_lateral_speed_mps =
      input.object_lateral_speed_mps;
  lateral.collision_on_left = input.collision_on_left;
  lateral.vehicle_half_width_m = input.vehicle_half_width_m;
  lateral.previous_near_m = input.previous_near_m;
  lateral.use_lowpass = input.use_lateral_lowpass;
  const auto lateral_bounds =
      behaviorDynamicRegulatedLateralBounds(lateral);
  if (!lateral_bounds) return std::nullopt;
  const auto timing = behaviorDynamicCollisionTiming(
      {input.ego_speed_mps, input.object_path_speed_mps,
       object->object_arc_m - ego->object_arc_m,
       object->minimum_longitudinal_m,
       object->maximum_longitudinal_m,
       input.ego_front_m, input.ego_rear_m});
  BehaviorDynamicLongitudinalBoundsInput longitudinal;
  longitudinal.object_min_longitudinal_m =
      object->minimum_longitudinal_m;
  longitudinal.object_max_longitudinal_m =
      object->maximum_longitudinal_m;
  longitudinal.object_path_speed_mps =
      input.object_path_speed_mps;
  longitudinal.ego_speed_mps = input.ego_speed_mps;
  longitudinal.collision_time = timing;
  longitudinal.valid_predicted_path_length_m =
      input.valid_predicted_path_length_m;
  const auto longitudinal_bounds =
      behaviorDynamicLongitudinalBounds(longitudinal);
  if (!longitudinal_bounds) return std::nullopt;
  BehaviorDynamicAvoidanceCorridorInput corridor;
  corridor.reference = input.reference;
  corridor.object_x = input.object_x;
  corridor.object_y = input.object_y;
  corridor.longitudinal_start_m =
      longitudinal_bounds->first;
  corridor.longitudinal_end_m =
      longitudinal_bounds->second;
  corridor.inner_lateral_m = lateral_bounds->near_m;
  corridor.outer_lateral_m = lateral_bounds->far_m;
  auto polygon = behaviorDynamicAvoidanceCorridor(corridor);
  if (!polygon) return std::nullopt;
  const auto feasibility_origin = input.feasibility_origin.value_or(
      BehaviorSafetyPose{0.0, input.ego_x, input.ego_y,
                         input.ego_yaw_rad, input.ego_speed_mps});
  const double feasible_values[]{
      feasibility_origin.x, feasibility_origin.y,
      feasibility_origin.yaw_rad,
      feasibility_origin.velocity_mps,
      input.vehicle_half_width_m,
      input.maximum_ego_lateral_acceleration_mps2,
      input.maximum_ego_lateral_jerk_mps3,
      input.ego_shift_delay_s};
  for (const double value : feasible_values)
    if (!std::isfinite(value)) return std::nullopt;
  if (input.vehicle_half_width_m <= 0.0 ||
      input.maximum_ego_lateral_acceleration_mps2 < 0.0 ||
      input.maximum_ego_lateral_jerk_mps3 < 0.0 ||
      input.ego_shift_delay_s < 0.0) return std::nullopt;
  std::vector<std::pair<double, double>> feasible;
  feasible.reserve(10);
  const double tangent_x = std::cos(
      feasibility_origin.yaw_rad);
  const double tangent_y = std::sin(
      feasibility_origin.yaw_rad);
  for (int second = 0; second < 10; ++second) {
    const double elapsed = std::max(
        static_cast<double>(second) - input.ego_shift_delay_s,
        0.0);
    const double shift =
        0.5 * input.maximum_ego_lateral_acceleration_mps2 *
            elapsed * elapsed +
        input.maximum_ego_lateral_jerk_mps3 *
            elapsed * elapsed * elapsed / 6.0 -
        input.vehicle_half_width_m;
    const double lateral_m = input.collision_on_left ?
        -shift : shift;
    const double forward_m = second *
        feasibility_origin.velocity_mps;
    feasible.emplace_back(
        feasibility_origin.x + forward_m * tangent_x -
            lateral_m * tangent_y,
        feasibility_origin.y + forward_m * tangent_y +
            lateral_m * tangent_x);
  }
  const auto cross = [](double ax, double ay,
                        double bx, double by) {
    return ax * by - ay * bx;
  };
  const std::size_t inner_count = (polygon->size() - 1) / 2;
  bool has_intersection = false;
  for (std::size_t i = 1; i < inner_count && !has_intersection; ++i) {
    const auto &a = polygon->at(i - 1);
    const auto &b = polygon->at(i);
    const double rx = b.first - a.first;
    const double ry = b.second - a.second;
    for (std::size_t j = 1; j < feasible.size(); ++j) {
      const auto &c = feasible[j - 1];
      const auto &d = feasible[j];
      const double sx = d.first - c.first;
      const double sy = d.second - c.second;
      const double denominator = cross(rx, ry, sx, sy);
      if (std::abs(denominator) < 1.0e-9) continue;
      const double from_x = c.first - a.first;
      const double from_y = c.second - a.second;
      const double t = cross(from_x, from_y, sx, sy) /
          denominator;
      const double u = cross(from_x, from_y, rx, ry) /
          denominator;
      if (t < -1.0e-8 || t > 1.0 + 1.0e-8 ||
          u < -1.0e-8 || u > 1.0 + 1.0e-8)
        continue;
      const double x = a.first + t * rx;
      const double y = a.second + t * ry;
      const auto crossing = behaviorDynamicPathOffsets(
          input.reference, x, y, {{x, y}});
      if (!crossing) return std::nullopt;
      corridor.longitudinal_start_m = std::max(
          corridor.longitudinal_start_m,
          crossing->object_arc_m - object->object_arc_m);
      has_intersection = true;
      break;
    }
  }
  if (!has_intersection) {
    const auto &point = polygon->front();
    double nearest = std::numeric_limits<double>::infinity();
    double lateral_offset = 0.0;
    for (std::size_t i = 1; i < feasible.size(); ++i) {
      const auto &a = feasible[i - 1];
      const auto &b = feasible[i];
      const double dx = b.first - a.first;
      const double dy = b.second - a.second;
      const double length_squared = dx * dx + dy * dy;
      if (length_squared < 1.0e-12) continue;
      const double ratio = std::clamp(
          ((point.first - a.first) * dx +
           (point.second - a.second) * dy) /
              length_squared, 0.0, 1.0);
      const double error = std::hypot(
          point.first - a.first - ratio * dx,
          point.second - a.second - ratio * dy);
      if (error < nearest) {
        nearest = error;
        lateral_offset = cross(
            dx, dy, point.first - a.first,
            point.second - a.second) /
            std::sqrt(length_squared);
      }
    }
    if (std::isfinite(nearest) &&
        (input.collision_on_left ? lateral_offset < 0.0 :
                                   lateral_offset > 0.0))
      return std::nullopt;
  }
  if (has_intersection) {
    polygon = behaviorDynamicAvoidanceCorridor(corridor);
    if (!polygon) return std::nullopt;
  }
  return BehaviorDynamicRegulatedPlan{
      *polygon, timing, *lateral_bounds,
      *longitudinal_bounds};
}

std::optional<BehaviorSafetyPolygon> behaviorDynamicAvoidanceCorridor(
    const BehaviorDynamicAvoidanceCorridorInput &input) {
  if (input.reference.size() < 2 ||
      !std::isfinite(input.object_x) ||
      !std::isfinite(input.object_y) ||
      !std::isfinite(input.longitudinal_start_m) ||
      !std::isfinite(input.longitudinal_end_m) ||
      !std::isfinite(input.inner_lateral_m) ||
      !std::isfinite(input.outer_lateral_m) ||
      input.longitudinal_start_m >= input.longitudinal_end_m ||
      input.inner_lateral_m == input.outer_lateral_m)
    return std::nullopt;
  std::vector<double> arcs{0.0};
  for (std::size_t index = 1; index < input.reference.size(); ++index) {
    const auto &a = input.reference[index - 1];
    const auto &b = input.reference[index];
    if (!std::isfinite(a.x) || !std::isfinite(a.y) ||
        !std::isfinite(b.x) || !std::isfinite(b.y))
      return std::nullopt;
    const double segment = std::hypot(b.x - a.x, b.y - a.y);
    if (segment <= 1.0e-8) return std::nullopt;
    arcs.push_back(arcs.back() + segment);
  }
  double object_s = 0.0;
  double nearest_squared = std::numeric_limits<double>::infinity();
  for (std::size_t index = 1; index < input.reference.size(); ++index) {
    const auto &a = input.reference[index - 1];
    const auto &b = input.reference[index];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_squared = dx * dx + dy * dy;
    const double ratio = std::clamp(
        ((input.object_x - a.x) * dx +
         (input.object_y - a.y) * dy) / length_squared,
        0.0, 1.0);
    const double distance_squared = std::pow(
        input.object_x - a.x - ratio * dx, 2) + std::pow(
        input.object_y - a.y - ratio * dy, 2);
    if (distance_squared < nearest_squared) {
      nearest_squared = distance_squared;
      object_s = arcs[index - 1] + ratio *
          (arcs[index] - arcs[index - 1]);
    }
  }
  const double start = std::max(
      0.0, object_s + input.longitudinal_start_m);
  const double end = std::min(
      arcs.back(), object_s + input.longitudinal_end_m);
  if (end - start <= 1.0e-8) return std::nullopt;
  std::vector<double> samples{start};
  for (const double arc : arcs)
    if (start < arc && arc < end) samples.push_back(arc);
  samples.push_back(end);
  const auto offset_point = [&](double arc, double lateral) {
    const auto upper = std::upper_bound(arcs.begin(), arcs.end(), arc);
    const std::size_t index = std::clamp<std::size_t>(
        upper == arcs.begin() ? 0 :
            static_cast<std::size_t>(upper - arcs.begin() - 1),
        0, arcs.size() - 2);
    const auto &a = input.reference[index];
    const auto &b = input.reference[index + 1];
    const double ratio = (arc - arcs[index]) /
        (arcs[index + 1] - arcs[index]);
    const double yaw = std::atan2(b.y - a.y, b.x - a.x);
    return std::pair<double, double>{
        a.x + ratio * (b.x - a.x) - lateral * std::sin(yaw),
        a.y + ratio * (b.y - a.y) + lateral * std::cos(yaw)};
  };
  BehaviorSafetyPolygon polygon;
  polygon.reserve(2 * samples.size() + 1);
  for (const double arc : samples)
    polygon.push_back(offset_point(arc, input.inner_lateral_m));
  for (auto it = samples.rbegin(); it != samples.rend(); ++it)
    polygon.push_back(offset_point(*it, input.outer_lateral_m));
  polygon.push_back(polygon.front());
  return polygon;
}

BehaviorDynamicAvoidanceTracker::BehaviorDynamicAvoidanceTracker(
    BehaviorDynamicAvoidanceParameters parameters)
    : parameters_(std::move(parameters)) {}

std::vector<BehaviorDynamicAvoidanceObservation>
BehaviorDynamicAvoidanceTracker::advance(
    const std::vector<BehaviorDynamicAvoidanceObservation> &observed) {
  std::unordered_set<std::string> current;
  for (const auto &object : observed) {
    if (object.id.empty() || !std::isfinite(object.speed_mps) ||
        !std::isfinite(object.x) || !std::isfinite(object.y) ||
        object.speed_mps < parameters_.minimum_speed_mps ||
        object.speed_mps > parameters_.maximum_speed_mps ||
        behaviorDynamicObjectType(
            object.classification, parameters_) ==
            BehaviorDynamicObjectType::OutOfScope)
      continue;
    if (!current.insert(object.id).second) continue;
    objects_[object.id] = object;
    auto &count = counters_[object.id];
    count = std::min(parameters_.successive_entry_count,
                     std::max(1, count + 1));
  }
  for (auto &[id, count] : counters_)
    if (current.count(id) == 0) count = std::max(0, count - 1);
  for (const auto &[id, count] : counters_) {
    if (count >= parameters_.successive_entry_count &&
        std::find(valid_ids_.begin(), valid_ids_.end(), id) ==
            valid_ids_.end())
      valid_ids_.push_back(id);
  }
  valid_ids_.erase(std::remove_if(
      valid_ids_.begin(), valid_ids_.end(),
      [&](const auto &id) {
        return counters_.count(id) == 0 ||
            counters_.at(id) < parameters_.successive_exit_count;
      }), valid_ids_.end());
  for (auto it = counters_.begin(); it != counters_.end();) {
    if (it->second < parameters_.successive_exit_count) {
      objects_.erase(it->first);
      it = counters_.erase(it);
    } else {
      ++it;
    }
  }
  std::vector<BehaviorDynamicAvoidanceObservation> valid;
  for (const auto &id : valid_ids_)
    if (const auto it = objects_.find(id); it != objects_.end())
      valid.push_back(it->second);
  return valid;
}

void BehaviorDynamicAvoidanceTracker::reset() {
  counters_.clear();
  objects_.clear();
  valid_ids_.clear();
}

}  // planning 네임스페이스
