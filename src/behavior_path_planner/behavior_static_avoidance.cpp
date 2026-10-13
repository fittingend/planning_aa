// SPDX-License-Identifier: Apache-2.0
// 원본 정적 장애물 회피의 측방 여유 분기를 독립 계산으로 옮깁니다.

#include "planning/behavior_static_avoidance.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

namespace planning {
namespace {

std::optional<std::pair<double, double>> segmentIntersection(
    std::pair<double, double> first_start,
    std::pair<double, double> first_end,
    std::pair<double, double> second_start,
    std::pair<double, double> second_end) {
  const double first_dx = first_end.first - first_start.first;
  const double first_dy = first_end.second - first_start.second;
  const double second_dx = second_end.first - second_start.first;
  const double second_dy = second_end.second - second_start.second;
  const double denominator =
      first_dx * second_dy - first_dy * second_dx;
  if (std::abs(denominator) < 1.0e-12) return std::nullopt;
  const double origin_dx = second_start.first - first_start.first;
  const double origin_dy = second_start.second - first_start.second;
  const double first_fraction =
      (origin_dx * second_dy - origin_dy * second_dx) / denominator;
  const double second_fraction =
      (origin_dx * first_dy - origin_dy * first_dx) / denominator;
  if (first_fraction < 0.0 || first_fraction > 1.0 ||
      second_fraction < 0.0 || second_fraction > 1.0) {
    return std::nullopt;
  }
  return std::pair{
      first_start.first + first_fraction * first_dx,
      first_start.second + first_fraction * first_dy};
}

double projectedArc(const Trajectory &path,
                    const std::vector<double> &arcs,
                    double x, double y) {
  const auto nearest = std::min_element(
      path.begin(), path.end(), [x, y](const auto &first,
                                       const auto &second) {
        return std::hypot(first.x - x, first.y - y) <
               std::hypot(second.x - x, second.y - y);
      });
  const std::size_t nearest_index =
      static_cast<std::size_t>(nearest - path.begin());
  std::size_t segment = 0;
  if (nearest_index + 1 == path.size()) {
    segment = path.size() - 2;
  } else if (nearest_index != 0) {
    const double dx = path[nearest_index + 1].x - nearest->x;
    const double dy = path[nearest_index + 1].y - nearest->y;
    const double length = std::hypot(dx, dy);
    const double along = length > 1.0e-12 ?
        ((x - nearest->x) * dx + (y - nearest->y) * dy) /
            length : 0.0;
    segment = along <= 0.0 ? nearest_index - 1 : nearest_index;
  }
  const double dx = path[segment + 1].x - path[segment].x;
  const double dy = path[segment + 1].y - path[segment].y;
  const double length = std::hypot(dx, dy);
  if (length <= 1.0e-12) {
    throw std::invalid_argument("정적 회피 기준 경로에 길이 0인 선분이 있습니다");
  }
  return arcs[segment] +
      ((x - path[segment].x) * dx +
       (y - path[segment].y) * dy) / length;
}

}  // 익명 네임스페이스

StaticAvoidanceBoundaryDistance staticAvoidanceRoadShoulderDistance(
    const std::vector<StaticAvoidanceOverhangPoint> &overhang_points,
    double envelope_area_m2, double object_length_m,
    const std::vector<std::pair<double, double>> &opposite_boundary,
    double centerline_yaw_rad, bool object_on_right) {
  if (!std::isfinite(envelope_area_m2) || envelope_area_m2 < 0.0 ||
      !std::isfinite(object_length_m) || object_length_m < 0.0 ||
      !std::isfinite(centerline_yaw_rad)) {
    throw std::invalid_argument("정적 회피 경계 거리 입력이 유효하지 않습니다");
  }
  for (const auto &point : overhang_points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw std::invalid_argument("정적 회피 돌출점이 유효하지 않습니다");
    }
  }
  for (const auto &[x, y] : opposite_boundary) {
    if (!std::isfinite(x) || !std::isfinite(y)) {
      throw std::invalid_argument("정적 회피 경계 좌표가 유효하지 않습니다");
    }
  }
  StaticAvoidanceBoundaryDistance result;
  if (opposite_boundary.size() < 2 || overhang_points.empty()) return result;
  const double envelope_width = envelope_area_m2 /
      std::max(object_length_m, 1.0e-3);
  const double lateral_x = -std::sin(centerline_yaw_rad);
  const double lateral_y = std::cos(centerline_yaw_rad);
  bool found = false;
  for (const auto &point : overhang_points) {
    const std::pair origin{point.x, point.y};
    const auto cast = [&](double lateral) {
      return std::pair{
          point.x + lateral * lateral_x,
          point.y + lateral * lateral_y};
    };
    for (std::size_t index = 1; index < opposite_boundary.size(); ++index) {
      auto intersection = segmentIntersection(
          origin, cast(object_on_right ? 100.0 : -100.0),
          opposite_boundary[index - 1], opposite_boundary[index]);
      double distance = 0.0;
      if (intersection) {
        distance = std::hypot(
            intersection->first - point.x,
            intersection->second - point.y);
      } else {
        intersection = segmentIntersection(
            origin, cast((object_on_right ? -0.5 : 0.5) *
                         envelope_width),
            opposite_boundary[index - 1], opposite_boundary[index]);
        if (!intersection) continue;
        distance = -std::hypot(
            intersection->first - point.x,
            intersection->second - point.y);
      }
      if (!found || distance < result.distance_m) {
        found = true;
        result.distance_m = distance;
        result.envelope_point = origin;
        result.boundary_point = intersection;
      }
      break;
    }
  }
  return result;
}

std::optional<StaticAvoidanceLongitudinalSpan>
staticAvoidanceEnvelopeLongitudinalSpan(
    const Trajectory &reference,
    double ego_x, double ego_y,
    const std::vector<std::pair<double, double>> &envelope) {
  if (reference.size() < 2 || envelope.empty()) return std::nullopt;
  if (!std::isfinite(ego_x) || !std::isfinite(ego_y)) {
    throw std::invalid_argument("정적 회피 차량 위치가 유효하지 않습니다");
  }
  std::vector<double> arcs(reference.size(), 0.0);
  for (std::size_t index = 1; index < reference.size(); ++index) {
    arcs[index] = arcs[index - 1] +
        std::hypot(reference[index].x - reference[index - 1].x,
                   reference[index].y - reference[index - 1].y);
  }
  const double ego_arc = projectedArc(reference, arcs, ego_x, ego_y);
  double earliest = std::numeric_limits<double>::infinity();
  double latest = -earliest;
  for (const auto &[x, y] : envelope) {
    if (!std::isfinite(x) || !std::isfinite(y)) {
      throw std::invalid_argument("정적 회피 객체 외피 좌표가 유효하지 않습니다");
    }
    const double relative = projectedArc(reference, arcs, x, y) - ego_arc;
    earliest = std::min(earliest, relative);
    latest = std::max(latest, relative);
  }
  return StaticAvoidanceLongitudinalSpan{earliest, latest - earliest};
}

std::vector<std::pair<double, double>> staticAvoidanceEnvelopePolygon(
    const std::vector<std::pair<double, double>> &object_polygon,
    const TrajectoryPoint &closest_pose,
    double buffer_m) {
  if (object_polygon.empty() || !std::isfinite(buffer_m) ||
      buffer_m < 0.0 || !std::isfinite(closest_pose.x) ||
      !std::isfinite(closest_pose.y)) {
    throw std::invalid_argument("정적 회피 외피 매개변수가 유효하지 않습니다");
  }
  const double heading = std::atan2(
      2.0 * (closest_pose.orientation_w *
                 closest_pose.orientation_z +
             closest_pose.orientation_x *
                 closest_pose.orientation_y),
      1.0 - 2.0 * (closest_pose.orientation_y *
                      closest_pose.orientation_y +
                   closest_pose.orientation_z *
                      closest_pose.orientation_z));
  const double cosine = std::cos(heading);
  const double sine = std::sin(heading);
  double minimum_x = std::numeric_limits<double>::infinity();
  double maximum_x = -minimum_x;
  double minimum_y = minimum_x;
  double maximum_y = -minimum_x;
  for (const auto &[x, y] : object_polygon) {
    if (!std::isfinite(x) || !std::isfinite(y)) {
      throw std::invalid_argument("정적 회피 객체 다각형이 유효하지 않습니다");
    }
    const double dx = x - closest_pose.x;
    const double dy = y - closest_pose.y;
    const double local_x = dx * cosine + dy * sine;
    const double local_y = -dx * sine + dy * cosine;
    minimum_x = std::min(minimum_x, local_x);
    maximum_x = std::max(maximum_x, local_x);
    minimum_y = std::min(minimum_y, local_y);
    maximum_y = std::max(maximum_y, local_y);
  }
  minimum_x -= buffer_m;
  maximum_x += buffer_m;
  minimum_y -= buffer_m;
  maximum_y += buffer_m;
  std::vector<std::pair<double, double>> envelope;
  for (const auto &[along, across] : {
           std::pair{minimum_x, minimum_y},
           std::pair{maximum_x, minimum_y},
           std::pair{maximum_x, maximum_y},
           std::pair{minimum_x, maximum_y},
           std::pair{minimum_x, minimum_y}}) {
    envelope.emplace_back(
        closest_pose.x + along * cosine - across * sine,
        closest_pose.y + along * sine + across * cosine);
  }
  return envelope;
}

std::vector<StaticAvoidanceOverhangPoint>
staticAvoidanceEnvelopeOverhangDistances(
    const Trajectory &reference,
    const std::vector<std::pair<double, double>> &envelope,
    bool object_on_right) {
  std::vector<StaticAvoidanceOverhangPoint> result;
  if (reference.empty()) return result;
  result.reserve(envelope.size());
  for (const auto &[x, y] : envelope) {
    if (!std::isfinite(x) || !std::isfinite(y)) {
      throw std::invalid_argument("정적 회피 객체 외피 좌표가 유효하지 않습니다");
    }
    const auto nearest = std::min_element(
        reference.begin(), reference.end(),
        [x, y](const auto &first, const auto &second) {
          return std::hypot(first.x - x, first.y - y) <
                 std::hypot(second.x - x, second.y - y);
        });
    const double heading = std::atan2(
        2.0 * (nearest->orientation_w * nearest->orientation_z +
               nearest->orientation_x * nearest->orientation_y),
        1.0 - 2.0 * (nearest->orientation_y * nearest->orientation_y +
                     nearest->orientation_z * nearest->orientation_z));
    const double lateral =
        -(x - nearest->x) * std::sin(heading) +
        (y - nearest->y) * std::cos(heading);
    result.push_back({lateral, x, y});
  }
  std::sort(result.begin(), result.end(),
            [object_on_right](const auto &first, const auto &second) {
              return object_on_right ?
                  first.lateral_m > second.lateral_m :
                  first.lateral_m < second.lateral_m;
            });
  return result;
}

std::optional<double> staticAvoidanceLateralMargin(
    const StaticAvoidanceMarginInput &input) {
  const double values[] = {
      input.vehicle_width_m, input.road_shoulder_distance_m,
      input.distance_factor, input.lateral_hard_margin_m,
      input.parked_lateral_hard_margin_m, input.lateral_soft_margin_m,
      input.soft_drivable_bound_margin_m,
      input.hard_drivable_bound_margin_m};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("정적 회피 여유 거리 입력이 유효하지 않습니다");
    }
  }
  if (input.vehicle_width_m <= 0.0 || input.distance_factor < 0.0 ||
      input.lateral_hard_margin_m <= -input.vehicle_width_m * 0.5 ||
      input.parked_lateral_hard_margin_m <=
          -input.vehicle_width_m * 0.5 ||
      input.lateral_soft_margin_m < 0.0) {
    throw std::invalid_argument("정적 회피 여유 거리 범위가 유효하지 않습니다");
  }
  const double hard_margin = input.parked
      ? input.parked_lateral_hard_margin_m : input.lateral_hard_margin_m;
  const double half_width = input.vehicle_width_m * 0.5;
  const double maximum_margin = hard_margin * input.distance_factor +
      input.lateral_soft_margin_m + half_width;
  const double minimum_margin = hard_margin + half_width;
  const double soft_limit = input.road_shoulder_distance_m -
      input.soft_drivable_bound_margin_m - half_width;
  const double hard_limit = input.road_shoulder_distance_m -
      input.hard_drivable_bound_margin_m - half_width;
  if (hard_limit < minimum_margin) return std::nullopt;
  if (soft_limit < minimum_margin) return minimum_margin;
  return std::min(soft_limit, maximum_margin);
}

std::optional<StaticAvoidanceObjectGeometry>
staticAvoidanceObjectGeometry(
    const Trajectory &reference, double ego_x, double ego_y,
    const std::vector<std::pair<double, double>> &object_polygon,
    const TrajectoryPoint &closest_reference_pose,
    double envelope_buffer_m, bool object_on_right,
    const StaticAvoidanceMarginInput &margin_input,
    const std::optional<StaticAvoidanceBoundaryInput> &boundary) {
  auto envelope = staticAvoidanceEnvelopePolygon(
      object_polygon, closest_reference_pose, envelope_buffer_m);
  const auto longitudinal = staticAvoidanceEnvelopeLongitudinalSpan(
      reference, ego_x, ego_y, envelope);
  const auto overhang = staticAvoidanceEnvelopeOverhangDistances(
      reference, envelope, object_on_right);
  if (!longitudinal || overhang.empty()) return std::nullopt;
  auto effective_margin = margin_input;
  std::optional<StaticAvoidanceBoundaryDistance> boundary_distance;
  if (boundary) {
    double twice_area = 0.0;
    for (std::size_t index = 1; index < envelope.size(); ++index) {
      twice_area += envelope[index - 1].first * envelope[index].second -
                    envelope[index].first * envelope[index - 1].second;
    }
    boundary_distance = staticAvoidanceRoadShoulderDistance(
        overhang, std::abs(twice_area) * 0.5,
        longitudinal->length_m, boundary->opposite_boundary,
        boundary->centerline_yaw_rad, object_on_right);
    effective_margin.road_shoulder_distance_m =
        boundary_distance->distance_m;
  }
  const auto margin = staticAvoidanceLateralMargin(effective_margin);
  if (!margin) return std::nullopt;
  const double shift = staticAvoidanceShiftLength(
      object_on_right, overhang.front().lateral_m, *margin);
  return StaticAvoidanceObjectGeometry{
      std::move(envelope), *longitudinal, overhang, overhang.front(),
      boundary_distance, *margin, shift};
}

double staticAvoidanceShiftLength(
    bool object_on_right, double overhang_m, double lateral_margin_m) {
  if (!std::isfinite(overhang_m) || !std::isfinite(lateral_margin_m) ||
      lateral_margin_m < 0.0) {
    throw std::invalid_argument("정적 회피 이동량 입력이 유효하지 않습니다");
  }
  const double shift = object_on_right
      ? overhang_m + lateral_margin_m
      : overhang_m - lateral_margin_m;
  return std::abs(shift) > 1.0e-3 ? shift : 0.0;
}

bool staticAvoidanceShiftNecessary(
    bool object_on_right, double shift_length_m) {
  if (!std::isfinite(shift_length_m)) {
    throw std::invalid_argument("정적 회피 이동 방향 입력이 유효하지 않습니다");
  }
  return object_on_right ? shift_length_m >= 0.0 : shift_length_m <= 0.0;
}

StaticAvoidanceFeasibilityResult staticAvoidanceFeasibleShift(
    const StaticAvoidanceFeasibilityInput &input) {
  const double values[] = {
      input.desired_shift_m, input.current_ego_shift_m,
      input.object_longitudinal_m, input.front_constant_distance_m,
      input.prepare_distance_m, input.nominal_avoidance_distance_m,
      input.avoidance_speed_mps, input.maximum_lateral_jerk_mps3,
      input.minimum_distance_jerk_mps3,
      input.minimum_distance_speed_mps,
      input.lateral_execution_threshold_m, input.vehicle_width_m,
      input.lateral_hard_margin_m, input.overhang_m};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("정적 회피 가능 이동량 입력이 유효하지 않습니다");
    }
  }
  if (input.front_constant_distance_m < 0.0 ||
      input.prepare_distance_m < 0.0 ||
      input.nominal_avoidance_distance_m < 0.0 ||
      input.maximum_lateral_jerk_mps3 <= 0.0 ||
      input.minimum_distance_jerk_mps3 <= 0.0 ||
      input.minimum_distance_speed_mps < 0.0 ||
      input.lateral_execution_threshold_m < 0.0 ||
      input.vehicle_width_m <= 0.0 ||
      input.lateral_hard_margin_m < 0.0) {
    throw std::invalid_argument("정적 회피 가능 이동량 범위가 유효하지 않습니다");
  }
  const auto minimum_distance = [&](double lateral) {
    return 4.0 * std::cbrt(
        0.5 * std::abs(lateral) /
        input.minimum_distance_jerk_mps3) *
        input.minimum_distance_speed_mps;
  };
  const double relative_shift =
      input.desired_shift_m - input.current_ego_shift_m;
  const bool nominal_distance_available = input.object_longitudinal_m >
      input.front_constant_distance_m + input.prepare_distance_m +
          input.nominal_avoidance_distance_m;
  const double remaining_distance = input.object_longitudinal_m -
      input.front_constant_distance_m - input.prepare_distance_m;
  const double avoidance_distance = nominal_distance_available ?
      input.nominal_avoidance_distance_m : remaining_distance;
  const auto feasible = [&](double shift) {
    return StaticAvoidanceFeasibilityResult{
        StaticAvoidanceFeasibilityReason::Feasible,
        shift, avoidance_distance};
  };
  const auto rejected = [&](StaticAvoidanceFeasibilityReason reason) {
    return StaticAvoidanceFeasibilityResult{
        reason, input.desired_shift_m, avoidance_distance};
  };
  if (nominal_distance_available || !input.best_effort_lateral_margin) {
    return feasible(input.desired_shift_m);
  }
  const bool enough_positive_shift =
      relative_shift < -1.0e-3 && input.desired_shift_m > 1.0e-3;
  const bool enough_negative_shift =
      relative_shift > 1.0e-3 && input.desired_shift_m < -1.0e-3;
  if ((input.object_on_right && enough_positive_shift) ||
      (!input.object_on_right && enough_negative_shift) ||
      (input.stoppable && !input.shorten_margin_immediately) ||
      input.already_approved) {
    return feasible(input.desired_shift_m);
  }
  if (avoidance_distance < 1.0e-3) {
    return rejected(
        StaticAvoidanceFeasibilityReason::InsufficientLongitudinalDistance);
  }
  const double velocity = std::abs(input.avoidance_speed_mps);
  const double required_jerk =
      0.5 * std::abs(relative_shift) *
      std::pow(4.0 * velocity /
                   std::max(std::abs(avoidance_distance), 1.0e-3), 3);
  if (required_jerk < input.maximum_lateral_jerk_mps3) {
    return feasible(input.desired_shift_m);
  }
  if (!input.best_effort_deceleration) {
    return rejected(avoidance_distance <
                            minimum_distance(relative_shift) + 1.0e-3 ?
        StaticAvoidanceFeasibilityReason::InsufficientLongitudinalDistance :
        StaticAvoidanceFeasibilityReason::NeedsDeceleration);
  }
  if (velocity < 1.0e-8) {
    return rejected(StaticAvoidanceFeasibilityReason::NeedsDeceleration);
  }
  const double feasible_relative_shift =
      2.0 * std::pow(
          std::abs(avoidance_distance) / (4.0 * velocity), 3) *
      input.maximum_lateral_jerk_mps3;
  if (feasible_relative_shift < input.lateral_execution_threshold_m) {
    return rejected(StaticAvoidanceFeasibilityReason::BelowExecutionThreshold);
  }
  const double feasible_shift = input.desired_shift_m > 0.0 ?
      feasible_relative_shift + input.current_ego_shift_m :
      -feasible_relative_shift + input.current_ego_shift_m;
  if (avoidance_distance < minimum_distance(feasible_shift) + 1.0e-3) {
    return rejected(
        StaticAvoidanceFeasibilityReason::InsufficientLongitudinalDistance);
  }
  const double buffer = input.desired_shift_m > 0.0 ? 1.0e-3 : -1.0e-3;
  if (std::abs(feasible_shift - input.overhang_m) - buffer <
      0.5 * input.vehicle_width_m + input.lateral_hard_margin_m) {
    return rejected(StaticAvoidanceFeasibilityReason::NeedsDeceleration);
  }
  return feasible(feasible_shift - buffer);
}

std::optional<StaticAvoidanceOutline> staticAvoidanceShiftOutline(
    const StaticAvoidanceOutlineInput &input) {
  const double values[] = {
      input.object_longitudinal_m, input.front_constant_distance_m,
      input.rear_constant_distance_m, input.avoidance_distance_m,
      input.minimum_avoidance_distance_m,
      input.maximum_return_distance_m, input.to_start_point_m,
      input.to_return_point_m, input.current_start_shift_m,
      input.target_shift_m};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("정적 회피 이동선 입력이 유효하지 않습니다");
    }
  }
  if (input.object_longitudinal_m <= 0.0 ||
      input.front_constant_distance_m < 0.0 ||
      input.rear_constant_distance_m < 0.0 ||
      input.avoidance_distance_m <= 0.0 ||
      input.minimum_avoidance_distance_m < 0.0 ||
      input.maximum_return_distance_m <= 0.0) {
    throw std::invalid_argument("정적 회피 이동선 범위가 유효하지 않습니다");
  }
  const double avoid_end = input.object_longitudinal_m -
      input.front_constant_distance_m;
  const double nearest = std::max(
      avoid_end - input.avoidance_distance_m, 1.0e-3);
  double avoid_start = nearest;
  if (input.to_start_point_m <= avoid_end) {
    const double furthest = std::max(
        avoid_end - input.minimum_avoidance_distance_m, 1.0e-3);
    if (nearest > furthest) return std::nullopt;
    avoid_start = std::clamp(input.to_start_point_m, nearest, furthest);
  }
  if (avoid_start <= 0.0 || avoid_start >= avoid_end) return std::nullopt;
  StaticAvoidanceOutline result;
  result.avoid = {avoid_start, avoid_end,
                  input.current_start_shift_m, input.target_shift_m};
  if (input.skip_return) return result;
  const double return_start = input.object_longitudinal_m +
      input.rear_constant_distance_m;
  const double return_end = input.to_return_point_m > return_start
      ? std::clamp(input.to_return_point_m, return_start,
                   return_start + input.maximum_return_distance_m)
      : return_start + input.maximum_return_distance_m;
  if (return_start <= 0.0 || return_start >= return_end) {
    return std::nullopt;
  }
  result.return_to_center = StaticAvoidanceShiftArc{
      return_start, return_end, input.target_shift_m, 0.0};
  return result;
}

StaticAvoidanceMergedShiftProfile mergeStaticAvoidanceShiftArcs(
    const std::vector<double> &arcs_from_ego_m,
    std::size_t ego_index, double current_shift_m,
    const std::vector<StaticAvoidanceShiftArc> &shift_arcs) {
  if (arcs_from_ego_m.empty() || ego_index >= arcs_from_ego_m.size() ||
      !std::isfinite(current_shift_m)) {
    throw std::invalid_argument("정적 회피 이동선 병합 입력이 유효하지 않습니다");
  }
  for (std::size_t index = 0; index < arcs_from_ego_m.size(); ++index) {
    if (!std::isfinite(arcs_from_ego_m[index]) ||
        (index > 0 && arcs_from_ego_m[index] < arcs_from_ego_m[index - 1])) {
      throw std::invalid_argument("정적 회피 호길이 순서가 유효하지 않습니다");
    }
  }
  for (const auto &line : shift_arcs) {
    if (!std::isfinite(line.start_m) || !std::isfinite(line.end_m) ||
        !std::isfinite(line.start_shift_m) ||
        !std::isfinite(line.end_shift_m) ||
        line.end_m <= line.start_m) {
      throw std::invalid_argument("정적 회피 이동선 구간이 유효하지 않습니다");
    }
  }
  StaticAvoidanceMergedShiftProfile profile;
  profile.positive_m.assign(arcs_from_ego_m.size(), 0.0);
  profile.negative_m.assign(arcs_from_ego_m.size(), 0.0);
  profile.combined_m.assign(arcs_from_ego_m.size(), 0.0);
  for (std::size_t index = 0; index < arcs_from_ego_m.size(); ++index) {
    const double arc = arcs_from_ego_m[index];
    bool has_object = false;
    for (const auto &line : shift_arcs) {
      if (arc < line.start_m || arc > line.end_m) continue;
      has_object = true;
      const double shift = arc < line.end_m ?
          line.start_shift_m +
              (arc - line.start_m) /
                  (line.end_m - line.start_m) *
                  (line.end_shift_m - line.start_shift_m) :
          0.0;
      profile.positive_m[index] = std::max(profile.positive_m[index], shift);
      profile.negative_m[index] = std::min(profile.negative_m[index], shift);
    }
    if (index <= ego_index) {
      profile.combined_m[index] = current_shift_m;
    } else if (!has_object) {
      profile.combined_m[index] = profile.combined_m[index - 1];
    } else {
      profile.combined_m[index] = profile.positive_m[index] +
                                  profile.negative_m[index];
    }
  }
  return profile;
}

std::vector<StaticAvoidanceShiftArc> extractStaticAvoidanceShiftArcs(
    const std::vector<double> &arcs_from_ego_m,
    std::size_t ego_index,
    const std::vector<double> &shift_profile_m,
    double gradient_change_threshold) {
  const std::size_t count = arcs_from_ego_m.size();
  if (count < 2 || shift_profile_m.size() != count || ego_index >= count ||
      !std::isfinite(gradient_change_threshold) ||
      gradient_change_threshold < 0.0) {
    throw std::invalid_argument("정적 회피 이동선 추출 입력이 유효하지 않습니다");
  }
  for (std::size_t index = 0; index < count; ++index) {
    if (!std::isfinite(arcs_from_ego_m[index]) ||
        !std::isfinite(shift_profile_m[index]) ||
        (index > 0 && arcs_from_ego_m[index] -
            arcs_from_ego_m[index - 1] < 1.0e-5)) {
      throw std::invalid_argument("정적 회피 이동선 표본이 유효하지 않습니다");
    }
  }
  const auto backward = [&](std::size_t index) {
    if (index == 0) return 0.0;
    return (shift_profile_m[index] - shift_profile_m[index - 1]) /
           (arcs_from_ego_m[index] - arcs_from_ego_m[index - 1]);
  };
  const auto forward = [&](std::size_t index) {
    if (index + 1 == count) return 0.0;
    return (shift_profile_m[index + 1] - shift_profile_m[index]) /
           (arcs_from_ego_m[index + 1] - arcs_from_ego_m[index]);
  };
  std::vector<StaticAvoidanceShiftArc> lines;
  std::optional<std::size_t> start;
  for (std::size_t index = ego_index; index + 1 < count; ++index) {
    if (!start && std::abs(shift_profile_m[index]) > 0.001) {
      start = index;
    }
    if (std::abs(forward(index) - backward(index)) <=
        gradient_change_threshold) {
      continue;
    }
    if (!start) {
      start = index;
      continue;
    }
    if (*start < index) {
      lines.push_back({arcs_from_ego_m[*start], arcs_from_ego_m[index],
                       shift_profile_m[*start], shift_profile_m[index]});
    }
    start = index;
  }
  if (start && *start < count - 1 &&
      std::abs(backward(count - 1)) > gradient_change_threshold) {
    lines.push_back({arcs_from_ego_m[*start], arcs_from_ego_m.back(),
                     shift_profile_m[*start], shift_profile_m.back()});
  }
  return lines;
}

std::vector<StaticAvoidanceShiftArc> trimStaticAvoidanceShiftArcs(
    std::vector<StaticAvoidanceShiftArc> shift_arcs,
    const StaticAvoidanceTrimParameters &parameters) {
  const double values[] = {
      parameters.minimum_prepare_distance_m,
      parameters.minimum_arc_length_m,
      parameters.quantize_size_m,
      parameters.similar_gradient_first_m,
      parameters.similar_gradient_second_m,
      parameters.similar_gradient_third_m};
  for (const double value : values) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("정적 회피 이동선 잡음 제거 값이 유효하지 않습니다");
    }
  }
  for (const auto &line : shift_arcs) {
    if (!std::isfinite(line.start_m) || !std::isfinite(line.end_m) ||
        !std::isfinite(line.start_shift_m) ||
        !std::isfinite(line.end_shift_m) || line.end_m <= line.start_m) {
      throw std::invalid_argument("정적 회피 이동선 구간이 유효하지 않습니다");
    }
  }
  const auto order = [](const StaticAvoidanceShiftArc &left,
                        const StaticAvoidanceShiftArc &right) {
    return left.start_m < right.start_m;
  };
  std::stable_sort(shift_arcs.begin(), shift_arcs.end(), order);
  const auto remove_short = [&]() {
    shift_arcs.erase(std::remove_if(
        shift_arcs.begin(), shift_arcs.end(), [&](const auto &line) {
          return line.end_m - line.start_m <
                     parameters.minimum_arc_length_m ||
                 line.start_m <= parameters.minimum_prepare_distance_m;
        }), shift_arcs.end());
  };
  const auto merge_gradients = [&](double threshold) {
    if (shift_arcs.empty()) return;
    std::vector<StaticAvoidanceShiftArc> merged{shift_arcs.front()};
    StaticAvoidanceShiftArc base = shift_arcs.front();
    for (std::size_t index = 1; index < shift_arcs.size(); ++index) {
      const auto &next = shift_arcs[index];
      const double base_delta = base.end_shift_m - base.start_shift_m;
      const double next_delta = next.end_shift_m - next.start_shift_m;
      const double base_gradient =
          base_delta / (base.end_m - base.start_m);
      const double expected_delta = base_gradient *
          (next.end_m - base.start_m);
      const bool opposite =
          (base_delta < -0.001 && next_delta > 0.001) ||
          (base_delta > 0.001 && next_delta < -0.001);
      const bool violates = opposite ||
          std::abs(next.end_shift_m - base.start_shift_m -
                   expected_delta) > threshold;
      if (violates) {
        merged.push_back(next);
        base = next;
      } else {
        merged.back() = {base.start_m, next.end_m,
                         base.start_shift_m, next.end_shift_m};
      }
    }
    shift_arcs = std::move(merged);
    std::stable_sort(shift_arcs.begin(), shift_arcs.end(), order);
  };
  remove_short();
  merge_gradients(parameters.similar_gradient_first_m);
  if (parameters.quantize_size_m >= 1.0e-5) {
    for (auto &line : shift_arcs) {
      line.end_shift_m = std::round(
          line.end_shift_m / parameters.quantize_size_m) *
          parameters.quantize_size_m;
    }
    std::stable_sort(shift_arcs.begin(), shift_arcs.end(), order);
  }
  remove_short();
  merge_gradients(parameters.similar_gradient_second_m);
  merge_gradients(parameters.similar_gradient_third_m);
  return shift_arcs;
}

std::vector<StaticAvoidanceShiftArc> mergeStaticAvoidanceRegisteredShiftArcs(
    const std::vector<StaticAvoidanceShiftArc> &registered,
    const std::vector<StaticAvoidanceShiftArc> &proposed) {
  if (proposed.empty()) return registered;
  const auto valid = [](const StaticAvoidanceShiftArc &line) {
    return std::isfinite(line.start_m) && std::isfinite(line.end_m) &&
           std::isfinite(line.start_shift_m) &&
           std::isfinite(line.end_shift_m) && line.end_m > line.start_m;
  };
  for (const auto &line : registered) {
    if (!valid(line)) {
      throw std::invalid_argument("등록된 정적 회피 이동선이 유효하지 않습니다");
    }
  }
  for (const auto &line : proposed) {
    if (!valid(line)) {
      throw std::invalid_argument("새 정적 회피 이동선이 유효하지 않습니다");
    }
  }
  double first_new_start = proposed.front().start_m;
  for (const auto &line : proposed) {
    first_new_start = std::min(first_new_start, line.start_m);
  }
  const double first_new_end = proposed.front().end_m;
  const double first_new_shift = proposed.front().end_shift_m;
  std::vector<StaticAvoidanceShiftArc> future = proposed;
  for (const auto &old : registered) {
    if (old.start_m >= first_new_start) continue;
    if (old.end_m > first_new_end) {
      if (old.end_shift_m > -1.0e-3 &&
          first_new_shift > -1.0e-3 &&
          old.end_shift_m < first_new_shift) {
        continue;
      }
      if (old.end_shift_m < 1.0e-3 &&
          first_new_shift < 1.0e-3 &&
          old.end_shift_m > first_new_shift) {
        continue;
      }
    }
    future.push_back(old);
  }
  std::stable_sort(future.begin(), future.end(),
                   [](const auto &left, const auto &right) {
                     return left.start_m < right.start_m;
                   });
  return future;
}

std::vector<StaticAvoidanceShiftArc>
reprojectStaticAvoidanceRegisteredShiftArcs(
    const Trajectory &reference, std::size_t ego_index,
    double ego_linear_shift_m,
    const std::vector<StaticAvoidanceAnchoredShiftArc> &registered) {
  if (reference.empty() || ego_index >= reference.size() ||
      !std::isfinite(ego_linear_shift_m)) {
    throw std::invalid_argument("등록된 회피 이동선의 기준 경로가 유효하지 않습니다");
  }
  std::vector<double> arcs(reference.size(), 0.0);
  for (std::size_t index = 0; index < reference.size(); ++index) {
    if (!std::isfinite(reference[index].x) ||
        !std::isfinite(reference[index].y)) {
      throw std::invalid_argument("정적 회피 기준 경로 좌표가 유효하지 않습니다");
    }
    if (index > 0) {
      arcs[index] = arcs[index - 1] + std::hypot(
          reference[index].x - reference[index - 1].x,
          reference[index].y - reference[index - 1].y);
    }
  }
  const auto nearest = [&](double x, double y) {
    std::size_t selected = 0;
    double minimum = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < reference.size(); ++index) {
      const double distance = std::hypot(
          reference[index].x - x, reference[index].y - y);
      if (distance < minimum) {
        minimum = distance;
        selected = index;
      }
    }
    return selected;
  };
  std::vector<StaticAvoidanceShiftArc> result;
  for (const auto &anchor : registered) {
    if (!std::isfinite(anchor.start_x) ||
        !std::isfinite(anchor.start_y) ||
        !std::isfinite(anchor.end_x) ||
        !std::isfinite(anchor.end_y) ||
        !std::isfinite(anchor.start_shift_m) ||
        !std::isfinite(anchor.end_shift_m)) {
      throw std::invalid_argument("등록된 회피 이동선의 기준점이 유효하지 않습니다");
    }
    const auto start_index = nearest(anchor.start_x, anchor.start_y);
    const auto end_index = nearest(anchor.end_x, anchor.end_y);
    if (end_index < ego_index) continue;
    const double start_m = arcs[start_index] - arcs[ego_index];
    const double end_m = arcs[end_index] - arcs[ego_index];
    if (end_m <= start_m) continue;
    if (start_m < 0.0) {
      const double gradient =
          (anchor.end_shift_m - anchor.start_shift_m) /
          (end_m - start_m);
      const double registered_at_ego =
          anchor.start_shift_m - gradient * start_m;
      if (std::abs(ego_linear_shift_m - registered_at_ego) > 0.1) {
        continue;
      }
    }
    result.push_back(StaticAvoidanceShiftArc{
        start_m, end_m,
        anchor.start_shift_m, anchor.end_shift_m});
  }
  return result;
}

std::optional<StaticAvoidanceRegisteredPlan>
planStaticAvoidanceWithRegisteredShifts(
    const Trajectory &reference, std::size_t ego_index,
    double ego_linear_shift_m,
    const std::vector<StaticAvoidanceAnchoredShiftArc> &registered,
    const std::vector<StaticAvoidanceShiftArc> &proposed,
    bool approved,
    const BehaviorPathShiftParameters &shift_parameters) {
  if (reference.size() < 3 || ego_index >= reference.size()) {
    throw std::invalid_argument("등록 회피 경로 입력이 유효하지 않습니다");
  }
  std::vector<double> arcs(reference.size(), 0.0);
  for (std::size_t index = 1; index < reference.size(); ++index) {
    arcs[index] = arcs[index - 1] + std::hypot(
        reference[index].x - reference[index - 1].x,
        reference[index].y - reference[index - 1].y);
  }
  const auto retained = reprojectStaticAvoidanceRegisteredShiftArcs(
      reference, ego_index, ego_linear_shift_m, registered);
  const auto active = approved ?
      mergeStaticAvoidanceRegisteredShiftArcs(retained, proposed) : retained;
  StaticAvoidanceRegisteredPlan result;
  result.active_shift_arcs = active;
  if (active.empty()) {
    result.path = reference;
    return result;
  }
  const auto shifted = generateStaticAvoidanceMergedPath(
      reference, arcs[ego_index], active, shift_parameters);
  if (!shifted) return std::nullopt;
  result.path = *shifted;
  const auto point_at = [&](double relative_arc_m) {
    const double arc = std::clamp(
        arcs[ego_index] + relative_arc_m, 0.0, arcs.back());
    const auto upper = std::upper_bound(arcs.begin(), arcs.end(), arc);
    const std::size_t segment = std::min(
        static_cast<std::size_t>(upper - arcs.begin() - 1),
        reference.size() - 2);
    const double length = arcs[segment + 1] - arcs[segment];
    const double ratio = length > 1.0e-9 ?
        (arc - arcs[segment]) / length : 0.0;
    return std::pair<double, double>{
        reference[segment].x + ratio *
            (reference[segment + 1].x - reference[segment].x),
        reference[segment].y + ratio *
            (reference[segment + 1].y - reference[segment].y)};
  };
  for (const auto &line : active) {
    bool reused = false;
    for (const auto &old_anchor : registered) {
      const auto old_line = reprojectStaticAvoidanceRegisteredShiftArcs(
          reference, ego_index, ego_linear_shift_m, {old_anchor});
      if (old_line.size() != 1) continue;
      if (std::abs(old_line.front().start_m - line.start_m) < 1.0e-6 &&
          std::abs(old_line.front().end_m - line.end_m) < 1.0e-6 &&
          std::abs(old_line.front().start_shift_m - line.start_shift_m) <
              1.0e-6 &&
          std::abs(old_line.front().end_shift_m - line.end_shift_m) <
              1.0e-6) {
        result.registered_shift_arcs.push_back(old_anchor);
        reused = true;
        break;
      }
    }
    if (reused) continue;
    const auto start = point_at(line.start_m);
    const auto end = point_at(line.end_m);
    result.registered_shift_arcs.push_back({
        start.first, start.second, end.first, end.second,
        line.start_shift_m, line.end_shift_m});
  }
  return result;
}

std::optional<Trajectory> generateStaticAvoidancePath(
    const Trajectory &reference, double ego_arc_m,
    const StaticAvoidanceOutline &outline,
    const BehaviorPathShiftParameters &parameters) {
  std::vector<StaticAvoidanceShiftArc> lines{outline.avoid};
  if (outline.return_to_center) lines.push_back(*outline.return_to_center);
  return generateStaticAvoidanceMergedPath(
      reference, ego_arc_m, lines, parameters);
}

std::optional<Trajectory> generateStaticAvoidanceMergedPath(
    const Trajectory &reference, double ego_arc_m,
    const std::vector<StaticAvoidanceShiftArc> &shift_arcs,
    const BehaviorPathShiftParameters &parameters) {
  if (reference.size() < 3 || !std::isfinite(ego_arc_m) || ego_arc_m < 0.0) {
    throw std::invalid_argument("정적 회피 기준 경로가 유효하지 않습니다");
  }
  if (shift_arcs.empty()) return std::nullopt;
  std::vector<double> arcs(reference.size(), 0.0);
  for (std::size_t index = 1; index < reference.size(); ++index) {
    arcs[index] = arcs[index - 1] + std::hypot(
        reference[index].x - reference[index - 1].x,
        reference[index].y - reference[index - 1].y);
  }
  const auto index_at = [&](double relative_arc) -> std::optional<std::size_t> {
    const double target = ego_arc_m + relative_arc;
    if (!std::isfinite(target) || target < 0.0 || target > arcs.back()) {
      return std::nullopt;
    }
    const auto after = std::lower_bound(arcs.begin(), arcs.end(), target);
    if (after == arcs.begin()) return 0;
    if (after == arcs.end()) return arcs.size() - 1;
    const auto before = std::prev(after);
    return target - *before <= *after - target
        ? static_cast<std::size_t>(before - arcs.begin())
        : static_cast<std::size_t>(after - arcs.begin());
  };
  std::vector<BehaviorPathShiftLine> lines;
  const auto append = [&](StaticAvoidanceShiftArc shift) {
    const auto start = index_at(shift.start_m);
    const auto end = index_at(shift.end_m);
    if (!start || !end || *end <= *start + 1) return false;
    lines.push_back({*start, *end, shift.end_shift_m});
    return true;
  };
  for (const auto &shift : shift_arcs) {
    if (!append(shift)) return std::nullopt;
  }
  auto settings = parameters;
  settings.base_offset_m = shift_arcs.front().start_shift_m;
  try {
    return generateBehaviorShiftedPath(reference, lines, settings);
  } catch (const std::invalid_argument &) {
    return std::nullopt;
  }
}

std::optional<StaticAvoidanceMultiObjectPlan> planStaticAvoidanceMultiObject(
    const Trajectory &reference, std::size_t ego_index,
    double ego_arc_m, double current_shift_m,
    const std::vector<StaticAvoidanceShiftArc> &raw_shift_arcs,
    const StaticAvoidanceTrimParameters &trim_parameters,
    const BehaviorPathShiftParameters &shift_parameters) {
  if (reference.size() < 3 || ego_index >= reference.size() ||
      !std::isfinite(ego_arc_m) || ego_arc_m < 0.0 ||
      !std::isfinite(current_shift_m)) {
    throw std::invalid_argument("정적 회피 다중 객체 경로 입력이 유효하지 않습니다");
  }
  std::vector<double> relative_arcs(reference.size(), -ego_arc_m);
  for (std::size_t index = 1; index < reference.size(); ++index) {
    relative_arcs[index] = relative_arcs[index - 1] + std::hypot(
        reference[index].x - reference[index - 1].x,
        reference[index].y - reference[index - 1].y);
  }
  auto profile = mergeStaticAvoidanceShiftArcs(
      relative_arcs, ego_index, current_shift_m, raw_shift_arcs);
  auto lines = extractStaticAvoidanceShiftArcs(
      relative_arcs, ego_index, profile.combined_m);
  lines = trimStaticAvoidanceShiftArcs(std::move(lines), trim_parameters);
  const auto path = generateStaticAvoidanceMergedPath(
      reference, ego_arc_m, lines, shift_parameters);
  if (!path) return std::nullopt;
  return StaticAvoidanceMultiObjectPlan{
      std::move(profile), std::move(lines), *path};
}

std::optional<StaticAvoidanceSingleObjectPlan>
planStaticAvoidanceSingleObject(
    const Trajectory &reference,
    const StaticAvoidanceSingleObjectInput &input) {
  if (reference.size() < 3) return std::nullopt;
  auto geometry = staticAvoidanceObjectGeometry(
      reference, input.ego_x, input.ego_y,
      input.object_polygon, input.closest_reference_pose,
      input.envelope_buffer_m, input.object_on_right,
      input.margin, input.boundary);
  if (!geometry || std::abs(geometry->desired_shift_m) < 1.0e-3 ||
      !staticAvoidanceShiftNecessary(
          input.object_on_right, geometry->desired_shift_m)) {
    return std::nullopt;
  }
  auto feasibility_input = input.feasibility;
  feasibility_input.desired_shift_m = geometry->desired_shift_m;
  feasibility_input.object_longitudinal_m =
      geometry->longitudinal.start_from_ego_m;
  feasibility_input.object_on_right = input.object_on_right;
  feasibility_input.overhang_m = geometry->nearest_overhang.lateral_m;
  feasibility_input.vehicle_width_m = input.margin.vehicle_width_m;
  feasibility_input.lateral_hard_margin_m = input.margin.parked ?
      input.margin.parked_lateral_hard_margin_m :
      input.margin.lateral_hard_margin_m;
  const auto feasibility = staticAvoidanceFeasibleShift(feasibility_input);
  if (feasibility.reason !=
          StaticAvoidanceFeasibilityReason::Feasible ||
      feasibility.avoidance_distance_m <= 0.0) {
    return std::nullopt;
  }
  auto outline_input = input.outline;
  outline_input.object_longitudinal_m =
      geometry->longitudinal.start_from_ego_m;
  outline_input.avoidance_distance_m =
      feasibility.avoidance_distance_m;
  outline_input.minimum_avoidance_distance_m =
      4.0 * std::cbrt(
          0.5 * std::abs(feasibility.shift_m -
                         feasibility_input.current_ego_shift_m) /
          feasibility_input.minimum_distance_jerk_mps3) *
      feasibility_input.minimum_distance_speed_mps;
  outline_input.target_shift_m = feasibility.shift_m;
  const auto outline = staticAvoidanceShiftOutline(outline_input);
  if (!outline) return std::nullopt;
  std::vector<double> arcs(reference.size(), 0.0);
  for (std::size_t index = 1; index < reference.size(); ++index) {
    arcs[index] = arcs[index - 1] +
        std::hypot(reference[index].x - reference[index - 1].x,
                   reference[index].y - reference[index - 1].y);
  }
  const double ego_arc = projectedArc(
      reference, arcs, input.ego_x, input.ego_y);
  if (ego_arc < 0.0) return std::nullopt;
  auto path = generateStaticAvoidancePath(
      reference, ego_arc, *outline, input.path_shift);
  if (!path) return std::nullopt;
  return StaticAvoidanceSingleObjectPlan{
      std::move(*geometry), feasibility, *outline, std::move(*path)};
}

}  // planning 네임스페이스
