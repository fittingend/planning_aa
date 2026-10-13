// SPDX-License-Identifier: Apache-2.0
// 목표 앞 횡이동과 두 점 스플라인 진입을 독립 C++ 경로로 계산합니다.

#include "planning/behavior_pull_over_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {

struct Projection {
  double arc_m = 0.0;
  double lateral_m = 0.0;
  std::size_t next_index = 0;
  double segment_ratio = 0.0;
};

Projection project(const Trajectory &path, double x, double y) {
  Projection result;
  double nearest_distance = std::numeric_limits<double>::infinity();
  double accumulated = 0.0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const auto &a = path[index];
    const auto &b = path[index + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length = std::hypot(dx, dy);
    if (length < 1.0e-9) continue;
    const double fraction = std::clamp(
        ((x - a.x) * dx + (y - a.y) * dy) /
            (length * length), 0.0, 1.0);
    const double residual_x = x - a.x - fraction * dx;
    const double residual_y = y - a.y - fraction * dy;
    const double distance = std::hypot(residual_x, residual_y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      result.arc_m = accumulated + fraction * length;
      result.lateral_m =
          (dx * residual_y - dy * residual_x) / length;
      result.next_index = index + 1;
      result.segment_ratio = fraction;
    }
    accumulated += length;
  }
  return result;
}

double heading(const TrajectoryPoint &point) {
  return std::atan2(
      2.0 * (point.orientation_w * point.orientation_z +
             point.orientation_x * point.orientation_y),
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z));
}

double curvature(const Trajectory &path, std::size_t index) {
  if (index == 0 || index + 1 >= path.size()) return 0.0;
  const auto &a = path[index - 1];
  const auto &b = path[index];
  const auto &c = path[index + 1];
  const double first = std::hypot(b.x - a.x, b.y - a.y);
  const double second = std::hypot(c.x - b.x, c.y - b.y);
  const double third = std::hypot(c.x - a.x, c.y - a.y);
  if (first * second * third < 1.0e-9) return 0.0;
  return 2.0 * ((b.x - a.x) * (c.y - a.y) -
                (b.y - a.y) * (c.x - a.x)) /
         (first * second * third);
}

std::size_t nearestIndex(const Trajectory &path,
                         const TrajectoryPoint &point) {
  std::size_t nearest = 0;
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < path.size(); ++index) {
    const double candidate = std::hypot(
        path[index].x - point.x, path[index].y - point.y);
    if (candidate < distance) {
      distance = candidate;
      nearest = index;
    }
  }
  return nearest;
}

}  // 익명 네임스페이스

std::optional<ShiftPullOverCandidate> generateShiftPullOverCandidate(
    const Trajectory &reference, const TrajectoryPoint &ego,
    BehaviorGoalPose goal,
    const ShiftPullOverParameters &parameters) {
  if (reference.size() < 3 ||
      !std::isfinite(goal.x) || !std::isfinite(goal.y) ||
      !std::isfinite(goal.yaw_rad) ||
      !std::isfinite(parameters.minimum_lateral_jerk_mps3) ||
      !std::isfinite(parameters.maximum_lateral_jerk_mps3) ||
      !std::isfinite(parameters.pull_over_velocity_mps) ||
      !std::isfinite(parameters.after_shift_straight_distance_m) ||
      !std::isfinite(parameters.interpolation_interval_m) ||
      parameters.minimum_lateral_jerk_mps3 <= 0.0 ||
      parameters.maximum_lateral_jerk_mps3 <
          parameters.minimum_lateral_jerk_mps3 ||
      parameters.jerk_samples <= 0 ||
      parameters.pull_over_velocity_mps <= 0.0 ||
      parameters.after_shift_straight_distance_m < 0.0 ||
      parameters.interpolation_interval_m <= 0.0) {
    throw std::invalid_argument("주차 횡이동 입력이 유효하지 않습니다");
  }
  const double shift_end_x = goal.x -
      parameters.after_shift_straight_distance_m *
          std::cos(goal.yaw_rad);
  const double shift_end_y = goal.y -
      parameters.after_shift_straight_distance_m *
          std::sin(goal.yaw_rad);
  Trajectory working_reference = reference;
  const auto &previous = reference[reference.size() - 2];
  const auto &last = reference.back();
  const double final_dx = last.x - previous.x;
  const double final_dy = last.y - previous.y;
  const double final_length = std::hypot(final_dx, final_dy);
  if (final_length < 1.0e-9) return std::nullopt;
  const double beyond_last =
      ((shift_end_x - last.x) * final_dx +
       (shift_end_y - last.y) * final_dy) / final_length;
  if (beyond_last > 1.0e-9) {
    if (working_reference.back().longitudinal_velocity_mps <= 0.0F)
      working_reference.back().longitudinal_velocity_mps =
          static_cast<float>(parameters.pull_over_velocity_mps);
    const int intervals = std::max(1, static_cast<int>(
        std::ceil(beyond_last / parameters.interpolation_interval_m)));
    for (int index = 1; index <= intervals; ++index) {
      const double distance = beyond_last * index / intervals;
      auto extended = working_reference[reference.size() - 1];
      extended.x += distance * final_dx / final_length;
      extended.y += distance * final_dy / final_length;
      working_reference.push_back(extended);
    }
  }
  const auto ego_projection = project(working_reference, ego.x, ego.y);
  const auto end_projection = project(
      working_reference, shift_end_x, shift_end_y);
  if (end_projection.arc_m <= ego_projection.arc_m + 1.0 ||
      end_projection.next_index >= working_reference.size())
    return std::nullopt;
  std::vector<double> arcs(working_reference.size(), 0.0);
  for (std::size_t index = 1; index < working_reference.size(); ++index) {
    arcs[index] = arcs[index - 1] + std::hypot(
        working_reference[index].x - working_reference[index - 1].x,
        working_reference[index].y - working_reference[index - 1].y);
  }
  const double jerk_step =
      (parameters.maximum_lateral_jerk_mps3 -
       parameters.minimum_lateral_jerk_mps3) /
      parameters.jerk_samples;
  for (int sample = 0; sample <= parameters.jerk_samples; ++sample) {
    const double jerk = parameters.minimum_lateral_jerk_mps3 +
                        sample * jerk_step;
    const double shift_distance = 4.0 * std::cbrt(
        0.5 * std::abs(end_projection.lateral_m) / jerk) *
        parameters.pull_over_velocity_mps;
    const double start_arc = end_projection.arc_m -
                             std::max(shift_distance, 1.0);
    if (start_arc <= ego_projection.arc_m || start_arc < 0.0) continue;
    const auto start_it = std::lower_bound(
        arcs.begin(), arcs.end(), start_arc);
    const auto start_index = static_cast<std::size_t>(
        start_it - arcs.begin());
    Trajectory input(working_reference.begin(), working_reference.begin() +
        static_cast<std::ptrdiff_t>(end_projection.next_index));
    const auto &end_next = working_reference[end_projection.next_index];
    auto projected_end = input.back();
    projected_end.x += end_projection.segment_ratio *
        (end_next.x - projected_end.x);
    projected_end.y += end_projection.segment_ratio *
        (end_next.y - projected_end.y);
    projected_end.z += end_projection.segment_ratio *
        (end_next.z - projected_end.z);
    if (end_projection.segment_ratio > 1.0 - 1.0e-9)
      projected_end = end_next;
    if (end_projection.segment_ratio > 1.0e-9)
      input.push_back(projected_end);
    const auto end_index = input.size() - 1;
    if (start_index + 2 >= end_index) continue;
    BehaviorPathShiftParameters shift;
    shift.offset_back = true;
    shift.velocity_mps = parameters.pull_over_velocity_mps;
    Trajectory shifted;
    try {
      shifted = generateBehaviorShiftedPath(
          input, {{start_index, end_index,
                   end_projection.lateral_m}}, shift);
    } catch (const std::invalid_argument &) {
      continue;
    }
    if (shifted.size() < 2) continue;
    const std::size_t actual_end_index = shifted.size() - 1;
    std::size_t actual_start_index = 0;
    double start_error = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < shifted.size(); ++index) {
      const double error = std::hypot(
          shifted[index].x - input[start_index].x,
          shifted[index].y - input[start_index].y);
      if (error < start_error) {
        start_error = error;
        actual_start_index = index;
      }
    }
    for (std::size_t index = actual_start_index;
         index < shifted.size(); ++index) {
      shifted[index].longitudinal_velocity_mps = std::min(
          shifted[index].longitudinal_velocity_mps,
          static_cast<float>(parameters.pull_over_velocity_mps));
    }
    const auto start_point = shifted.back();
    const double straight_distance = std::hypot(
        goal.x - start_point.x, goal.y - start_point.y);
    const double start_yaw = std::atan2(
        2.0 * (start_point.orientation_w *
                   start_point.orientation_z +
               start_point.orientation_x *
                   start_point.orientation_y),
        1.0 - 2.0 * (start_point.orientation_y *
                         start_point.orientation_y +
                     start_point.orientation_z *
                         start_point.orientation_z));
    const auto spline_coordinate = [straight_distance](
        double start, double end, double start_derivative,
        double end_derivative, double distance) {
      const double squared = straight_distance * straight_distance;
      const double cubic =
          (end_derivative * straight_distance - 2.0 * end +
           start_derivative * straight_distance + 2.0 * start) /
          (squared * straight_distance);
      const double quadratic =
          (3.0 * end - end_derivative * straight_distance -
           2.0 * start_derivative * straight_distance -
           3.0 * start) / squared;
      return start + (start_derivative +
                      (quadratic + cubic * distance) * distance) *
                         distance;
    };
    std::vector<TrajectoryPoint> straight;
    for (double distance = 0.3;
         distance < straight_distance - 0.3;
         distance += parameters.interpolation_interval_m) {
      TrajectoryPoint point = start_point;
      point.x = spline_coordinate(
          start_point.x, goal.x, std::cos(start_yaw),
          std::cos(goal.yaw_rad), distance);
      point.y = spline_coordinate(
          start_point.y, goal.y, std::sin(start_yaw),
          std::sin(goal.yaw_rad), distance);
      straight.push_back(point);
    }
    for (std::size_t index = 0; index < straight.size(); ++index) {
      const double next_x = index + 1 < straight.size() ?
          straight[index + 1].x : goal.x;
      const double next_y = index + 1 < straight.size() ?
          straight[index + 1].y : goal.y;
      const double yaw = std::atan2(
          next_y - straight[index].y,
          next_x - straight[index].x);
      auto &point = straight[index];
      point.orientation_x = 0.0;
      point.orientation_y = 0.0;
      point.orientation_z = std::sin(yaw / 2.0);
      point.orientation_w = std::cos(yaw / 2.0);
    }
    shifted.insert(shifted.end(), straight.begin(), straight.end());
    auto goal_point = start_point;
    goal_point.x = goal.x;
    goal_point.y = goal.y;
    goal_point.orientation_x = 0.0;
    goal_point.orientation_y = 0.0;
    goal_point.orientation_z = std::sin(goal.yaw_rad / 2.0);
    goal_point.orientation_w = std::cos(goal.yaw_rad / 2.0);
    goal_point.longitudinal_velocity_mps = 0.0F;
    shifted.push_back(goal_point);
    return ShiftPullOverCandidate{
        std::move(shifted), actual_start_index, actual_end_index,
        end_projection.lateral_m, jerk, {}};
  }
  return std::nullopt;
}

std::vector<ShiftPullOverCandidate> generateBezierPullOverCandidates(
    const Trajectory &reference, const TrajectoryPoint &ego,
    BehaviorGoalPose goal,
    const ShiftPullOverParameters &parameters) {
  const auto base = generateShiftPullOverCandidate(
      reference, ego, goal, parameters);
  std::vector<ShiftPullOverCandidate> candidates;
  if (!base || base->shift_start_index + 2 >=
                   base->shift_end_index ||
      base->shift_end_index >= base->path.size())
    return candidates;
  candidates.reserve(48);
  const auto first_index = base->shift_start_index;
  const auto last_index = base->shift_end_index;
  const auto &initial = base->path[first_index];
  const auto &final = base->path[last_index];
  const double first_yaw = heading(initial);
  const double final_yaw = heading(final);
  const double distance = std::hypot(
      final.x - initial.x, final.y - initial.y);
  if (distance < 1.0e-6) return candidates;
  const double first_curvature = curvature(
      reference, nearestIndex(reference, initial));
  const double final_curvature = curvature(
      reference, nearestIndex(reference, final));
  constexpr std::array<double, 6> coefficients{1.0, 5.0, 10.0,
                                                10.0, 5.0, 1.0};
  constexpr std::array<double, 5> derivative_coefficients{
      1.0, 4.0, 6.0, 4.0, 1.0};
  for (int first_sample = 0; first_sample <= 3; ++first_sample) {
    for (int final_sample = 0; final_sample <= 3; ++final_sample) {
      for (int acceleration_sample = 0;
           acceleration_sample <= 2; ++acceleration_sample) {
        const double initial_tangent_length =
            first_sample * distance / 3.0;
        const double final_tangent_length =
            final_sample * 0.75 * distance;
        const double acceleration_length =
            acceleration_sample * 5.0 * distance;
        const double first_cosine = std::cos(first_yaw);
        const double first_sine = std::sin(first_yaw);
        const double final_cosine = std::cos(final_yaw);
        const double final_sine = std::sin(final_yaw);
        const double first_acc_x =
            acceleration_length * first_cosine -
            first_curvature * initial_tangent_length *
                initial_tangent_length * first_sine;
        const double first_acc_y =
            acceleration_length * first_sine +
            first_curvature * initial_tangent_length *
                initial_tangent_length * first_cosine;
        const double final_acc_x =
            acceleration_length * final_cosine -
            final_curvature * final_tangent_length *
                final_tangent_length * final_sine;
        const double final_acc_y =
            acceleration_length * final_sine +
            final_curvature * final_tangent_length *
                final_tangent_length * final_cosine;
        std::array<std::array<double, 2>, 6> control{};
        control[0] = {initial.x, initial.y};
        control[5] = {final.x, final.y};
        control[1] = {
            initial.x + initial_tangent_length * first_cosine / 5.0,
            initial.y + initial_tangent_length * first_sine / 5.0};
        control[4] = {
            final.x - final_tangent_length * final_cosine / 5.0,
            final.y - final_tangent_length * final_sine / 5.0};
        control[2] = {
            2.0 * control[1][0] - control[0][0] + first_acc_x / 20.0,
            2.0 * control[1][1] - control[0][1] + first_acc_y / 20.0};
        control[3] = {
            2.0 * control[4][0] - control[5][0] - final_acc_x / 20.0,
            2.0 * control[4][1] - control[5][1] - final_acc_y / 20.0};
        auto candidate = *base;
        const double count = static_cast<double>(
            last_index - first_index);
        for (std::size_t index = first_index;
             index < last_index; ++index) {
          const double t = (index - first_index) / count;
          const double one_minus = 1.0 - t;
          double x = 0.0;
          double y = 0.0;
          double derivative_x = 0.0;
          double derivative_y = 0.0;
          for (int control_index = 0; control_index < 6;
               ++control_index) {
            const double weight = coefficients[control_index] *
                std::pow(one_minus, 5 - control_index) *
                std::pow(t, control_index);
            x += weight * control[control_index][0];
            y += weight * control[control_index][1];
            if (control_index < 5) {
              const double derivative_weight =
                  5.0 * derivative_coefficients[control_index] *
                  std::pow(one_minus, 4 - control_index) *
                  std::pow(t, control_index);
              derivative_x += derivative_weight *
                  (control[control_index + 1][0] -
                   control[control_index][0]);
              derivative_y += derivative_weight *
                  (control[control_index + 1][1] -
                   control[control_index][1]);
            }
          }
          auto &point = candidate.path[index];
          point.x = x;
          point.y = y;
          const double yaw = std::atan2(derivative_y, derivative_x);
          point.orientation_x = 0.0;
          point.orientation_y = 0.0;
          point.orientation_z = std::sin(yaw / 2.0);
          point.orientation_w = std::cos(yaw / 2.0);
        }
        candidates.push_back(std::move(candidate));
      }
    }
  }
  return candidates;
}

std::optional<ShiftPullOverCandidate> generateGeometricPullOverCandidate(
    const Trajectory &reference, const TrajectoryPoint &ego,
    BehaviorGoalPose goal,
    const GeometricPullOverParameters &parameters) {
  if (reference.size() < 3 || !std::isfinite(goal.x) ||
      !std::isfinite(goal.y) || !std::isfinite(goal.yaw_rad) ||
      !std::isfinite(parameters.wheelbase_m) ||
      !std::isfinite(parameters.maximum_steering_angle_rad) ||
      !std::isfinite(parameters.far_turn_radius_m) ||
      !std::isfinite(parameters.arc_interval_m) ||
      !std::isfinite(parameters.after_straight_distance_m) ||
      !std::isfinite(parameters.forward_velocity_mps) ||
      !std::isfinite(parameters.backward_velocity_mps) ||
      parameters.wheelbase_m <= 0.0 ||
      parameters.maximum_steering_angle_rad <= 0.0 ||
      parameters.maximum_steering_angle_rad >= std::acos(-1.0) / 2.0 ||
      parameters.far_turn_radius_m < 0.0 ||
      parameters.arc_interval_m <= 0.0 ||
      parameters.after_straight_distance_m < 0.0 ||
      parameters.forward_velocity_mps <= 0.0 ||
      parameters.backward_velocity_mps >= 0.0)
    throw std::invalid_argument("기하 주차 입력이 유효하지 않습니다");
  const auto ego_projection = project(reference, ego.x, ego.y);
  const auto goal_projection = project(reference, goal.x, goal.y);
  const double near_radius = parameters.wheelbase_m /
      std::tan(parameters.maximum_steering_angle_rad);
  const double far_radius = parameters.far_turn_radius_m > 0.0 ?
      parameters.far_turn_radius_m : near_radius;
  if (far_radius < near_radius) return std::nullopt;
  const double radius_sum = near_radius + far_radius;
  const double lateral = goal_projection.lateral_m;
  if (std::abs(lateral) < 0.1 ||
      std::abs(lateral) >= 2.0 * radius_sum) return std::nullopt;
  const double turn_angle = std::acos(
      1.0 - std::abs(lateral) / radius_sum);
  const double arc_forward_length = radius_sum *
      std::sin(turn_angle);
  const double start_arc = goal_projection.arc_m +
      (parameters.forward ? -1.0 : 1.0) *
      (parameters.after_straight_distance_m + arc_forward_length);
  if (start_arc < ego_projection.arc_m + 0.1 || start_arc < 0.0)
    return std::nullopt;
  std::vector<double> lengths(reference.size(), 0.0);
  for (std::size_t index = 1; index < reference.size(); ++index)
    lengths[index] = lengths[index - 1] + std::hypot(
        reference[index].x - reference[index - 1].x,
        reference[index].y - reference[index - 1].y);
  if (start_arc > lengths.back()) return std::nullopt;
  const auto upper = std::lower_bound(
      lengths.begin(), lengths.end(), start_arc);
  const std::size_t after = upper == lengths.begin() ? 1 :
      static_cast<std::size_t>(upper - lengths.begin());
  const std::size_t before = after - 1;
  const double segment = lengths[after] - lengths[before];
  if (segment < 1.0e-9) return std::nullopt;
  const double fraction = (start_arc - lengths[before]) / segment;
  TrajectoryPoint road_start = reference[before];
  road_start.x += fraction * (reference[after].x - road_start.x);
  road_start.y += fraction * (reference[after].y - road_start.y);
  const double road_yaw = std::atan2(
      reference[after].y - reference[before].y,
      reference[after].x - reference[before].x);
  if (std::abs(std::remainder(goal.yaw_rad - road_yaw,
                              2.0 * std::acos(-1.0))) > 0.15)
    return std::nullopt;
  const double cosine = std::cos(road_yaw);
  const double sine = std::sin(road_yaw);
  const double sign = lateral > 0.0 ? 1.0 : -1.0;
  const double expected_x = parameters.forward ?
      arc_forward_length + parameters.after_straight_distance_m :
      -arc_forward_length - parameters.after_straight_distance_m;
  const double expected_y = lateral;
  if (std::hypot(road_start.x + expected_x * cosine -
                     expected_y * sine - goal.x,
                 road_start.y + expected_x * sine +
                     expected_y * cosine - goal.y) > 0.35)
    return std::nullopt;
  Trajectory path;
  path.push_back(ego);
  for (std::size_t index = 0; index < reference.size(); ++index)
    if (lengths[index] > ego_projection.arc_m + 1.0e-6 &&
        lengths[index] < start_arc - 1.0e-6)
      path.push_back(reference[index]);
  road_start.longitudinal_velocity_mps = 0.0F;
  path.push_back(road_start);
  const std::size_t shift_start = path.size() - 1;
  const double travel_sign = parameters.forward ? sign : -sign;
  const double origin_x = parameters.forward ? 0.0 :
      -arc_forward_length;
  const double origin_y = parameters.forward ? 0.0 : lateral;
  Trajectory arcs;
  std::size_t first_arc_end = 0;
  for (int stage = 0; stage < 2; ++stage) {
    const double stage_radius = stage == 0 ?
        near_radius : far_radius;
    const int samples = std::max(2, static_cast<int>(std::ceil(
        stage_radius * turn_angle / parameters.arc_interval_m)));
    for (int index = stage == 0 ? 0 : 1; index <= samples; ++index) {
      const double angle = turn_angle * index / samples;
      double x = origin_x;
      double y = origin_y;
      double yaw = 0.0;
      if (stage == 0) {
        x += near_radius * std::sin(angle);
        y += travel_sign * near_radius * (1.0 - std::cos(angle));
        yaw = travel_sign * angle;
      } else {
        x += near_radius * std::sin(turn_angle) +
             far_radius * (std::sin(turn_angle) -
                           std::sin(turn_angle - angle));
        y += travel_sign * (
            near_radius * (1.0 - std::cos(turn_angle)) +
            far_radius * (std::cos(turn_angle - angle) -
                          std::cos(turn_angle)));
        yaw = travel_sign * (turn_angle - angle);
      }
      TrajectoryPoint point = road_start;
      point.x = road_start.x + x * cosine - y * sine;
      point.y = road_start.y + x * sine + y * cosine;
      point.orientation_z = std::sin((road_yaw + yaw) / 2.0);
      point.orientation_w = std::cos((road_yaw + yaw) / 2.0);
      point.longitudinal_velocity_mps = static_cast<float>(
          parameters.forward ? parameters.forward_velocity_mps :
              parameters.backward_velocity_mps);
      arcs.push_back(point);
    }
    if (stage == 0) first_arc_end = arcs.size() - 1;
  }
  if (!parameters.forward) {
    first_arc_end = arcs.size() - 1 - first_arc_end;
    std::reverse(arcs.begin(), arcs.end());
  }
  const auto first_arc_end_point = arcs[first_arc_end];
  for (const auto &point : arcs)
    if (std::hypot(path.back().x - point.x,
                   path.back().y - point.y) > 0.01)
      path.push_back(point);
  const std::size_t shift_end = path.size() - 1;
  const std::size_t first_arc_end_index = nearestIndex(
      path, first_arc_end_point);
  path[first_arc_end_index].longitudinal_velocity_mps = 0.0F;
  TrajectoryPoint goal_point = path.back();
  goal_point.x = goal.x;
  goal_point.y = goal.y;
  goal_point.orientation_z = std::sin(goal.yaw_rad / 2.0);
  goal_point.orientation_w = std::cos(goal.yaw_rad / 2.0);
  goal_point.longitudinal_velocity_mps = 0.0F;
  if (std::hypot(path.back().x - goal.x,
                 path.back().y - goal.y) > 0.01)
    path.push_back(goal_point);
  else path.back() = goal_point;
  const auto final_index = path.size() - 1;
  return ShiftPullOverCandidate{
      std::move(path), shift_start, shift_end, lateral, 0.0,
      {shift_start, first_arc_end_index, final_index}};
}

std::optional<ShiftPullOverCandidate> generateCurvedGeometricPullOverCandidate(
    const Trajectory &reference, const TrajectoryPoint &ego,
    BehaviorGoalPose goal,
    const GeometricPullOverParameters &parameters) {
  if (reference.size() < 3 || !std::isfinite(goal.x) ||
      !std::isfinite(goal.y) || !std::isfinite(goal.yaw_rad) ||
      !std::isfinite(parameters.wheelbase_m) ||
      !std::isfinite(parameters.maximum_steering_angle_rad) ||
      !std::isfinite(parameters.far_turn_radius_m) ||
      !std::isfinite(parameters.arc_interval_m) ||
      !std::isfinite(parameters.after_straight_distance_m) ||
      !std::isfinite(parameters.forward_velocity_mps) ||
      !std::isfinite(parameters.backward_velocity_mps) ||
      parameters.wheelbase_m <= 0.0 ||
      parameters.maximum_steering_angle_rad <= 0.0 ||
      parameters.maximum_steering_angle_rad >= std::acos(-1.0) / 2.0 ||
      parameters.far_turn_radius_m < 0.0 ||
      parameters.arc_interval_m <= 0.0 ||
      parameters.after_straight_distance_m < 0.0 ||
      parameters.forward_velocity_mps <= 0.0 ||
      parameters.backward_velocity_mps >= 0.0)
    throw std::invalid_argument("곡선 기하 주차 입력이 유효하지 않습니다");
  const double near_min = parameters.wheelbase_m /
      std::tan(parameters.maximum_steering_angle_rad);
  const double far_radius = parameters.far_turn_radius_m > 0.0 ?
      parameters.far_turn_radius_m : near_min;
  if (far_radius < near_min) return std::nullopt;
  const auto goal_projection = project(reference, goal.x, goal.y);
  const double lateral = goal_projection.lateral_m;
  if (std::abs(lateral) < 0.1) return std::nullopt;
  const double side = lateral > 0.0 ? 1.0 : -1.0;
  const double travel_direction = parameters.forward ? 1.0 : -1.0;
  const double first_direction = travel_direction * side;
  const double arc_end_x = goal.x - travel_direction *
      parameters.after_straight_distance_m * std::cos(goal.yaw_rad);
  const double arc_end_y = goal.y - travel_direction *
      parameters.after_straight_distance_m * std::sin(goal.yaw_rad);
  const double far_x = arc_end_x +
      side * far_radius * std::sin(goal.yaw_rad);
  const double far_y = arc_end_y -
      side * far_radius * std::cos(goal.yaw_rad);
  std::vector<double> lengths(reference.size(), 0.0);
  for (std::size_t index = 1; index < reference.size(); ++index)
    lengths[index] = lengths[index - 1] + std::hypot(
        reference[index].x - reference[index - 1].x,
        reference[index].y - reference[index - 1].y);
  const double ego_arc = project(reference, ego.x, ego.y).arc_m;
  const double end_arc = project(reference, arc_end_x, arc_end_y).arc_m;
  const auto turn = 2.0 * std::acos(-1.0);
  const auto rotation = [](double x, double y, double angle) {
    return std::pair<double, double>{
        x * std::cos(angle) - y * std::sin(angle),
        x * std::sin(angle) + y * std::cos(angle)};
  };
  const auto directed = [turn](double ax, double ay,
                               double bx, double by,
                               double direction) {
    double value = direction * std::atan2(
        ax * by - ay * bx, ax * bx + ay * by);
    if (value < 0.0) value += turn;
    return value;
  };
  double best_score = std::numeric_limits<double>::infinity();
  std::optional<ShiftPullOverCandidate> best;
  for (std::size_t segment = 0; segment + 1 < reference.size(); ++segment) {
    const auto &a = reference[segment];
    const auto &b = reference[segment + 1];
    const double segment_length = lengths[segment + 1] - lengths[segment];
    if (segment_length < 1.0e-6) continue;
    const double road_yaw = std::atan2(b.y - a.y, b.x - a.x);
    const int samples = std::max(1, static_cast<int>(std::ceil(
        segment_length / 0.5)));
    for (int sample = 0; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / samples;
      const double start_arc = lengths[segment] + ratio * segment_length;
      if (start_arc <= ego_arc + 1.0 ||
          (parameters.forward && start_arc >= end_arc - 1.0) ||
          (!parameters.forward && start_arc <= end_arc + 1.0))
        continue;
      const double start_x = a.x + ratio * (b.x - a.x);
      const double start_y = a.y + ratio * (b.y - a.y);
      const double dx = far_x - start_x;
      const double dy = far_y - start_y;
      const double normal_dot = -dx * std::sin(road_yaw) +
          dy * std::cos(road_yaw);
      const double denominator = 2.0 *
          (far_radius + side * normal_dot);
      if (denominator <= 1.0e-6) continue;
      const double near_radius =
          (dx * dx + dy * dy - far_radius * far_radius) /
          denominator;
      if (!std::isfinite(near_radius) ||
          near_radius < near_min || near_radius > 4.0 * far_radius)
        continue;
      const double near_x = start_x -
          side * near_radius * std::sin(road_yaw);
      const double near_y = start_y +
          side * near_radius * std::cos(road_yaw);
      const double ux = (far_x - near_x) / (near_radius + far_radius);
      const double uy = (far_y - near_y) / (near_radius + far_radius);
      if (std::abs(std::hypot(ux, uy) - 1.0) > 0.01) continue;
      const double start_rx = side * std::sin(road_yaw);
      const double start_ry = -side * std::cos(road_yaw);
      const double end_rx = -side * std::sin(goal.yaw_rad);
      const double end_ry = side * std::cos(goal.yaw_rad);
      const double first_angle = directed(
          start_rx, start_ry, ux, uy, first_direction);
      const double second_angle = directed(
          -ux, -uy, end_rx, end_ry, -first_direction);
      if (first_angle < 0.01 || second_angle < 0.01 ||
          first_angle > 1.3 || second_angle > 1.3)
        continue;
      const double score = near_radius - near_min +
          0.01 * (first_angle + second_angle);
      if (score >= best_score) continue;
      Trajectory path{ego};
      for (std::size_t index = 0; index < reference.size(); ++index)
        if (lengths[index] > ego_arc + 1.0e-6 &&
            lengths[index] < start_arc - 1.0e-6)
          path.push_back(reference[index]);
      TrajectoryPoint start = a;
      start.x = start_x;
      start.y = start_y;
      start.orientation_z = std::sin(road_yaw / 2.0);
      start.orientation_w = std::cos(road_yaw / 2.0);
      start.longitudinal_velocity_mps = 0.0F;
      path.push_back(start);
      const auto start_index = path.size() - 1;
      const int first_samples = std::max(2, static_cast<int>(std::ceil(
          near_radius * first_angle / parameters.arc_interval_m)));
      for (int index = 1; index <= first_samples; ++index) {
        const double angle = first_direction * first_angle * index /
            first_samples;
        const auto [rx, ry] = rotation(start_rx, start_ry, angle);
        TrajectoryPoint point = start;
        point.x = near_x + near_radius * rx;
        point.y = near_y + near_radius * ry;
        const double yaw = road_yaw + angle;
        point.orientation_z = std::sin(yaw / 2.0);
        point.orientation_w = std::cos(yaw / 2.0);
        point.longitudinal_velocity_mps = static_cast<float>(
            parameters.forward ? parameters.forward_velocity_mps :
                parameters.backward_velocity_mps);
        path.push_back(point);
      }
      const auto first_end = path.size() - 1;
      path.back().longitudinal_velocity_mps = 0.0F;
      const int second_samples = std::max(2, static_cast<int>(std::ceil(
          far_radius * second_angle / parameters.arc_interval_m)));
      for (int index = 1; index <= second_samples; ++index) {
        const double angle = -first_direction * second_angle * index /
            second_samples;
        const auto [rx, ry] = rotation(-ux, -uy, angle);
        TrajectoryPoint point = start;
        point.x = far_x + far_radius * rx;
        point.y = far_y + far_radius * ry;
        const double yaw = road_yaw + first_direction * first_angle +
            angle;
        point.orientation_z = std::sin(yaw / 2.0);
        point.orientation_w = std::cos(yaw / 2.0);
        point.longitudinal_velocity_mps = static_cast<float>(
            parameters.forward ? parameters.forward_velocity_mps :
                parameters.backward_velocity_mps);
        path.push_back(point);
      }
      const auto shift_end = path.size() - 1;
      if (std::hypot(path.back().x - arc_end_x,
                     path.back().y - arc_end_y) > 0.05 ||
          std::abs(std::remainder(
              heading(path.back()) - goal.yaw_rad, turn)) > 0.05)
        continue;
      const int straight_samples = std::max(1, static_cast<int>(std::ceil(
          parameters.after_straight_distance_m /
          parameters.arc_interval_m)));
      for (int index = 1; index <= straight_samples; ++index) {
        const double ratio = static_cast<double>(index) / straight_samples;
        TrajectoryPoint point = path.back();
        point.x = arc_end_x + ratio * (goal.x - arc_end_x);
        point.y = arc_end_y + ratio * (goal.y - arc_end_y);
        point.orientation_z = std::sin(goal.yaw_rad / 2.0);
        point.orientation_w = std::cos(goal.yaw_rad / 2.0);
        point.longitudinal_velocity_mps = index == straight_samples ?
            0.0F : static_cast<float>(parameters.forward ?
                parameters.forward_velocity_mps :
                parameters.backward_velocity_mps);
        path.push_back(point);
      }
      best_score = score;
      best = ShiftPullOverCandidate{
          std::move(path), start_index, shift_end, lateral, 0.0,
          {start_index, first_end, shift_end +
              static_cast<std::size_t>(straight_samples)}};
    }
  }
  return best;
}

}  // planning 네임스페이스
