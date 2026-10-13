// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// PathShifter의 점별 횡이동과 가속도 제한 곡선입니다.

#include "planning/behavior_path_shift.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace planning {
namespace {

struct ShiftKnots {
  std::vector<double> distance;
  std::vector<double> shift;
};

ShiftKnots simpleKnots(double distance, double shift, bool offset_back) {
  ShiftKnots knots{{0.0, distance / 4.0, 3.0 * distance / 4.0, distance},
                   {0.0, shift / 12.0, 11.0 * shift / 12.0, shift}};
  if (!offset_back) std::reverse(knots.shift.begin(), knots.shift.end());
  return knots;
}

ShiftKnots movingKnots(double distance, double shift, double velocity,
                       double acceleration, double total_time,
                       bool offset_back) {
  const double quarter = total_time / 4.0;
  const double first = std::min(
      velocity * quarter + 0.5 * acceleration * quarter * quarter,
      distance);
  const double next_velocity = velocity + acceleration * quarter;
  const double second = std::min(
      first + 2.0 * next_velocity * quarter +
          2.0 * acceleration * quarter * quarter, distance);
  ShiftKnots knots{{0.0, first, second, distance},
                   {0.0, shift / 12.0, 11.0 * shift / 12.0, shift}};
  if (!offset_back) std::reverse(knots.shift.begin(), knots.shift.end());
  return knots;
}

ShiftKnots makeKnots(double distance, double shift,
                     const BehaviorPathShiftParameters &parameters) {
  const double velocity = std::abs(parameters.velocity_mps);
  const double acceleration = parameters.longitudinal_acceleration_mps2 >
          0.0001 ? parameters.longitudinal_acceleration_mps2 : 0.0;
  if (velocity < 1.0e-5 && acceleration < 0.0001) {
    return simpleKnots(distance, shift, parameters.offset_back);
  }
  const double magnitude = std::abs(shift);
  const double total_time = acceleration > 0.0001
      ? (-velocity + std::sqrt(velocity * velocity +
          2.0 * acceleration * distance)) / acceleration
      : distance / velocity;
  const double lateral_max = 8.0 * magnitude /
      (total_time * total_time);
  if (lateral_max < parameters.lateral_acceleration_limit_mps2) {
    return movingKnots(distance, shift, velocity, acceleration,
                       total_time, parameters.offset_back);
  }
  const double lateral_limit = parameters.lateral_acceleration_limit_mps2;
  const double jerk_time = total_time / 2.0 -
      2.0 * magnitude / (lateral_limit * total_time);
  const double acceleration_time = 4.0 * magnitude /
      (lateral_limit * total_time) - total_time / 2.0;
  const double lateral_jerk = 2.0 * lateral_limit * lateral_limit *
      total_time / (lateral_limit * total_time * total_time -
                    4.0 * magnitude);
  if (jerk_time < 0.0 || acceleration_time < 0.0 ||
      lateral_jerk < 0.0 || jerk_time / total_time < 0.1) {
    return simpleKnots(distance, shift, parameters.offset_back);
  }

  const double tj = jerk_time;
  const double ta = acceleration_time;
  const double tj3 = tj * tj * tj;
  const double ta_tj2 = ta * tj * tj;
  const double ta2_tj = ta * ta * tj;
  const double first = std::min(
      tj * velocity + 0.5 * acceleration * tj * tj, distance);
  const double velocity1 = velocity + acceleration * tj;
  const double second = std::min(
      first + ta * velocity1 + 0.5 * acceleration * ta * ta,
      distance);
  const double velocity2 = velocity1 + acceleration * ta;
  const double third = std::min(
      second + tj * velocity2 + 0.5 * acceleration * tj * tj,
      distance);
  const double velocity3 = velocity2 + acceleration * tj;
  const double fifth = std::min(
      third + tj * velocity3 + 0.5 * acceleration * tj * tj,
      distance);
  const double velocity5 = velocity3 + acceleration * tj;
  const double sixth = std::min(
      fifth + ta * velocity5 + 0.5 * acceleration * ta * ta,
      distance);
  const double velocity6 = velocity5 + acceleration * ta;
  const double seventh = std::min(
      sixth + tj * velocity6 + 0.5 * acceleration * tj * tj,
      distance);
  const double sign = shift > 0.0 ? 1.0 : -1.0;
  ShiftKnots knots{
      {0.0, first, second, third, fifth, sixth, seventh},
      {0.0,
       sign * lateral_jerk * tj3 / 6.0,
       sign * lateral_jerk * (tj3 / 6.0 + ta_tj2 / 2.0 + ta2_tj / 2.0),
       sign * lateral_jerk * (tj3 + 1.5 * ta_tj2 + ta2_tj / 2.0),
       sign * lateral_jerk * (11.0 * tj3 / 6.0 +
                              2.5 * ta_tj2 + ta2_tj / 2.0),
       sign * lateral_jerk * (11.0 * tj3 / 6.0 +
                              3.0 * ta_tj2 + ta2_tj),
       sign * lateral_jerk * (2.0 * tj3 + 3.0 * ta_tj2 + ta2_tj)}};
  if (!parameters.offset_back) {
    std::reverse(knots.shift.begin(), knots.shift.end());
  }
  return knots;
}

double interpolateNaturalSpline(const ShiftKnots &knots, double query) {
  const std::size_t count = knots.distance.size();
  if (count < 2 || count != knots.shift.size()) {
    throw std::invalid_argument("횡이동 곡선의 기준점이 유효하지 않습니다");
  }
  std::vector<double> width(count - 1);
  for (std::size_t i = 0; i + 1 < count; ++i) {
    width[i] = knots.distance[i + 1] - knots.distance[i];
    if (!(width[i] > 1.0e-9)) {
      throw std::invalid_argument("횡이동 곡선의 기준점 간격이 유효하지 않습니다");
    }
  }
  std::vector<double> second(count, 0.0);
  if (count > 2) {
    std::vector<double> upper(count - 2, 0.0);
    std::vector<double> rhs(count - 2, 0.0);
    for (std::size_t i = 1; i + 1 < count; ++i) {
      const double lower = i > 1 ? width[i - 1] : 0.0;
      const double diagonal = 2.0 * (width[i - 1] + width[i]);
      const double denominator = diagonal - lower *
          (i > 1 ? upper[i - 2] : 0.0);
      upper[i - 1] = i + 2 < count ? width[i] / denominator : 0.0;
      const double slope_right =
          (knots.shift[i + 1] - knots.shift[i]) / width[i];
      const double slope_left =
          (knots.shift[i] - knots.shift[i - 1]) / width[i - 1];
      rhs[i - 1] = (6.0 * (slope_right - slope_left) -
          lower * (i > 1 ? rhs[i - 2] : 0.0)) / denominator;
    }
    for (std::size_t i = count - 2; i > 0; --i) {
      second[i] = rhs[i - 1] - upper[i - 1] * second[i + 1];
    }
  }
  const auto upper = std::lower_bound(
      knots.distance.begin(), knots.distance.end(), query);
  const std::size_t interval = std::clamp<std::size_t>(
      static_cast<std::size_t>(std::distance(knots.distance.begin(), upper)) -
          (upper == knots.distance.begin() ? 0 : 1),
      0, count - 2);
  const double delta = query - knots.distance[interval];
  const double a = (second[interval + 1] - second[interval]) /
      (6.0 * width[interval]);
  const double b = second[interval] / 2.0;
  const double c = (knots.shift[interval + 1] - knots.shift[interval]) /
      width[interval] - width[interval] *
      (2.0 * second[interval] + second[interval + 1]) / 6.0;
  return ((a * delta + b) * delta + c) * delta +
      knots.shift[interval];
}

void addShift(BehaviorPathShiftResult &result, std::size_t index,
              double amount) {
  if (std::abs(amount) < 1.0e-8) return;
  auto &point = result.path.at(index);
  const double yaw = 2.0 * std::atan2(
      point.orientation_z, point.orientation_w);
  point.x -= std::sin(yaw) * amount;
  point.y += std::cos(yaw) * amount;
  result.shift_lengths_m.at(index) += amount;
}

double lateralOffsetToPath(const Trajectory &path,
                           BehaviorPathShiftAnchor position) {
  double best_distance = std::numeric_limits<double>::infinity();
  double best_offset = 0.0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const double dx = path[index + 1].x - path[index].x;
    const double dy = path[index + 1].y - path[index].y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared < 1.0e-12) continue;
    const double ratio = std::clamp(
        ((position.x - path[index].x) * dx +
         (position.y - path[index].y) * dy) / length_squared,
        0.0, 1.0);
    const double px = path[index].x + ratio * dx;
    const double py = path[index].y + ratio * dy;
    const double distance = std::hypot(position.x - px, position.y - py);
    if (distance < best_distance) {
      best_distance = distance;
      best_offset = (dx * (position.y - py) -
                     dy * (position.x - px)) /
                    std::sqrt(length_squared);
    }
  }
  return best_offset;
}

}  // 익명 네임스페이스

BehaviorPathShiftLine behaviorShiftLineFromPositions(
    const Trajectory &reference, BehaviorPathShiftAnchor start,
    BehaviorPathShiftAnchor end, double end_shift_m) {
  if (reference.empty() || !std::isfinite(start.x) ||
      !std::isfinite(start.y) || !std::isfinite(end.x) ||
      !std::isfinite(end.y) || !std::isfinite(end_shift_m)) {
    throw std::invalid_argument("횡이동 위치 입력이 유효하지 않습니다");
  }
  const auto nearest = [&](BehaviorPathShiftAnchor anchor) {
    std::size_t best_index = 0;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < reference.size(); ++index) {
      const double distance = std::hypot(
          reference[index].x - anchor.x,
          reference[index].y - anchor.y);
      if (distance < best_distance) {
        best_index = index;
        best_distance = distance;
      }
    }
    return best_index;
  };
  return {nearest(start), nearest(end), end_shift_m};
}

BehaviorPathShiftResult shiftBehaviorPath(
    const Trajectory &reference,
    const std::vector<BehaviorPathShiftLine> &lines,
    const BehaviorPathShiftParameters &parameters) {
  if (reference.empty()) {
    throw std::invalid_argument("횡이동 기준 경로가 비어 있습니다");
  }
  if (!std::isfinite(parameters.base_offset_m) ||
      !std::isfinite(parameters.velocity_mps) ||
      !std::isfinite(parameters.longitudinal_acceleration_mps2) ||
      !std::isfinite(parameters.lateral_acceleration_limit_mps2) ||
      parameters.lateral_acceleration_limit_mps2 <= 0.0) {
    throw std::invalid_argument("횡이동 설정이 유효하지 않습니다");
  }
  std::vector<double> arcs(reference.size(), 0.0);
  for (std::size_t index = 1; index < reference.size(); ++index) {
    arcs[index] = arcs[index - 1] + std::hypot(
        reference[index].x - reference[index - 1].x,
        reference[index].y - reference[index - 1].y);
  }
  BehaviorPathShiftResult result{reference,
                                 std::vector<double>(reference.size(), 0.0)};
  if (std::abs(parameters.base_offset_m) > 1.0e-4) {
    for (std::size_t index = 0; index < reference.size(); ++index) {
      addShift(result, index, parameters.base_offset_m);
    }
  }
  auto sorted = lines;
  std::stable_sort(sorted.begin(), sorted.end(),
                   [](const auto &first, const auto &second) {
                     return first.start_index < second.start_index;
                   });
  for (const auto &line : sorted) {
    if (line.end_index >= reference.size() ||
        line.end_index <= line.start_index + 1 ||
        !std::isfinite(line.end_shift_m)) {
      throw std::invalid_argument("횡이동 구간이 유효하지 않습니다");
    }
    const double delta = line.end_shift_m -
        result.shift_lengths_m.at(line.end_index);
    const double length = std::max(
        arcs[line.end_index] - arcs[line.start_index], 1.0e-8);
    if (parameters.type == BehaviorPathShiftType::Linear) {
      for (std::size_t index = line.start_index;
           index < reference.size(); ++index) {
        const double amount = index <= line.end_index
            ? (arcs[index] - arcs[line.start_index]) / length * delta
            : delta;
        addShift(result, index, amount);
      }
      continue;
    }
    const auto knots = makeKnots(length, delta, parameters);
    for (std::size_t index = line.start_index + 1;
         index < line.end_index; ++index) {
      addShift(result, index, interpolateNaturalSpline(
          knots, arcs[index] - arcs[line.start_index]));
    }
    if (parameters.offset_back) {
      for (std::size_t index = line.end_index;
           index < reference.size(); ++index) addShift(result, index, delta);
    } else {
      const double before = interpolateNaturalSpline(
          knots, arcs[line.start_index + 1] - arcs[line.start_index]);
      for (std::size_t index = 0; index <= line.start_index; ++index) {
        addShift(result, index, before);
      }
    }
  }
  return result;
}

Trajectory finalizeBehaviorPathShift(const Trajectory &shifted_path) {
  constexpr double pi = 3.14159265358979323846;
  Trajectory result;
  result.reserve(shifted_path.size());
  for (const auto &point : shifted_path) {
    if (!result.empty() &&
        std::abs(point.x - result.back().x) < 1.0e-8 &&
        std::abs(point.y - result.back().y) < 1.0e-8) continue;
    result.push_back(point);
  }
  const auto remove_first_invalid = [&]() {
    for (std::size_t index = 0; index + 1 < result.size(); ++index) {
      const double first_yaw = 2.0 * std::atan2(
          result[index].orientation_z, result[index].orientation_w);
      const double second_yaw = 2.0 * std::atan2(
          result[index + 1].orientation_z,
          result[index + 1].orientation_w);
      const double direction_yaw = std::atan2(
          result[index + 1].y - result[index].y,
          result[index + 1].x - result[index].x);
      const double yaw_difference = std::remainder(
          first_yaw - second_yaw, 2.0 * pi);
      const double direction_difference = std::remainder(
          first_yaw - direction_yaw, 2.0 * pi);
      if (std::abs(yaw_difference) > pi / 2.0 ||
          std::abs(direction_difference) >= pi / 2.0) {
        result.erase(result.begin() + static_cast<std::ptrdiff_t>(index + 1));
        return;
      }
    }
  };
  remove_first_invalid();
  std::size_t previous_size = 0;
  do {
    previous_size = result.size();
    for (std::size_t index = 0; index + 1 < result.size(); ++index) {
      const double dx = result[index + 1].x - result[index].x;
      const double dy = result[index + 1].y - result[index].y;
      const double dz = result[index + 1].z - result[index].z;
      const double yaw = std::atan2(dy, dx);
      const double pitch = std::atan2(dz, std::hypot(dx, dy));
      result[index].orientation_x = -std::sin(pitch / 2.0) *
          std::sin(yaw / 2.0);
      result[index].orientation_y = std::sin(pitch / 2.0) *
          std::cos(yaw / 2.0);
      result[index].orientation_z = std::cos(pitch / 2.0) *
          std::sin(yaw / 2.0);
      result[index].orientation_w = std::cos(pitch / 2.0) *
          std::cos(yaw / 2.0);
    }
    if (result.size() >= 2) {
      result.back().orientation_x = result[result.size() - 2].orientation_x;
      result.back().orientation_y = result[result.size() - 2].orientation_y;
      result.back().orientation_z = result[result.size() - 2].orientation_z;
      result.back().orientation_w = result[result.size() - 2].orientation_w;
    }
    remove_first_invalid();
  } while (previous_size != result.size());
  return result;
}

Trajectory generateBehaviorShiftedPath(
    const Trajectory &reference,
    const std::vector<BehaviorPathShiftLine> &lines,
    const BehaviorPathShiftParameters &parameters) {
  return finalizeBehaviorPathShift(
      shiftBehaviorPath(reference, lines, parameters).path);
}

std::optional<BehaviorSideShiftResult> generateBehaviorSideShiftPath(
    const Trajectory &reference, std::size_t ego_index,
    double ego_speed_mps, double current_shift_m,
    double requested_shift_m,
    const BehaviorSideShiftParameters &parameters) {
  if (reference.size() < 3 || ego_index >= reference.size() ||
      !std::isfinite(ego_speed_mps) ||
      !std::isfinite(current_shift_m) ||
      !std::isfinite(requested_shift_m) ||
      !std::isfinite(parameters.minimum_start_distance_m) ||
      !std::isfinite(parameters.start_delay_s) ||
      !std::isfinite(parameters.lateral_jerk_mps3) ||
      !std::isfinite(parameters.minimum_shift_distance_m) ||
      !std::isfinite(parameters.minimum_shift_speed_mps) ||
      !std::isfinite(parameters.request_time_limit_s) ||
      parameters.minimum_start_distance_m < 0.0 ||
      parameters.start_delay_s < 0.0 ||
      parameters.lateral_jerk_mps3 <= 0.0 ||
      parameters.minimum_shift_distance_m < 0.0 ||
      parameters.minimum_shift_speed_mps < 0.0 ||
      parameters.request_time_limit_s < 0.0)
    return std::nullopt;
  const double speed = std::abs(ego_speed_mps);
  const double start_distance = std::max(
      parameters.minimum_start_distance_m,
      speed * parameters.start_delay_s);
  const double delta_shift = requested_shift_m - current_shift_m;
  const double shift_distance = std::max(
      parameters.minimum_shift_distance_m,
      4.0 * std::cbrt(0.5 * std::abs(delta_shift) /
                      parameters.lateral_jerk_mps3) *
          std::max(speed, parameters.minimum_shift_speed_mps));
  const auto index_at_distance = [&](double distance) {
    std::size_t index = ego_index;
    double traveled = 0.0;
    while (index + 1 < reference.size() && traveled < distance) {
      traveled += std::hypot(
          reference[index + 1].x - reference[index].x,
          reference[index + 1].y - reference[index].y);
      ++index;
    }
    return index;
  };
  const BehaviorPathShiftLine line{
      index_at_distance(start_distance),
      index_at_distance(start_distance + shift_distance),
      requested_shift_m};
  if (line.end_index <= line.start_index + 1)
    return std::nullopt;
  BehaviorPathShiftParameters settings;
  settings.base_offset_m = current_shift_m;
  settings.velocity_mps = std::max(
      speed, parameters.minimum_shift_speed_mps);
  return BehaviorSideShiftResult{
      generateBehaviorShiftedPath(reference, {line}, settings), line};
}

void trimBehaviorPathShiftLines(
    std::vector<BehaviorPathShiftLine> &lines, std::size_t nearest_index,
    double &base_offset_m) {
  std::optional<BehaviorPathShiftLine> last_removed;
  lines.erase(std::remove_if(lines.begin(), lines.end(),
      [&](const auto &line) {
        if (line.end_index > nearest_index) return false;
        // 원본의 비교자 방향 때문에 제거된 구간 중 가장 작은 끝 인덱스를 고릅니다.
        if (!last_removed || line.end_index < last_removed->end_index) {
          last_removed = line;
        }
        return true;
      }), lines.end());
  if (last_removed) base_offset_m = last_removed->end_shift_m;
  if (std::abs(base_offset_m) < 1.0e-4) base_offset_m = 0.0;
}

double requiredStartPullOutDistance(
    double longitudinal_acceleration_mps2, double shift_time_s,
    double shift_length_m, double maximum_curvature,
    double minimum_distance_m) {
  if (!std::isfinite(longitudinal_acceleration_mps2) ||
      !std::isfinite(shift_time_s) || !std::isfinite(shift_length_m) ||
      !std::isfinite(maximum_curvature) || maximum_curvature <= 0.0 ||
      !std::isfinite(minimum_distance_m) || minimum_distance_m < 0.0) {
    throw std::invalid_argument("출발 횡이동 거리 설정이 유효하지 않습니다");
  }
  const double acceleration_distance =
      longitudinal_acceleration_mps2 * shift_time_s * shift_time_s / 2.0;
  const double curvature_distance = std::sqrt(std::max(
      4.0 * shift_length_m / maximum_curvature -
          shift_length_m * shift_length_m, 0.0));
  return std::max({acceleration_distance, curvature_distance,
                   minimum_distance_m});
}

double startPullOutShiftTime(
    double lateral_shift_m, double lateral_jerk_mps3,
    double lateral_acceleration_mps2) {
  if (!std::isfinite(lateral_shift_m) ||
      !std::isfinite(lateral_jerk_mps3) ||
      !std::isfinite(lateral_acceleration_mps2)) {
    throw std::invalid_argument("출발 횡이동 시간 입력이 유효하지 않습니다");
  }
  const double jerk = std::abs(lateral_jerk_mps3);
  const double acceleration = std::abs(lateral_acceleration_mps2);
  const double distance = std::abs(lateral_shift_m);
  if (jerk < 1.0e-8 || acceleration < 1.0e-8) return 1.0e10;
  double jerk_time = acceleration / jerk;
  double acceleration_time = (
      std::sqrt(acceleration * acceleration +
                4.0 * jerk * jerk * distance / acceleration) -
      3.0 * acceleration) / (2.0 * jerk);
  if (acceleration_time < 0.0) {
    jerk_time = std::cbrt(distance / (2.0 * jerk));
    acceleration_time = 0.0;
  }
  return 4.0 * jerk_time + 2.0 * acceleration_time;
}

double startPullOutDistanceBeforeShift(
    const Trajectory &centerline, double after_shift_arc_m,
    double lateral_shift_m) {
  if (centerline.size() < 2 || !std::isfinite(after_shift_arc_m) ||
      after_shift_arc_m < 0.0 || !std::isfinite(lateral_shift_m)) {
    throw std::invalid_argument("출발 호길이 입력이 유효하지 않습니다");
  }
  double before_arc = 0.0;
  double after_arc = 0.0;
  for (std::size_t index = 1; index < centerline.size(); ++index) {
    const auto &first = centerline[index - 1];
    const auto &second = centerline[index];
    const double segment_length = std::hypot(
        second.x - first.x, second.y - first.y);
    double curvature = 0.0;
    if (index + 1 < centerline.size()) {
      const auto &third = centerline[index + 1];
      const double next_length = std::hypot(
          third.x - second.x, third.y - second.y);
      const double chord_length = std::hypot(
          third.x - first.x, third.y - first.y);
      const double denominator = segment_length * next_length *
          chord_length;
      if (denominator < 1.0e-10) {
        throw std::invalid_argument("출발 경로의 곡률 기준점이 겹칩니다");
      }
      curvature = 2.0 * (
          (second.x - first.x) * (third.y - first.y) -
          (second.y - first.y) * (third.x - first.x)) / denominator;
    }
    const double factor = curvature < 0.0
        ? 1.0 - curvature * lateral_shift_m
        : 1.0 + curvature * lateral_shift_m;
    if (factor <= 0.0) {
      throw std::invalid_argument("출발 횡이동의 곡률 변환이 유효하지 않습니다");
    }
    const double after_segment = curvature < 0.0
        ? segment_length * factor
        : segment_length / factor;
    if (after_arc + after_segment > after_shift_arc_m) {
      const double remainder = after_shift_arc_m - after_arc;
      before_arc += curvature < 0.0
          ? remainder / factor
          : remainder * factor;
      break;
    }
    before_arc += segment_length;
    after_arc += after_segment;
  }
  return before_arc;
}

std::vector<StartPullOutCandidate> generateStartPullOutCandidates(
    const Trajectory &centerline, BehaviorPathShiftAnchor start,
    double start_lateral_offset_m, bool path_terminal_is_goal,
    const StartPullOutParameters &parameters) {
  if (centerline.size() < 3 || !std::isfinite(start.x) ||
      !std::isfinite(start.y) || !std::isfinite(start_lateral_offset_m) ||
      !std::isfinite(parameters.lateral_jerk_mps3) ||
      parameters.lateral_jerk_mps3 == 0.0 ||
      !std::isfinite(parameters.minimum_lateral_acceleration_mps2) ||
      !std::isfinite(parameters.maximum_lateral_acceleration_mps2) ||
      parameters.lateral_acceleration_samples <= 0 ||
      parameters.minimum_lateral_acceleration_mps2 <= 0.0 ||
      parameters.maximum_lateral_acceleration_mps2 <
          parameters.minimum_lateral_acceleration_mps2 ||
      !std::isfinite(parameters.maximum_curvature) ||
      parameters.maximum_curvature <= 0.0 ||
      !std::isfinite(parameters.end_curvature_threshold) ||
      parameters.end_curvature_threshold < 0.0 ||
      !std::isfinite(parameters.minimum_distance_m) ||
      parameters.minimum_distance_m < 0.0 ||
      !std::isfinite(parameters.centerline_interval_m) ||
      parameters.centerline_interval_m <= 0.0) {
    throw std::invalid_argument("출발 횡이동 후보 입력이 유효하지 않습니다");
  }
  const auto start_index = behaviorShiftLineFromPositions(
      centerline, start, start, 0.0).start_index;
  if (start_index + 2 >= centerline.size()) return {};
  const auto no_shift = [&]() {
    StartPullOutCandidate candidate;
    candidate.path = centerline;
    candidate.shift_end_index = start_index;
    return candidate;
  };
  std::vector<StartPullOutCandidate> candidates;
  bool has_no_shift = false;
  if (std::abs(start_lateral_offset_m) < 0.01) {
    candidates.push_back(no_shift());
    has_no_shift = true;
  }
  const double road_velocity =
      centerline[start_index].longitudinal_velocity_mps;
  const Trajectory from_start(centerline.begin() +
      static_cast<std::ptrdiff_t>(start_index), centerline.end());
  const double resolution = std::max(
      (parameters.maximum_lateral_acceleration_mps2 -
       parameters.minimum_lateral_acceleration_mps2) /
          parameters.lateral_acceleration_samples,
      std::numeric_limits<double>::epsilon());
  const int sample_count =
      parameters.maximum_lateral_acceleration_mps2 ==
              parameters.minimum_lateral_acceleration_mps2
          ? 0 : parameters.lateral_acceleration_samples;
  for (int sample = 0; sample <= sample_count; ++sample) {
    const double lateral_acceleration =
        parameters.minimum_lateral_acceleration_mps2 +
        resolution * sample;
    if (lateral_acceleration >
        parameters.maximum_lateral_acceleration_mps2 + 1.0e-12) break;
    const double shift_time = startPullOutShiftTime(
        start_lateral_offset_m, parameters.lateral_jerk_mps3,
        lateral_acceleration);
    if (shift_time <= 0.0 || !std::isfinite(shift_time)) continue;
    const double longitudinal_acceleration = std::clamp(
        road_velocity / shift_time, 0.0, 1.0);
    const double distance = requiredStartPullOutDistance(
        longitudinal_acceleration, shift_time, start_lateral_offset_m,
        parameters.maximum_curvature, parameters.minimum_distance_m);
    const double converted_distance = std::max(
        distance, startPullOutDistanceBeforeShift(
            from_start, distance, start_lateral_offset_m));
    double arc = 0.0;
    double selected_distance = converted_distance;
    double minimum_curvature = std::numeric_limits<double>::infinity();
    for (std::size_t index = 1; index + 1 < from_start.size(); ++index) {
      const auto &previous = from_start[index - 1];
      const auto &current = from_start[index];
      const auto &next = from_start[index + 1];
      const double first = std::hypot(
          current.x - previous.x, current.y - previous.y);
      const double second = std::hypot(
          next.x - current.x, next.y - current.y);
      const double chord = std::hypot(
          next.x - previous.x, next.y - previous.y);
      const double cross = (current.x - previous.x) *
          (next.y - current.y) - (current.y - previous.y) *
          (next.x - current.x);
      const double curvature = first * second * chord > 1.0e-9
          ? std::abs(2.0 * cross / (first * second * chord)) : 0.0;
      arc += first;
      if (index + 2 == from_start.size()) arc += second;
      if (arc > converted_distance) {
        if (curvature < minimum_curvature) {
          minimum_curvature = curvature;
          selected_distance = arc;
        }
        if (curvature < parameters.end_curvature_threshold) break;
      }
    }
    if (selected_distance < parameters.centerline_interval_m) {
      if (!has_no_shift) {
        candidates.push_back(no_shift());
        has_no_shift = true;
      }
      continue;
    }
    double traversed = 0.0;
    std::size_t end_index = start_index;
    for (std::size_t index = start_index + 1;
         index < centerline.size(); ++index) {
      traversed += std::hypot(
          centerline[index].x - centerline[index - 1].x,
          centerline[index].y - centerline[index - 1].y);
      end_index = index;
      if (traversed >= selected_distance) break;
    }
    if (traversed + parameters.centerline_interval_m <
        selected_distance) continue;
    if (end_index <= start_index + 1) {
      if (!has_no_shift) {
        candidates.push_back(no_shift());
        has_no_shift = true;
      }
      continue;
    }
    BehaviorPathShiftParameters shift_settings;
    shift_settings.offset_back = false;
    shift_settings.velocity_mps = 1.0;
    shift_settings.longitudinal_acceleration_mps2 =
        longitudinal_acceleration;
    shift_settings.lateral_acceleration_limit_mps2 = lateral_acceleration;
    const auto line = BehaviorPathShiftLine{
        start_index, end_index, start_lateral_offset_m};
    Trajectory shifted;
    try {
      shifted = generateBehaviorShiftedPath(
          centerline, {line}, shift_settings);
    } catch (const std::invalid_argument &) {
      continue;
    }
    if (shifted.empty()) continue;
    const BehaviorPathShiftAnchor end{
        centerline[end_index].x, centerline[end_index].y};
    for (std::size_t iteration = 0; iteration < 100; ++iteration) {
      const double previous_offset = lateralOffsetToPath(shifted, start);
      const auto refinement = behaviorShiftLineFromPositions(
          shifted, start, end, previous_offset);
      if (refinement.end_index <= refinement.start_index + 1) break;
      shift_settings.velocity_mps = 0.0;
      try {
        shifted = generateBehaviorShiftedPath(
            shifted, {refinement}, shift_settings);
      } catch (const std::invalid_argument &) {
        break;
      }
      if (std::abs(previous_offset -
                   lateralOffsetToPath(shifted, start)) < 0.01) break;
    }
    if (shifted.empty()) continue;
    const double terminal_velocity = longitudinal_acceleration * shift_time;
    const std::size_t terminal_index = behaviorShiftLineFromPositions(
        shifted, end, end, 0.0).start_index;
    for (std::size_t index = 0; index < terminal_index; ++index) {
      shifted[index].longitudinal_velocity_mps = std::min(
          shifted[index].longitudinal_velocity_mps,
          static_cast<float>(terminal_velocity));
    }
    if (path_terminal_is_goal) {
      shifted.back().longitudinal_velocity_mps = 0.0F;
    }
    candidates.push_back({std::move(shifted), terminal_index,
                          terminal_velocity, longitudinal_acceleration,
                          lateral_acceleration, true});
  }
  return candidates;
}

std::optional<LaneChangeShiftCandidate> generateLaneChangeShiftCandidate(
    const Trajectory &prepare_path, const Trajectory &target_reference,
    BehaviorPathShiftAnchor change_start,
    BehaviorPathShiftAnchor change_end, double lateral_shift_m,
    double initial_velocity_mps, double longitudinal_acceleration_mps2,
    double lateral_acceleration_mps2, double terminal_velocity_mps) {
  if (prepare_path.size() < 2 || target_reference.size() < 3 ||
      !std::isfinite(lateral_shift_m) ||
      !std::isfinite(initial_velocity_mps) ||
      !std::isfinite(longitudinal_acceleration_mps2) ||
      !std::isfinite(lateral_acceleration_mps2) ||
      lateral_acceleration_mps2 == 0.0 ||
      !std::isfinite(terminal_velocity_mps)) {
    throw std::invalid_argument("차로 변경 횡이동 후보 입력이 유효하지 않습니다");
  }
  const auto line = behaviorShiftLineFromPositions(
      target_reference, change_start, change_end, lateral_shift_m);
  if (line.end_index <= line.start_index + 1) return std::nullopt;
  BehaviorPathShiftParameters settings;
  settings.offset_back = false;
  settings.velocity_mps = initial_velocity_mps;
  settings.longitudinal_acceleration_mps2 =
      longitudinal_acceleration_mps2;
  settings.lateral_acceleration_limit_mps2 =
      std::abs(lateral_acceleration_mps2);
  Trajectory shifted;
  try {
    shifted = generateBehaviorShiftedPath(
        target_reference, {line}, settings);
  } catch (const std::invalid_argument &) {
    return std::nullopt;
  }
  if (shifted.size() < line.end_index + 1 || shifted.size() < 2) {
    return std::nullopt;
  }
  const std::size_t end_index = behaviorShiftLineFromPositions(
      shifted, change_end, change_end, 0.0).start_index;
  for (std::size_t index = 0; index < end_index; ++index) {
    shifted[index].longitudinal_velocity_mps = std::min(
        shifted[index].longitudinal_velocity_mps,
        static_cast<float>(terminal_velocity_mps));
  }
  const double prepare_yaw = 2.0 * std::atan2(
      prepare_path[prepare_path.size() - 2].orientation_z,
      prepare_path[prepare_path.size() - 2].orientation_w);
  const double shifted_yaw = 2.0 * std::atan2(
      shifted[1].orientation_z, shifted[1].orientation_w);
  constexpr double pi = 3.14159265358979323846;
  if (std::abs(std::remainder(
          prepare_yaw - shifted_yaw, 2.0 * pi)) >
      5.0 * pi / 180.0) return std::nullopt;
  Trajectory joined = prepare_path;
  joined.insert(joined.end(), shifted.begin() + 1, shifted.end());
  Trajectory clean;
  clean.reserve(joined.size());
  for (const auto &point : joined) {
    if (!clean.empty() &&
        std::hypot(point.x - clean.back().x,
                   point.y - clean.back().y) < 1.0e-8) continue;
    clean.push_back(point);
  }
  return LaneChangeShiftCandidate{
      std::move(clean), std::move(shifted), end_index};
}

std::vector<double> sampleLaneChangeAccelerations(
    double minimum_mps2, double maximum_mps2, int intervals) {
  if (!std::isfinite(minimum_mps2) || !std::isfinite(maximum_mps2) ||
      intervals <= 0) {
    throw std::invalid_argument("차로 변경 가속도 표본 입력이 유효하지 않습니다");
  }
  if (minimum_mps2 > maximum_mps2) return {};
  if (maximum_mps2 - minimum_mps2 < 1.0e-8) return {minimum_mps2};
  std::vector<double> values{minimum_mps2};
  const double interval = (maximum_mps2 - minimum_mps2) / intervals;
  for (int index = 1; index <= intervals; ++index) {
    const double value = index == intervals
        ? maximum_mps2 : minimum_mps2 + index * interval;
    if (values.back() < -1.0e-8 && value > 1.0e-8) {
      values.push_back(0.0);
    }
    values.push_back(value);
  }
  std::reverse(values.begin(), values.end());
  return values;
}

std::optional<LaneChangeTargetReferenceArcRange>
laneChangeTargetReferenceArcRange(
    double change_start_arc_m, double change_length_m,
    double forward_path_length_m, double target_lane_length_m,
    double next_change_buffer_m, std::optional<double> goal_arc_m) {
  const double values[] = {
      change_start_arc_m, change_length_m, forward_path_length_m,
      target_lane_length_m, next_change_buffer_m};
  for (const double value : values) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("목표 차로 중심선 구간 입력이 유효하지 않습니다");
    }
  }
  if (goal_arc_m && (!std::isfinite(*goal_arc_m) || *goal_arc_m < 0.0)) {
    throw std::invalid_argument("목표 차로 목표점 호길이가 유효하지 않습니다");
  }
  const double limit_m = goal_arc_m.value_or(target_lane_length_m) -
                         next_change_buffer_m;
  const double end_m = std::min(
      change_start_arc_m + change_length_m + forward_path_length_m,
      limit_m);
  if (end_m - change_start_arc_m + 1.0e-4 < change_length_m) {
    return std::nullopt;
  }
  return LaneChangeTargetReferenceArcRange{change_start_arc_m, end_m};
}

double laneChangeTargetReferenceInterval(
    double change_length_m, double change_velocity_mps) {
  if (!std::isfinite(change_length_m) || change_length_m < 0.0 ||
      !std::isfinite(change_velocity_mps) || change_velocity_mps < 0.0) {
    throw std::invalid_argument("목표 차로 재표본화 입력이 유효하지 않습니다");
  }
  return std::max(change_length_m / 30.0, change_velocity_mps * 0.2);
}

double laneChangeActualPrepareDuration(
    double current_velocity_mps, double minimum_change_velocity_mps,
    double maximum_longitudinal_acceleration_mps2,
    double minimum_prepare_duration_s, double maximum_prepare_duration_s,
    double active_signal_duration_s) {
  const double values[] = {
      current_velocity_mps, minimum_change_velocity_mps,
      maximum_longitudinal_acceleration_mps2,
      minimum_prepare_duration_s, maximum_prepare_duration_s,
      active_signal_duration_s};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("차로 변경 준비 시간 입력이 유효하지 않습니다");
    }
  }
  if (minimum_prepare_duration_s < 0.0 ||
      maximum_prepare_duration_s < minimum_prepare_duration_s ||
      maximum_longitudinal_acceleration_mps2 < 0.0 ||
      active_signal_duration_s < 0.0) {
    throw std::invalid_argument("차로 변경 준비 시간 범위가 유효하지 않습니다");
  }
  double minimum = minimum_prepare_duration_s;
  if (current_velocity_mps < minimum_change_velocity_mps) {
    minimum = maximum_longitudinal_acceleration_mps2 < 1.0e-8
        ? maximum_prepare_duration_s
        : std::max((minimum_change_velocity_mps - current_velocity_mps) /
                       maximum_longitudinal_acceleration_mps2,
                   minimum_prepare_duration_s);
  }
  return std::max(maximum_prepare_duration_s - active_signal_duration_s,
                  minimum);
}

std::vector<double> laneChangePrepareDurations(
    double distance_to_terminal_start_m,
    double front_offset_m, double signal_activation_length_m,
    double nominal_prepare_duration_s, double maximum_prepare_duration_s) {
  const double values[] = {
      distance_to_terminal_start_m, front_offset_m,
      signal_activation_length_m, nominal_prepare_duration_s,
      maximum_prepare_duration_s};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("차로 변경 준비 표본 입력이 유효하지 않습니다");
    }
  }
  if (front_offset_m < 0.0 || signal_activation_length_m < 0.0 ||
      nominal_prepare_duration_s < 0.0 ||
      maximum_prepare_duration_s < 0.0) {
    throw std::invalid_argument("차로 변경 준비 표본 범위가 유효하지 않습니다");
  }
  if (distance_to_terminal_start_m >=
      front_offset_m + signal_activation_length_m) {
    return {nominal_prepare_duration_s};
  }
  std::vector<double> durations;
  for (double duration = maximum_prepare_duration_s;
       duration >= 0.0; duration -= 0.5) {
    durations.push_back(duration);
  }
  return durations;
}

bool laneChangeNeedsAccelerationSampling(
    double maximum_acceleration_mps2, bool ego_stuck,
    double maximum_current_buffer_m, double distance_to_terminal_end_m,
    double distance_to_target_end_m) {
  const double values[] = {
      maximum_acceleration_mps2, maximum_current_buffer_m,
      distance_to_terminal_end_m, distance_to_target_end_m};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("차로 변경 가속도 표본 조건이 유효하지 않습니다");
    }
  }
  return maximum_acceleration_mps2 < 0.0 || ego_stuck ||
      maximum_current_buffer_m > distance_to_terminal_end_m ||
      maximum_current_buffer_m >= distance_to_target_end_m;
}

std::vector<LaneChangePhaseMetrics> generateLaneChangePreparePhases(
    double current_velocity_mps, double maximum_path_velocity_mps,
    double maximum_vehicle_velocity_mps, double minimum_change_velocity_mps,
    double prepare_duration_s, double minimum_acceleration_mps2,
    double maximum_acceleration_mps2, int acceleration_intervals,
    bool sample_acceleration, double minimum_length_m,
    double maximum_length_m) {
  const double values[] = {
      current_velocity_mps, maximum_path_velocity_mps,
      maximum_vehicle_velocity_mps, minimum_change_velocity_mps,
      prepare_duration_s, minimum_acceleration_mps2,
      maximum_acceleration_mps2, minimum_length_m, maximum_length_m};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("차로 변경 준비 구간 입력이 유효하지 않습니다");
    }
  }
  if (prepare_duration_s < 0.0 || acceleration_intervals <= 0 ||
      maximum_vehicle_velocity_mps < minimum_change_velocity_mps ||
      minimum_length_m > maximum_length_m ||
      minimum_acceleration_mps2 >= 0.0 ||
      maximum_acceleration_mps2 < 0.0) {
    throw std::invalid_argument("차로 변경 준비 구간 범위가 유효하지 않습니다");
  }
  constexpr double epsilon = 1.0e-8;
  const double minimum_acceleration = prepare_duration_s < epsilon
      ? -std::abs(minimum_acceleration_mps2)
      : std::clamp((minimum_change_velocity_mps - current_velocity_mps) /
                       prepare_duration_s,
                   -std::abs(minimum_acceleration_mps2), -epsilon);
  const double maximum_acceleration = prepare_duration_s < epsilon
      ? maximum_acceleration_mps2
      : std::clamp((maximum_path_velocity_mps - current_velocity_mps) /
                       prepare_duration_s,
                   0.0, maximum_acceleration_mps2);
  const auto samples = sample_acceleration
      ? sampleLaneChangeAccelerations(minimum_acceleration,
                                      maximum_acceleration,
                                      acceleration_intervals)
      : std::vector<double>{maximum_acceleration};
  std::vector<LaneChangePhaseMetrics> phases;
  for (const double sampled_acceleration : samples) {
    const double terminal_velocity = std::clamp(
        current_velocity_mps + sampled_acceleration * prepare_duration_s,
        minimum_change_velocity_mps, maximum_vehicle_velocity_mps);
    const double actual_acceleration = prepare_duration_s < 1.0e-3
        ? 0.0 : (terminal_velocity - current_velocity_mps) /
                    prepare_duration_s;
    const double length = std::min(
        current_velocity_mps * prepare_duration_s +
            0.5 * actual_acceleration * prepare_duration_s *
                prepare_duration_s,
        maximum_vehicle_velocity_mps * prepare_duration_s);
    if (length < minimum_length_m || length > maximum_length_m) continue;
    phases.push_back({prepare_duration_s, length, terminal_velocity,
                      sampled_acceleration, actual_acceleration, 0.0});
  }
  return phases;
}

std::vector<LaneChangePhaseMetrics> generateLaneChangeShiftPhases(
    double lateral_shift_m, double initial_velocity_mps,
    double maximum_path_velocity_mps, double maximum_vehicle_velocity_mps,
    double minimum_change_velocity_mps,
    double prepare_acceleration_mps2, double minimum_lateral_acceleration_mps2,
    double maximum_lateral_acceleration_mps2, int lateral_intervals,
    double lateral_jerk_mps3, double maximum_length_m,
    bool near_current_terminal_start, double deceleration_factor) {
  const double values[] = {
      lateral_shift_m, initial_velocity_mps, maximum_path_velocity_mps,
      maximum_vehicle_velocity_mps, minimum_change_velocity_mps,
      prepare_acceleration_mps2, minimum_lateral_acceleration_mps2,
      maximum_lateral_acceleration_mps2, lateral_jerk_mps3,
      maximum_length_m, deceleration_factor};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("차로 변경 횡이동 구간 입력이 유효하지 않습니다");
    }
  }
  if (lateral_intervals <= 0 || lateral_jerk_mps3 <= 0.0 ||
      minimum_lateral_acceleration_mps2 <= 0.0 ||
      maximum_lateral_acceleration_mps2 <
          minimum_lateral_acceleration_mps2 ||
      maximum_vehicle_velocity_mps < minimum_change_velocity_mps) {
    throw std::invalid_argument("차로 변경 횡이동 구간 범위가 유효하지 않습니다");
  }
  std::vector<LaneChangePhaseMetrics> phases;
  const int last_index = maximum_lateral_acceleration_mps2 ==
          minimum_lateral_acceleration_mps2 ? 0 : lateral_intervals;
  for (int index = 0; index <= last_index; ++index) {
    const double lateral_acceleration =
        minimum_lateral_acceleration_mps2 +
        (maximum_lateral_acceleration_mps2 -
         minimum_lateral_acceleration_mps2) * index / lateral_intervals;
    const double duration = startPullOutShiftTime(
        lateral_shift_m, lateral_jerk_mps3, lateral_acceleration);
    if (duration <= 0.0 || !std::isfinite(duration)) continue;
    const double actual_acceleration = prepare_acceleration_mps2 <= 0.0
        ? (near_current_terminal_start
               ? prepare_acceleration_mps2 * deceleration_factor : 0.0)
        : std::clamp((maximum_path_velocity_mps - initial_velocity_mps) /
                         duration,
                     0.0, prepare_acceleration_mps2);
    const double length = std::min(
        initial_velocity_mps * duration +
            0.5 * actual_acceleration * duration * duration,
        maximum_vehicle_velocity_mps * duration);
    if (length > maximum_length_m) continue;
    const double terminal_velocity = std::clamp(
        initial_velocity_mps + actual_acceleration * duration,
        minimum_change_velocity_mps, maximum_vehicle_velocity_mps);
    phases.push_back({duration, length, terminal_velocity,
                      prepare_acceleration_mps2, actual_acceleration,
                      lateral_acceleration});
  }
  return phases;
}

LaneChangeCandidateLengthGate::LaneChangeCandidateLengthGate(
    double prepare_threshold_m, double shift_threshold_m)
    : prepare_threshold_m_(prepare_threshold_m),
      shift_threshold_m_(shift_threshold_m) {
  if (!std::isfinite(prepare_threshold_m_) ||
      !std::isfinite(shift_threshold_m_) ||
      prepare_threshold_m_ < 0.0 || shift_threshold_m_ < 0.0) {
    throw std::invalid_argument("차로 변경 후보 길이 비교값이 유효하지 않습니다");
  }
}

bool LaneChangeCandidateLengthGate::acceptsPrepare(
    double prepare_length_m) const {
  if (!std::isfinite(prepare_length_m) || prepare_length_m < 0.0) {
    throw std::invalid_argument("차로 변경 준비 길이가 유효하지 않습니다");
  }
  return !last_constructed_lengths_m_ ||
      std::abs(last_constructed_lengths_m_->first - prepare_length_m) >
          prepare_threshold_m_;
}

bool LaneChangeCandidateLengthGate::acceptsShift(
    double prepare_length_m, double shift_length_m) const {
  if (!std::isfinite(prepare_length_m) || prepare_length_m < 0.0 ||
      !std::isfinite(shift_length_m) || shift_length_m < 0.0) {
    throw std::invalid_argument("차로 변경 구간 길이가 유효하지 않습니다");
  }
  return !last_constructed_lengths_m_ ||
      std::abs(last_constructed_lengths_m_->first - prepare_length_m) >
          prepare_threshold_m_ ||
      std::abs(last_constructed_lengths_m_->second - shift_length_m) >
          shift_threshold_m_;
}

void LaneChangeCandidateLengthGate::recordConstructed(
    double prepare_length_m, double shift_length_m) {
  if (!std::isfinite(prepare_length_m) || prepare_length_m < 0.0 ||
      !std::isfinite(shift_length_m) || shift_length_m < 0.0) {
    throw std::invalid_argument("생성한 차로 변경 후보 길이가 유효하지 않습니다");
  }
  last_constructed_lengths_m_ =
      std::pair<double, double>{prepare_length_m, shift_length_m};
}

}  // planning 네임스페이스
