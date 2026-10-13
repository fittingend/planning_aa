// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// VelocitySmootherNode::overwriteStopPoint의 계산 부분을 이관했습니다.

#include "planning/jerk_node_policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {
double yaw(const TrajectoryPoint &point) {
  return std::atan2(
      2.0 * (point.orientation_w * point.orientation_z +
             point.orientation_x * point.orientation_y),
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z));
}

JerkMotion previousMotionAt(
    const Trajectory &previous, const TrajectoryPoint &point,
    double nearest_distance, double nearest_yaw) {
  if (previous.empty()) {
    throw std::invalid_argument("이전 출력 궤적이 비어 있습니다");
  }
  if (previous.size() == 1) {
    return {std::fabs(previous.front().longitudinal_velocity_mps),
            previous.front().acceleration_mps2};
  }
  // VelocitySmootherNode::insertBehindVelocity first searches previous path
  // points with distance and yaw constraints, retries distance only, then
  // falls back to the nearest position. The segment is chosen from that
  // point. A global nearest-segment search can jump to the opposite branch
  // when a path doubles back over itself.
  const auto point_yaw = yaw(point);
  const auto select_index = [&](bool check_distance, bool check_yaw)
      -> std::optional<std::size_t> {
    std::optional<std::size_t> selected;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < previous.size(); ++i) {
      const double dx = previous[i].x - point.x;
      const double dy = previous[i].y - point.y;
      const double distance2 = dx * dx + dy * dy;
      if (distance2 >= best ||
          (check_distance && distance2 > nearest_distance * nearest_distance) ||
          (check_yaw && std::fabs(std::remainder(
              yaw(previous[i]) - point_yaw,
              2.0 * 3.141592653589793)) > nearest_yaw)) continue;
      selected = i;
      best = distance2;
    }
    return selected;
  };
  const auto nearest = select_index(true, true)
      .value_or(select_index(true, false)
          .value_or(*select_index(false, false)));
  std::size_t segment = 0;
  if (nearest == previous.size() - 1) {
    segment = nearest - 1;
  } else if (nearest > 0) {
    const auto &a = previous[nearest];
    const auto &b = previous[nearest + 1];
    const double longitudinal =
        (point.x - a.x) * (b.x - a.x) +
        (point.y - a.y) * (b.y - a.y);
    segment = longitudinal <= 0.0 ? nearest - 1 : nearest;
  }
  const auto &a = previous[segment];
  const auto &b = previous[segment + 1];
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length2 = dx * dx + dy * dy;
  const double nearest_ratio = length2 > 1.0e-12 ? std::clamp(
      ((point.x - a.x) * dx + (point.y - a.y) * dy) / length2,
      0.0, 1.0) : 0.0;
  return {
      std::fabs(a.longitudinal_velocity_mps +
                (b.longitudinal_velocity_mps - a.longitudinal_velocity_mps) *
                    nearest_ratio),
      a.acceleration_mps2 +
          (b.acceleration_mps2 - a.acceleration_mps2) * nearest_ratio};
}

Trajectory finalizeJerkNodeTrajectory(
    const Trajectory &node_resampled_input, std::size_t closest_index,
    const Trajectory &optimized, const Trajectory *previous_output,
    const TrajectoryPoint &ego_pose, double ego_velocity,
    const JerkNodeOutputParameters &parameters,
    Trajectory *internal_output) {
  if (node_resampled_input.empty() || optimized.empty() ||
      closest_index >= node_resampled_input.size() ||
      !std::isfinite(ego_velocity) ||
      !std::isfinite(parameters.max_velocity) ||
      parameters.max_velocity < 0.0 ||
      (previous_output && previous_output->empty())) {
    throw std::invalid_argument("스무더 출력에 필요한 궤적과 설정이 잘못되었습니다");
  }

  const Trajectory clipped(
      node_resampled_input.begin() + closest_index, node_resampled_input.end());
  Trajectory output = optimized;
  enforceJerkNodeStop(
      clipped, output, parameters.nearest_distance, parameters.nearest_yaw);
  output.insert(
      output.begin(), node_resampled_input.begin(),
      node_resampled_input.begin() + closest_index);
  output.back().longitudinal_velocity_mps = 0.0F;
  for (std::size_t i = closest_index; i < output.size(); ++i) {
    output[i].longitudinal_velocity_mps =
        std::min(output[i].longitudinal_velocity_mps,
                 static_cast<float>(parameters.max_velocity));
  }
  for (std::size_t i = 0; i < closest_index; ++i) {
    const JerkMotion motion = previous_output &&
        !parameters.keep_closest_motion_behind
        ? previousMotionAt(*previous_output, output[i],
                           parameters.nearest_distance,
                           parameters.nearest_yaw)
        : JerkMotion{output[closest_index].longitudinal_velocity_mps,
                     output[closest_index].acceleration_mps2};
    output[i].longitudinal_velocity_mps = static_cast<float>(motion.velocity);
    output[i].acceleration_mps2 = static_cast<float>(motion.acceleration);
  }
  if (internal_output) *internal_output = output;
  auto resampled = resampleJerkNodeOutput(
      output, ego_velocity, ego_pose, parameters.nearest_distance,
      parameters.nearest_yaw, parameters.post_resampling,
      parameters.preserve_velocity_restarts);
  resampled.back().longitudinal_velocity_mps = 0.0F;
  return resampled;
}
}  // 익명 네임스페이스

JerkMotion projectJerkPreviousMotion(
    const Trajectory &previous, const TrajectoryPoint &ego_pose,
    double nearest_distance, double nearest_yaw) {
  if (previous.empty() || !std::isfinite(nearest_distance) ||
      nearest_distance < 0.0 || !std::isfinite(nearest_yaw) ||
      nearest_yaw < 0.0) {
    throw std::invalid_argument("이전 출력 투영 조건이 잘못되었습니다");
  }
  if (previous.size() == 1) {
    return {std::fabs(previous.front().longitudinal_velocity_mps),
            previous.front().acceleration_mps2};
  }

  // motion_utils::findFirstNearestIndexWithSoftConstraints searches the first
  // contiguous run satisfying both thresholds, then distance only, then all
  // points. It selects a point before choosing the neighboring segment.
  const double ego_yaw = yaw(ego_pose);
  auto select_index = [&](bool check_distance, bool check_yaw)
      -> std::optional<std::size_t> {
    double best = std::numeric_limits<double>::infinity();
    std::optional<std::size_t> selected;
    for (std::size_t i = 0; i < previous.size(); ++i) {
      const auto &point = previous[i];
      const double squared_distance =
          (point.x - ego_pose.x) * (point.x - ego_pose.x) +
          (point.y - ego_pose.y) * (point.y - ego_pose.y);
      const bool within =
          (!check_distance || squared_distance <= nearest_distance * nearest_distance) &&
          (!check_yaw ||
           std::fabs(std::remainder(yaw(point) - ego_yaw,
                                    2.0 * 3.141592653589793)) <= nearest_yaw);
      if (!within) {
        if (selected) break;
        continue;
      }
      if (squared_distance < best) {
        best = squared_distance;
        selected = i;
      }
    }
    return selected;
  };
  const std::size_t nearest = select_index(true, true)
      .value_or(select_index(true, false)
          .value_or(*select_index(false, false)));
  std::size_t segment = 0;
  if (nearest == previous.size() - 1) {
    segment = nearest - 1;
  } else if (nearest > 0) {
    const auto &a = previous[nearest];
    const auto &b = previous[nearest + 1];
    const double longitudinal =
        (ego_pose.x - a.x) * (b.x - a.x) +
        (ego_pose.y - a.y) * (b.y - a.y);
    segment = longitudinal <= 0.0 ? nearest - 1 : nearest;
  }
  const auto &a = previous[segment];
  const auto &b = previous[segment + 1];
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length_squared = dx * dx + dy * dy;
  const double ratio = length_squared > 1.0e-12 ? std::clamp(
      ((ego_pose.x - a.x) * dx + (ego_pose.y - a.y) * dy) / length_squared,
      0.0, 1.0) : 0.0;
  return {
      std::fabs(a.longitudinal_velocity_mps +
                (b.longitudinal_velocity_mps - a.longitudinal_velocity_mps) * ratio),
      a.acceleration_mps2 +
          (b.acceleration_mps2 - a.acceleration_mps2) * ratio};
}

StopPolicyResult enforceJerkNodeStop(
    const Trajectory &input, Trajectory &output, double maximum_distance,
    double maximum_yaw) {
  if (!std::isfinite(maximum_distance) || maximum_distance < 0.0 ||
      !std::isfinite(maximum_yaw) || maximum_yaw < 0.0) {
    throw std::invalid_argument("정지점 검색 거리가 잘못되었습니다");
  }
  StopPolicyResult result;
  std::size_t stop = 0;
  while (stop < input.size() &&
         !(std::fabs(input[stop].longitudinal_velocity_mps) < 1.0e-3)) ++stop;
  if (stop == input.size()) return result;
  result.stop_found = true;
  double best = std::numeric_limits<double>::infinity();
  std::size_t nearest = 0;
  for (std::size_t i = 0; i < output.size(); ++i) {
    const double distance = std::hypot(output[i].x - input[stop].x,
                                       output[i].y - input[stop].y);
    const double yaw_difference = std::fabs(std::remainder(
        yaw(output[i]) - yaw(input[stop]), 2.0 * 3.141592653589793));
    if (distance <= maximum_distance && yaw_difference <= maximum_yaw &&
        distance < best) {
      best = distance;
      nearest = i;
      result.stop_output_found = true;
    }
  }
  if (!result.stop_output_found) return result;
  result.velocity_before_enforcement = output[nearest].longitudinal_velocity_mps;
  for (std::size_t i = nearest; i < output.size(); ++i) {
    output[i].longitudinal_velocity_mps = 0.0F;
  }
  return result;
}

void limitVelocityApproachingStop(
    Trajectory &trajectory, double stopping_velocity,
    double stopping_distance) {
  if (!std::isfinite(stopping_velocity) || stopping_velocity < 0.0 ||
      !std::isfinite(stopping_distance) || stopping_distance < 0.0) {
    throw std::invalid_argument("정지 접근 속도 설정이 잘못되었습니다");
  }
  std::size_t stop = 0;
  while (stop < trajectory.size() &&
         !(std::fabs(trajectory[stop].longitudinal_velocity_mps) < 1.0e-3)) ++stop;
  if (stop == trajectory.size() || stop == 0) return;
  double distance = 0.0;
  for (std::size_t i = stop; i-- > 0;) {
    distance += std::hypot(trajectory[i + 1].x - trajectory[i].x,
                           trajectory[i + 1].y - trajectory[i].y);
    if (distance > stopping_distance) break;
    if (trajectory[i].longitudinal_velocity_mps > stopping_velocity) {
      trajectory[i].longitudinal_velocity_mps = static_cast<float>(stopping_velocity);
    }
  }
}

Trajectory limitJerkNodeInputVelocity(
    const Trajectory &input, double maximum_velocity,
    double stopping_velocity, double stopping_distance) {
  if (!std::isfinite(maximum_velocity) || maximum_velocity < 0.0) {
    throw std::invalid_argument("최고 속도 설정이 잘못되었습니다");
  }
  auto output = input;
  for (auto &point : output) {
    point.longitudinal_velocity_mps = std::min(
        point.longitudinal_velocity_mps, static_cast<float>(maximum_velocity));
  }
  limitVelocityApproachingStop(
      output, stopping_velocity, stopping_distance);
  return output;
}

Trajectory finalizeFirstJerkNodeTrajectory(
    const Trajectory &node_resampled_input, std::size_t closest_index,
    const Trajectory &optimized, const TrajectoryPoint &ego_pose,
    double ego_velocity, const JerkNodeOutputParameters &parameters,
    Trajectory *internal_output) {
  return finalizeJerkNodeTrajectory(
      node_resampled_input, closest_index, optimized, nullptr,
      ego_pose, ego_velocity, parameters, internal_output);
}

Trajectory finalizeNextJerkNodeTrajectory(
    const Trajectory &node_resampled_input, std::size_t closest_index,
    const Trajectory &optimized, const Trajectory &previous_output,
    const TrajectoryPoint &ego_pose, double ego_velocity,
    const JerkNodeOutputParameters &parameters,
    Trajectory *internal_output) {
  return finalizeJerkNodeTrajectory(
      node_resampled_input, closest_index, optimized, &previous_output,
      ego_pose, ego_velocity, parameters, internal_output);
}

JerkInitialMotionResult selectJerkInitialMotion(
    double vehicle_velocity, double vehicle_acceleration,
    double target_velocity, double distance_to_stop,
    bool autonomous_control_enabled,
    const std::optional<JerkMotion> &previous_desired,
    const JerkInitialMotionParameters &p,
    std::optional<double> external_engage_acceleration) {
  const double values[] = {
      vehicle_velocity, vehicle_acceleration, target_velocity,
      p.replan_velocity_deviation, p.engage_velocity, p.engage_acceleration,
      p.engage_exit_ratio, p.stop_distance_to_prohibit_engage,
      p.min_deceleration, p.max_acceleration};
  for (double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("초기 운동 상태에는 유한한 값이 필요합니다");
    }
  }
  if (std::isnan(distance_to_stop) || distance_to_stop < 0.0 ||
      p.replan_velocity_deviation < 0.0 || p.engage_velocity < 0.0 ||
      p.engage_exit_ratio < 0.0 || p.max_acceleration < p.min_deceleration ||
      (external_engage_acceleration &&
       !std::isfinite(*external_engage_acceleration)) ||
      (previous_desired &&
       (!std::isfinite(previous_desired->velocity) ||
        !std::isfinite(previous_desired->acceleration)))) {
    throw std::invalid_argument("초기 운동 상태 설정이 잘못되었습니다");
  }

  const double speed = std::fabs(vehicle_velocity);
  const double target = std::fabs(target_velocity);
  if (!previous_desired) {
    return {{speed, 0.0}, JerkInitialType::EgoVelocity};
  }
  const double deviation = speed - std::fabs(previous_desired->velocity);
  if (std::fabs(deviation) > p.replan_velocity_deviation) {
    return {{speed, previous_desired->acceleration},
            JerkInitialType::LargeDeviationReplan};
  }

  const double engage_threshold = p.engage_velocity * p.engage_exit_ratio;
  if (speed < engage_threshold && target >= p.engage_velocity &&
      distance_to_stop > p.stop_distance_to_prohibit_engage) {
    return {{p.engage_velocity,
             external_engage_acceleration.value_or(p.engage_acceleration)},
            JerkInitialType::Engaging};
  }

  if (p.plan_from_ego_on_manual_mode && !autonomous_control_enabled) {
    return {{std::min(target, speed),
             std::clamp(vehicle_acceleration,
                        p.min_deceleration, p.max_acceleration)},
            JerkInitialType::EgoVelocity};
  }
  return {*previous_desired, JerkInitialType::PreviousOutput};
}

bool isReverseJerkTrajectory(const Trajectory &trajectory) {
  if (trajectory.empty()) return true;
  return std::any_of(
      trajectory.begin(), trajectory.end(),
      [](const TrajectoryPoint &point) {
        return point.longitudinal_velocity_mps < 0.0F;
      });
}

void flipJerkTrajectoryVelocity(Trajectory &trajectory) {
  for (auto &point : trajectory) {
    point.longitudinal_velocity_mps *= -1.0F;
  }
}

}  // planning 네임스페이스
