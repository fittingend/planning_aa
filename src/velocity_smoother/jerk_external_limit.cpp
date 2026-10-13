// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 trajectory_utils와 외부 속도 제한 변경 계산을 이관했습니다.

#include "planning/jerk_external_limit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace planning {
namespace {

struct DistanceResult {
  double distance = 0.0;
  bool valid = false;
};

DistanceResult stoppingDistance(
    double velocity, double acceleration, double target,
    double acceleration_min, double jerk_acc, double jerk_dec) {
  const auto time_to = [](double from, double to, double jerk) {
    return (to - from) / jerk;
  };
  const double difference_dec =
      0.5 * time_to(acceleration, 0.0, jerk_dec) * acceleration +
      0.5 * time_to(0.0, acceleration_min, jerk_dec) * acceleration_min;
  const double difference_acc =
      0.5 * time_to(acceleration_min, 0.0, jerk_acc) * acceleration_min;
  const double error = target - velocity;
  const double constant_time =
      (error - difference_dec - difference_acc) / acceleration_min;

  std::vector<std::pair<double, double>> profile;
  if (constant_time > 0.0) {
    profile = {
        {jerk_dec, time_to(acceleration, acceleration_min, jerk_dec)},
        {0.0, constant_time},
        {jerk_acc, time_to(acceleration_min, 0.0, jerk_acc)}};
  } else {
    const double difference_from_decel =
        0.5 * acceleration * time_to(acceleration, 0.0, jerk_acc);
    if (difference_from_decel > error || acceleration > 0.0) {
      const double squared =
          (error - 0.5 * acceleration *
                       time_to(acceleration, 0.0, jerk_dec)) *
          2.0 / (1.0 / jerk_dec - 1.0 / jerk_acc);
      if (squared < 0.0) return {};
      const double minimum_acceleration = -std::sqrt(squared);
      profile = {
          {jerk_dec, time_to(acceleration, minimum_acceleration, jerk_dec)},
          {jerk_acc, time_to(minimum_acceleration, 0.0, jerk_acc)}};
    } else {
      const double jerk =
          -acceleration * acceleration / (2.0 * (target - velocity));
      const double time = time_to(acceleration, 0.0, jerk);
      if (!std::isfinite(time) || time < 0.0) return {};
      profile = {{jerk, time}};
    }
  }

  double position = 0.0;
  for (auto [jerk, duration] : profile) {
    if (duration < 1.0e-6) duration = 0.0;
    if (!std::isfinite(duration)) return {};
    position += velocity * duration + 0.5 * acceleration * duration * duration +
                jerk * duration * duration * duration / 6.0;
    velocity += acceleration * duration + 0.5 * jerk * duration * duration;
    acceleration += jerk * duration;
  }
  return {position,
          std::fabs(velocity - target) <= 0.3 &&
          std::fabs(acceleration) <= 0.1};
}

}  // 익명 네임스페이스

JerkExternalLimitPlan planJerkExternalLimit(
    const JerkMotion &previous_motion, double target_velocity,
    const JerkExternalLimitParameters &p) {
  const double values[] = {
      previous_motion.velocity, previous_motion.acceleration,
      target_velocity, p.min_deceleration, p.max_jerk,
      p.min_jerk, p.insertion_margin};
  for (double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("외부 제한 계산에는 유한한 값이 필요합니다");
    }
  }
  if (target_velocity < 0.0 || p.min_deceleration >= 0.0 ||
      p.max_jerk <= 0.0 || p.min_jerk >= 0.0 ||
      p.insertion_margin < 0.0) {
    throw std::invalid_argument("외부 제한 설정이 잘못되었습니다");
  }

  JerkExternalLimitPlan result;
  result.target_velocity = target_velocity;
  result.velocity_before_limit = previous_motion.velocity;
  if (previous_motion.acceleration > 0.0) {
    result.velocity_before_limit -=
        0.5 * previous_motion.acceleration * previous_motion.acceleration /
        p.min_jerk;
  }
  if (target_velocity < result.velocity_before_limit) {
    const auto stop = stoppingDistance(
        previous_motion.velocity, previous_motion.acceleration,
        target_velocity, p.min_deceleration, p.max_jerk, p.min_jerk);
    result.insertion_distance = stop.distance + p.insertion_margin;
    result.distance_valid = stop.valid;
  } else {
    result.velocity_before_limit = target_velocity;
  }
  return result;
}

void advanceJerkExternalLimit(
    JerkExternalLimitPlan &plan, double travel_distance) {
  if (!std::isfinite(travel_distance) || travel_distance < 0.0) {
    throw std::invalid_argument("이동 거리는 음수가 아닌 유한한 값이어야 합니다");
  }
  plan.insertion_distance =
      std::max(plan.insertion_distance - travel_distance, 0.0);
}

Trajectory applyJerkExternalLimit(
    const Trajectory &input, const TrajectoryPoint &ego_pose,
    const JerkExternalLimitPlan &plan) {
  if (input.empty() || !std::isfinite(plan.velocity_before_limit) ||
      !std::isfinite(plan.target_velocity) ||
      !std::isfinite(plan.insertion_distance)) {
    throw std::invalid_argument("외부 제한 적용에 필요한 궤적과 설정이 잘못되었습니다");
  }
  auto output = input;
  for (auto &point : output) {
    point.longitudinal_velocity_mps = std::min(
        point.longitudinal_velocity_mps,
        static_cast<float>(plan.velocity_before_limit));
  }
  if (output.size() == 1) {
    output.back().longitudinal_velocity_mps = std::min(
        output.back().longitudinal_velocity_mps,
        static_cast<float>(plan.target_velocity));
    return output;
  }

  std::size_t nearest_segment = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < output.size(); ++i) {
    const double dx = output[i + 1].x - output[i].x;
    const double dy = output[i + 1].y - output[i].y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared <= 1.0e-12) continue;
    const double ratio = std::clamp(
        ((ego_pose.x - output[i].x) * dx +
         (ego_pose.y - output[i].y) * dy) / length_squared,
        0.0, 1.0);
    const double px = output[i].x + ratio * dx;
    const double py = output[i].y + ratio * dy;
    const double squared =
        (ego_pose.x - px) * (ego_pose.x - px) +
        (ego_pose.y - py) * (ego_pose.y - py);
    if (squared < nearest_distance) {
      nearest_distance = squared;
      nearest_segment = i;
    }
  }

  double remaining = std::max(plan.insertion_distance, 0.0);
  std::size_t index = nearest_segment;
  // 원본 insertTargetPoint는 차량 투영 비율을 더하지 않고 구간 시작점에서 잽니다.
  double ratio = 0.0;
  while (index + 1 < output.size()) {
    const double dx = output[index + 1].x - output[index].x;
    const double dy = output[index + 1].y - output[index].y;
    const double segment_length = std::hypot(dx, dy);
    const double available = segment_length * (1.0 - ratio);
    if (remaining <= available) {
      const double target_ratio = ratio + remaining / segment_length;
      const double epsilon = 1.0e-3;
      std::size_t limit_index = index + 1;
      if (target_ratio < 1.0 - epsilon && target_ratio > epsilon) {
        auto point = output[index];
        point.x += dx * target_ratio;
        point.y += dy * target_ratio;
        point.z += (output[index + 1].z - point.z) * target_ratio;
        output.insert(output.begin() + limit_index, point);
      } else if (target_ratio <= epsilon) {
        limit_index = index;
      }
      for (std::size_t i = limit_index; i < output.size(); ++i) {
        output[i].longitudinal_velocity_mps = std::min(
            output[i].longitudinal_velocity_mps,
            static_cast<float>(plan.target_velocity));
      }
      return output;
    }
    remaining -= available;
    ++index;
    ratio = 0.0;
  }
  output.back().longitudinal_velocity_mps = std::min(
      output.back().longitudinal_velocity_mps,
      static_cast<float>(plan.target_velocity));
  return output;
}

}  // planning 네임스페이스
