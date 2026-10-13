// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// autoware_velocity_smoother/src/smoother/jerk_filtered_smoother.cpp를 바탕으로 작성했습니다.

#include "planning/jerk_filter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace planning {
namespace {

double distance2d(const FilterPoint &a, const FilterPoint &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

FilterTrajectory forwardJerkFilter(
    double v0, double a0, double a_max, double a_start, double j_max,
    const FilterTrajectory &input) {
  auto apply_limits = [&input, a_start](double &v, double &a, std::size_t i) {
    const double v_limit = input.at(i).longitudinal_velocity_mps;
    constexpr double epsilon = 1.0e-5;
    if (v > v_limit + epsilon) {
      v = v_limit;
      a = 0.0;
      if (v_limit < 1.0e-3 && i < input.size() - 1) {
        const double next_limit = input.at(i + 1).longitudinal_velocity_mps;
        if (next_limit >= 1.0e-3) {
          a = a_start;
        }
      }
    }
    if (v < 0.0) {
      v = a = 0.0;
    }
  };

  auto output = input;
  double current_velocity = v0;
  double current_acceleration = a0;
  apply_limits(current_velocity, current_acceleration, 0);
  output.front().longitudinal_velocity_mps = static_cast<float>(current_velocity);
  output.front().acceleration_mps2 = static_cast<float>(current_acceleration);

  for (std::size_t i = 1; i < input.size(); ++i) {
    const double ds = distance2d(input.at(i), input.at(i - 1));
    const double max_dt = std::pow(6.0 * ds / j_max, 1.0 / 3.0);
    const double dt = std::min(ds / std::max(current_velocity, 1.0e-6), max_dt);
    if (current_acceleration + j_max * dt >= a_max) {
      const double temporary_jerk = std::min((a_max - current_acceleration) / dt, j_max);
      current_velocity = current_velocity + current_acceleration * dt +
                         0.5 * temporary_jerk * dt * dt;
      current_acceleration = a_max;
    } else {
      current_velocity = current_velocity + current_acceleration * dt +
                         0.5 * j_max * dt * dt;
      current_acceleration += j_max * dt;
    }
    apply_limits(current_velocity, current_acceleration, i);
    output.at(i).longitudinal_velocity_mps = static_cast<float>(current_velocity);
    output.at(i).acceleration_mps2 = static_cast<float>(current_acceleration);
  }
  return output;
}

FilterTrajectory backwardJerkFilter(
    double v0, double a0, double a_min, double a_stop, double j_min,
    const FilterTrajectory &input) {
  auto reversed = input;
  std::reverse(reversed.begin(), reversed.end());
  auto filtered = forwardJerkFilter(
      v0, std::fabs(a0), std::fabs(a_min), std::fabs(a_stop), std::fabs(j_min), reversed);
  std::reverse(filtered.begin(), filtered.end());
  for (auto &point : filtered) {
    point.acceleration_mps2 *= -1.0F;
  }
  return filtered;
}

FilterTrajectory mergeFilteredTrajectory(
    double v0, double a0, double a_min, double j_min,
    const FilterTrajectory &forward, const FilterTrajectory &backward) {
  auto merged = forward;
  std::size_t i = 0;

  if (backward.at(0).longitudinal_velocity_mps < v0) {
    double current_velocity = v0;
    double current_acceleration = a0;
    while (backward.at(i).longitudinal_velocity_mps < current_velocity && i < merged.size() - 1) {
      merged.at(i).longitudinal_velocity_mps = static_cast<float>(current_velocity);
      merged.at(i).acceleration_mps2 = static_cast<float>(current_acceleration);

      const double ds = distance2d(forward.at(i + 1), forward.at(i));
      const double max_dt = std::pow(6.0 * ds / std::fabs(j_min), 1.0 / 3.0);
      const double dt = std::min(ds / std::max(current_velocity, 1.0e-6), max_dt);
      if (current_acceleration + j_min * dt < a_min) {
        const double temporary_jerk = std::max((a_min - current_acceleration) / dt, j_min);
        current_velocity = current_velocity + current_acceleration * dt +
                           0.5 * temporary_jerk * dt * dt;
        current_acceleration = std::max(current_acceleration + temporary_jerk * dt, a_min);
      } else {
        current_velocity = current_velocity + current_acceleration * dt +
                           0.5 * j_min * dt * dt;
        current_acceleration += j_min * dt;
      }
      if (current_velocity > forward.at(i).longitudinal_velocity_mps) {
        current_velocity = forward.at(i).longitudinal_velocity_mps;
      }
      ++i;
    }
  }

  for (; i < merged.size(); ++i) {
    merged.at(i) = forward.at(i).longitudinal_velocity_mps <
                           backward.at(i).longitudinal_velocity_mps
                       ? forward.at(i)
                       : backward.at(i);
  }
  return merged;
}

}  // 익명 네임스페이스

JerkFilterStages runJerkFilterStage(
    const FilterTrajectory &input, double initial_velocity, double initial_acceleration,
    const JerkFilterLimits &limits) {
  if (input.empty()) {
    throw std::invalid_argument("jerk filter requires at least one trajectory point");
  }
  if (!std::isfinite(limits.max_jerk) || !std::isfinite(limits.min_jerk) ||
      !(limits.max_jerk > 0.0) || !(limits.min_jerk < 0.0)) {
    throw std::invalid_argument("jerk filter requires positive max and negative min jerk");
  }
  if (!std::isfinite(initial_velocity) || !std::isfinite(initial_acceleration)) {
    throw std::invalid_argument("jerk filter requires finite initial state");
  }
  for (std::size_t i = 0; i < input.size(); ++i) {
    const auto &point = input[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.longitudinal_velocity_mps) ||
        !std::isfinite(point.acceleration_mps2)) {
      throw std::invalid_argument("jerk filter requires finite trajectory points");
    }
    if (i > 0 && !(distance2d(point, input[i - 1]) > 0.0)) {
      throw std::invalid_argument("jerk filter requires positive point spacing");
    }
  }

  JerkFilterStages stages;
  stages.forward = forwardJerkFilter(
      initial_velocity, std::max(initial_acceleration, limits.min_deceleration),
      limits.max_acceleration, 0.0, limits.max_jerk, input);
  stages.backward = backwardJerkFilter(
      input.back().longitudinal_velocity_mps, limits.stop_deceleration,
      limits.min_deceleration, limits.stop_deceleration, limits.min_jerk, input);
  stages.merged = mergeFilteredTrajectory(
      initial_velocity, initial_acceleration, limits.min_deceleration, limits.min_jerk,
      stages.forward, stages.backward);
  return stages;
}

}  // planning 네임스페이스
