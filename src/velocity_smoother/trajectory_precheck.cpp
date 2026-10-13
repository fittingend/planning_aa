// Copyright 2021, 2022 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// autoware_planning_validator/src/{utils,planning_validator}.cpp를 바탕으로 작성했습니다.

#include "planning/trajectory_precheck.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <vector>

namespace planning {
namespace {

constexpr double kPi = 3.14159265358979323846;

double distance2d(const TrajectoryPoint &a, const TrajectoryPoint &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

CheckResult upperCheck(double observed, std::size_t index, double limit) {
  return {true, observed <= limit, observed, index};
}

CheckResult lowerCheck(double observed, std::size_t index, double limit) {
  return {true, observed >= limit, observed, index};
}

bool pointIsFinite(const TrajectoryPoint &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
         std::isfinite(point.orientation_x) && std::isfinite(point.orientation_y) &&
         std::isfinite(point.orientation_z) && std::isfinite(point.orientation_w) &&
         std::isfinite(point.longitudinal_velocity_mps) &&
         std::isfinite(point.heading_rate_rps) && std::isfinite(point.acceleration_mps2);
}

Trajectory resampleWithoutInterpolation(const Trajectory &input, double min_interval) {
  Trajectory output;
  if (input.empty()) {
    return output;
  }
  output.push_back(input.front());
  for (std::size_t i = 1; i < input.size(); ++i) {
    if (distance2d(output.back(), input[i]) > min_interval) {
      output.push_back(input[i]);
    }
  }
  return output;
}

std::vector<double> curvatures(const Trajectory &trajectory, double curvature_distance = 1.0) {
  std::vector<double> values(trajectory.size(), 0.0);
  if (trajectory.size() < 3) {
    return values;
  }

  std::vector<double> arc_length(trajectory.size(), 0.0);
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    arc_length[i] = arc_length[i - 1] + distance2d(trajectory[i - 1], trajectory[i]);
  }

  std::size_t first_distant_index = 0;
  std::size_t last_distant_index = trajectory.size() - 1;
  for (std::size_t i = 1; i < trajectory.size() - 1; ++i) {
    std::size_t previous = 0;
    for (std::size_t j = i - 1; j > 0; --j) {
      if (arc_length[i] - arc_length[j] > curvature_distance) {
        if (first_distant_index == 0) {
          first_distant_index = i;
        }
        previous = j;
        break;
      }
    }

    std::size_t next = trajectory.size() - 1;
    for (std::size_t j = i + 1; j < trajectory.size(); ++j) {
      if (arc_length[j] - arc_length[i] > curvature_distance) {
        last_distant_index = i;
        next = j;
        break;
      }
    }

    const auto &p1 = trajectory[previous];
    const auto &p2 = trajectory[i];
    const auto &p3 = trajectory[next];
    const double denominator =
        distance2d(p1, p2) * distance2d(p2, p3) * distance2d(p3, p1);
    if (std::fabs(denominator) >= 1.0e-10) {
      values[i] = 2.0 * ((p2.x - p1.x) * (p3.y - p1.y) -
                         (p2.y - p1.y) * (p3.x - p1.x)) /
                  denominator;
    }
  }

  for (std::size_t i = first_distant_index; i > 0; --i) {
    values[i - 1] = values[i];
  }
  for (std::size_t i = last_distant_index; i < values.size() - 1; ++i) {
    values[i + 1] = values[i];
  }
  return values;
}

template <typename Function>
std::pair<double, std::size_t> maximumByIndex(std::size_t count, Function value) {
  double maximum = 0.0;
  std::size_t index = 0;
  for (std::size_t i = 0; i < count; ++i) {
    const double candidate = value(i);
    if (maximum < candidate) {
      maximum = candidate;
      index = i;
    }
  }
  return {maximum, index};
}

}  // 익명 네임스페이스

bool TrajectoryPrecheckResult::localChecksPass() const noexcept {
  const CheckResult *checks[] = {
      &size, &finite, &interval, &relative_angle, &curvature, &lateral_acceleration,
      &longitudinal_max_acceleration, &longitudinal_min_acceleration, &steering_angle,
      &steering_rate};
  for (const auto *check : checks) {
    if (!check->evaluated || !check->passed) {
      return false;
    }
  }
  return true;
}

bool EgoCheckResult::checksPass() const noexcept {
  const CheckResult *checks[] = {
      &velocity_deviation, &distance_deviation, &longitudinal_distance_deviation,
      &forward_length};
  for (const auto *check : checks) {
    if (!check->evaluated || !check->passed) {
      return false;
    }
  }
  return true;
}

bool TrajectoryValidationResult::allValid() const noexcept {
  return local.localChecksPass() && ego.checksPass();
}

TrajectoryPrecheckResult checkTrajectoryLocal(
    const Trajectory &trajectory, const PrecheckThresholds &thresholds) {
  TrajectoryPrecheckResult result;
  result.size = {true, trajectory.size() >= 2, static_cast<double>(trajectory.size()), 0};
  if (!result.size.passed) {
    return result;
  }

  result.finite = {true, true, 0.0, 0};
  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    if (!pointIsFinite(trajectory[i])) {
      result.finite = {true, false, 0.0, i};
      return result;
    }
  }

  const auto [max_interval, interval_index] = maximumByIndex(
      trajectory.size() - 1, [&](std::size_t i) {
        return distance2d(trajectory[i], trajectory[i + 1]);
      });
  result.interval = upperCheck(
      max_interval, max_interval > 0.0 ? interval_index + 1 : 0,
      thresholds.max_interval_m);

  const auto [max_acc, max_acc_index] = maximumByIndex(
      trajectory.size(), [&](std::size_t i) {
        return static_cast<double>(trajectory[i].acceleration_mps2);
      });
  result.longitudinal_max_acceleration = upperCheck(
      max_acc, max_acc_index, thresholds.max_longitudinal_acceleration_mps2);

  double min_acc = 0.0;
  std::size_t min_acc_index = 0;
  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    if (min_acc > trajectory[i].acceleration_mps2) {
      min_acc = trajectory[i].acceleration_mps2;
      min_acc_index = i;
    }
  }
  result.longitudinal_min_acceleration = lowerCheck(
      min_acc, min_acc_index, thresholds.min_longitudinal_acceleration_mps2);

  const auto resampled = resampleWithoutInterpolation(trajectory, 1.0);
  const auto [max_relative_angle, relative_index] = maximumByIndex(
      resampled.size() < 3 ? 0 : resampled.size() - 2, [&](std::size_t i) {
        const double angle_a = std::atan2(resampled[i + 1].y - resampled[i].y,
                                          resampled[i + 1].x - resampled[i].x);
        const double angle_b = std::atan2(resampled[i + 2].y - resampled[i + 1].y,
                                          resampled[i + 2].x - resampled[i + 1].x);
        return std::abs(std::remainder(angle_b - angle_a, 2.0 * kPi));
      });
  result.relative_angle = upperCheck(
      max_relative_angle, relative_index, thresholds.max_relative_angle_rad);

  const auto curvature_values = curvatures(resampled);
  const auto curvature_it = std::max_element(
      curvature_values.begin(), curvature_values.end());
  const double max_curvature = curvature_it == curvature_values.end()
      ? 0.0 : *curvature_it;
  const std::size_t curvature_index = curvature_it == curvature_values.end()
      ? 0 : std::distance(curvature_values.begin(), curvature_it);
  result.curvature = upperCheck(
      max_curvature, curvature_index, thresholds.max_curvature_per_m);

  const auto [max_lateral_acceleration, lateral_index] = maximumByIndex(
      curvature_values.size(), [&](std::size_t i) {
        const double velocity = resampled[i].longitudinal_velocity_mps;
        return std::abs(velocity * velocity * curvature_values[i]);
      });
  result.lateral_acceleration = upperCheck(
      max_lateral_acceleration, lateral_index, thresholds.max_lateral_acceleration_mps2);

  std::vector<double> steering(curvature_values.size());
  for (std::size_t i = 0; i < steering.size(); ++i) {
    steering[i] = std::atan(curvature_values[i] * thresholds.wheelbase_m);
  }
  const auto [max_steering, steering_index] = maximumByIndex(
      steering.size(), [&](std::size_t i) { return std::abs(steering[i]); });
  result.steering_angle = upperCheck(
      max_steering, steering_index, thresholds.max_steering_angle_rad);

  const auto [max_steering_rate, steering_rate_index] = maximumByIndex(
      resampled.size() - 1, [&](std::size_t i) {
        const double ds = distance2d(resampled[i], resampled[i + 1]);
        const double velocity = 0.5 * (resampled[i].longitudinal_velocity_mps +
                                       resampled[i + 1].longitudinal_velocity_mps);
        const double dt = ds / std::max(velocity, 1.0e-5);
        return std::abs((steering[i + 1] - steering[i]) / dt);
      });
  result.steering_rate = upperCheck(
      max_steering_rate, steering_rate_index, thresholds.max_steering_rate_radps);

  return result;
}

EgoCheckResult checkTrajectoryAgainstEgo(
    const Trajectory &trajectory, const EgoState &ego, const PrecheckThresholds &thresholds) {
  EgoCheckResult result;
  if (trajectory.size() < 2 || !std::isfinite(ego.x) || !std::isfinite(ego.y) ||
      !std::isfinite(ego.longitudinal_velocity_mps)) {
    return result;
  }

  // 원본의 기본 완화 제약은 사실상 경계가 없습니다. 따라서 첫 최근접점 검색은
  // XY 거리가 최소인 점 중 첫 번째를 선택합니다.
  std::size_t nearest = 0;
  double nearest_distance = distance2d(trajectory[0], {ego.x, ego.y});
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    const double distance = distance2d(trajectory[i], {ego.x, ego.y});
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = i;
    }
  }

  const double velocity_deviation = std::abs(
      trajectory[nearest].longitudinal_velocity_mps - ego.longitudinal_velocity_mps);
  result.velocity_deviation = upperCheck(
      velocity_deviation, nearest, thresholds.max_velocity_deviation_mps);
  result.distance_deviation = upperCheck(
      nearest_distance, nearest, thresholds.max_distance_deviation_m);

  const auto offset = [&](std::size_t segment) {
    const auto &front = trajectory[segment];
    const auto &back = trajectory[segment + 1];
    const double length = distance2d(front, back);
    if (!(length > 0.0)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return ((back.x - front.x) * (ego.x - front.x) +
            (back.y - front.y) * (ego.y - front.y)) /
           length;
  };

  double longitudinal_deviation = 0.0;
  if (nearest == 0) {
    longitudinal_deviation = offset(0);
  } else if (nearest == trajectory.size() - 1) {
    longitudinal_deviation = offset(trajectory.size() - 2) -
                             distance2d(trajectory.back(), trajectory[trajectory.size() - 2]);
  }
  result.longitudinal_distance_deviation = {
      true,
      nearest != 0 && nearest != trajectory.size() - 1
          ? true
          : std::abs(longitudinal_deviation) <
                thresholds.max_longitudinal_distance_deviation_m,
      longitudinal_deviation,
      nearest};

  const double speed = std::abs(ego.longitudinal_velocity_mps);
  if (speed < 1.0 / 3.6) {
    result.forward_length = {true, true, 0.0, nearest};
    return result;
  }

  // 원본은 최근접 선분을 고른 다음 차량 투영점에서 궤적 종점까지의 호 길이를 측정합니다.
  std::size_t segment = nearest;
  if (nearest == trajectory.size() - 1) {
    segment = trajectory.size() - 2;
  } else if (nearest > 0 && offset(nearest) <= 0.0) {
    segment = nearest - 1;
  }
  double forward_length = -offset(segment);
  for (std::size_t i = segment; i < trajectory.size() - 1; ++i) {
    forward_length += distance2d(trajectory[i], trajectory[i + 1]);
  }

  result.forward_length_required_m =
      speed * speed / (2.0 * std::abs(thresholds.forward_length_deceleration_mps2)) -
      thresholds.forward_length_margin_m;
  result.forward_length = {
      true, forward_length > result.forward_length_required_m, forward_length, segment};
  return result;
}

TrajectoryValidationResult validateTrajectory(
    const Trajectory &trajectory, const EgoState &ego, const PrecheckThresholds &thresholds) {
  TrajectoryValidationResult result;
  result.local = checkTrajectoryLocal(trajectory, thresholds);
  if (result.local.size.passed && result.local.finite.passed) {
    result.ego = checkTrajectoryAgainstEgo(trajectory, ego, thresholds);
  }
  return result;
}

}  // planning 네임스페이스
