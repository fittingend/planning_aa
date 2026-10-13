// SPDX-License-Identifier: Apache-2.0
// 원본 VelocityOptimizer의 목적 함수와 선형 제약을 독립 QP로 옮겼습니다.

#include "planning/obstacle_cruise_optimization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {

CruiseOptimizationInitialMotion cruiseOptimizationInitialMotion(
    const double ego_velocity_mps, const double target_velocity_mps,
    const std::optional<CruiseOptimizationInitialMotion> previous_motion,
    const std::optional<double> distance_to_forward_stop_m,
    const CruiseOptimizationEngageParameters &parameters) {
  if (!std::isfinite(ego_velocity_mps) ||
      !std::isfinite(target_velocity_mps) ||
      (previous_motion &&
       (!std::isfinite(previous_motion->velocity_mps) ||
        !std::isfinite(previous_motion->acceleration_mps2))) ||
      (distance_to_forward_stop_m &&
       !std::isfinite(*distance_to_forward_stop_m)) ||
      !std::isfinite(parameters.replan_velocity_deviation_mps) ||
      !std::isfinite(parameters.engage_velocity_mps) ||
      !std::isfinite(parameters.engage_acceleration_mps2) ||
      !std::isfinite(parameters.engage_exit_ratio) ||
      !std::isfinite(parameters.stop_distance_to_prohibit_engage_m) ||
      parameters.replan_velocity_deviation_mps < 0.0 ||
      parameters.engage_velocity_mps < 0.0 ||
      parameters.engage_exit_ratio < 0.0 ||
      parameters.stop_distance_to_prohibit_engage_m < 0.0)
    throw std::invalid_argument("순항 최적화 초기 운동 입력이 유효하지 않습니다");
  const double ego_speed = std::abs(ego_velocity_mps);
  if (!previous_motion) return {ego_speed, 0.0};
  if (std::abs(ego_speed -
               std::abs(previous_motion->velocity_mps)) >
      parameters.replan_velocity_deviation_mps)
    return {ego_speed, previous_motion->acceleration_mps2};
  if (ego_speed < parameters.engage_velocity_mps *
                      parameters.engage_exit_ratio &&
      std::abs(target_velocity_mps) >= parameters.engage_velocity_mps &&
      (!distance_to_forward_stop_m ||
       *distance_to_forward_stop_m >
           parameters.stop_distance_to_prohibit_engage_m))
    return {parameters.engage_velocity_mps,
            parameters.engage_acceleration_mps2};
  return *previous_motion;
}

std::vector<double> cruiseOptimizationTimeVector(
    const double dense_interval_s, const double sparse_interval_s,
    const double dense_horizon_s, const double maximum_horizon_s) {
  if (!std::isfinite(dense_interval_s) ||
      !std::isfinite(sparse_interval_s) ||
      !std::isfinite(dense_horizon_s) ||
      !std::isfinite(maximum_horizon_s) ||
      dense_interval_s <= 0.0 || sparse_interval_s <= 0.0 ||
      dense_horizon_s <= 0.0 || maximum_horizon_s < dense_horizon_s)
    throw std::invalid_argument("순항 최적화 시각 격자가 유효하지 않습니다");
  std::vector<double> time;
  for (double t = 0.0; t < dense_horizon_s; t += dense_interval_s)
    time.push_back(t);
  if (dense_horizon_s - time.back() < 1e-3)
    time.back() = dense_horizon_s;
  else time.push_back(dense_horizon_s);
  for (double t = dense_horizon_s + sparse_interval_s;
       t < maximum_horizon_s; t += sparse_interval_s)
    time.push_back(t);
  if (maximum_horizon_s - time.back() < 1e-3)
    time.back() = maximum_horizon_s;
  else time.push_back(maximum_horizon_s);
  return time;
}

std::vector<CruiseOptimizationBoundary> cruiseOptimizationBoundaries(
    const std::vector<double> &time_s, const double trajectory_length_m,
    const std::vector<CruiseOptimizationObstacle> &obstacles,
    const CruiseOptimizationBoundaryParameters &parameters) {
  if (time_s.size() < 2 || !std::isfinite(trajectory_length_m) ||
      !std::isfinite(parameters.safe_distance_margin_m) ||
      !std::isfinite(parameters.minimum_object_acceleration_mps2) ||
      !std::isfinite(parameters.vehicle_half_width_m) ||
      trajectory_length_m < 0.0 ||
      parameters.safe_distance_margin_m < 0.0 ||
      parameters.minimum_object_acceleration_mps2 >= 0.0 ||
      parameters.vehicle_half_width_m < 0.0)
    throw std::invalid_argument("순항 최적화 거리 경계 입력이 유효하지 않습니다");
  std::vector<CruiseOptimizationBoundary> result(
      time_s.size(), {trajectory_length_m, false});
  for (const auto &object : obstacles) {
    if (object.collision_points.empty()) continue;
    const auto &first = object.collision_points.front();
    const bool on_trajectory = first.time_from_now_s <= 0.1 &&
        std::abs(first.lateral_distance_m) <
            parameters.vehicle_half_width_m + 0.1;
    const double velocity = std::abs(object.velocity_mps);
    const double braking_distance = velocity * velocity /
        (2.0 * std::abs(parameters.minimum_object_acceleration_mps2));
    if (on_trajectory) {
      double object_position = std::max(
          first.distance_from_ego_front_m -
              parameters.safe_distance_margin_m, 0.0);
      for (std::size_t i = 0; i < time_s.size(); ++i) {
        if (i > 0) object_position +=
            velocity * (time_s[i] - time_s[i - 1]);
        const double maximum = std::clamp(
            object_position + braking_distance, 0.0,
            trajectory_length_m);
        if (maximum < result[i].maximum_distance_m)
          result[i] = {maximum, true};
      }
      continue;
    }
    for (const auto &collision : object.collision_points) {
      if (!std::isfinite(collision.time_from_now_s) ||
          !std::isfinite(collision.distance_from_ego_front_m) ||
          collision.time_from_now_s < 0.0) continue;
      const double maximum = std::max(0.0,
          collision.distance_from_ego_front_m -
              parameters.safe_distance_margin_m + braking_distance);
      std::size_t segment = 0;
      for (std::size_t i = 0; i + 1 < time_s.size(); ++i) {
        if (time_s[i] < collision.time_from_now_s &&
            collision.time_from_now_s < time_s[i + 1]) {
          segment = i;
          break;
        }
      }
      for (std::size_t i = 0;
           i <= segment + 1 && i < time_s.size(); ++i) {
        if (time_s[i] < collision.time_from_now_s &&
            maximum < result[i].maximum_distance_m)
          result[i] = {maximum, true};
      }
    }
  }
  return result;
}

std::optional<CruiseOptimizationResult> processCruiseOptimizationResult(
    const double initial_velocity_mps, const CruiseOptimizationResult &result,
    const double trajectory_front_offset_m) {
  const auto n = result.distance_m.size();
  if (n == 0 || result.time_s.size() != n ||
      result.velocity_mps.size() != n ||
      result.acceleration_mps2.size() != n ||
      result.jerk_mps3.size() != n)
    return std::nullopt;
  CruiseOptimizationResult processed;
  processed.time_s.push_back(0.0);
  processed.distance_m.push_back(trajectory_front_offset_m);
  processed.velocity_mps.push_back(initial_velocity_mps);
  processed.acceleration_mps2.push_back(result.acceleration_mps2.front());
  processed.jerk_mps3.push_back(result.jerk_mps3.front());
  std::size_t break_index = n;
  for (std::size_t i = 1; i < n; ++i) {
    const double current = std::max(result.distance_m[i], 0.0) +
                           trajectory_front_offset_m;
    if (processed.distance_m.back() >= current) {
      processed.velocity_mps.back() = 0.0;
      break_index = i;
      break;
    }
    if (result.velocity_mps[i] < 0.01) {
      break_index = i;
      break;
    }
    if (std::abs(current - processed.distance_m.back()) < 1e-3) {
      processed.velocity_mps.back() = result.velocity_mps[i];
      processed.distance_m.back() = current;
      continue;
    }
    processed.time_s.push_back(result.time_s[i]);
    processed.distance_m.push_back(current);
    processed.velocity_mps.push_back(result.velocity_mps[i]);
    processed.acceleration_mps2.push_back(result.acceleration_mps2[i]);
    processed.jerk_mps3.push_back(result.jerk_mps3[i]);
  }
  for (std::size_t i = break_index; i < n; ++i) {
    const double current = std::max(result.distance_m[i], 0.0) +
                           trajectory_front_offset_m;
    if (processed.distance_m.back() >= current) {
      processed.velocity_mps.back() = 0.0;
      continue;
    }
    if (std::abs(current - processed.distance_m.back()) < 1e-3) {
      processed.velocity_mps.back() = 0.0;
      processed.distance_m.back() = current;
      continue;
    }
    processed.time_s.push_back(result.time_s[i]);
    processed.distance_m.push_back(current);
    processed.velocity_mps.push_back(0.0);
    processed.acceleration_mps2.push_back(0.0);
    processed.jerk_mps3.push_back(0.0);
  }
  return processed;
}

std::optional<Trajectory> applyCruiseOptimizationResult(
    const Trajectory &trajectory, const std::size_t ego_index,
    const double initial_velocity_mps,
    const CruiseOptimizationResult &raw_result,
    const double velocity_margin_mps) {
  if (trajectory.size() < 2 || ego_index >= trajectory.size() ||
      !std::isfinite(initial_velocity_mps) ||
      !std::isfinite(velocity_margin_mps) || velocity_margin_mps < 0.0)
    throw std::invalid_argument("순항 최적화 경로 적용 입력이 유효하지 않습니다");
  std::vector<double> arcs(trajectory.size(), 0.0);
  for (std::size_t i = 1; i < trajectory.size(); ++i)
    arcs[i] = arcs[i - 1] + std::hypot(
        trajectory[i].x - trajectory[i - 1].x,
        trajectory[i].y - trajectory[i - 1].y);
  const auto processed = processCruiseOptimizationResult(
      initial_velocity_mps, raw_result, arcs[ego_index]);
  if (!processed) return std::nullopt;
  Trajectory output = trajectory;
  if (processed->distance_m.size() == 1) {
    if (processed->velocity_mps.front() >= 0.01) return std::nullopt;
    output[ego_index].longitudinal_velocity_mps = initial_velocity_mps;
    for (std::size_t i = ego_index + 1; i < output.size(); ++i)
      output[i].longitudinal_velocity_mps = 0.0F;
    return output;
  }
  double stop_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < processed->velocity_mps.size(); ++i) {
    if (processed->velocity_mps[i] < 0.01) {
      stop_distance = processed->distance_m[i];
      break;
    }
  }
  for (std::size_t i = ego_index; i < trajectory.size(); ++i) {
    if (std::abs(trajectory[i].longitudinal_velocity_mps) < 1e-4) {
      stop_distance = std::min(stop_distance, arcs[i]);
      break;
    }
  }
  for (std::size_t i = 0; i < ego_index; ++i)
    output[i].longitudinal_velocity_mps = initial_velocity_mps;
  for (std::size_t i = ego_index; i < output.size(); ++i) {
    const double query = std::max(
        arcs[i], processed->distance_m.front());
    if (query > processed->distance_m.back()) {
      output[i].longitudinal_velocity_mps =
          trajectory[i].longitudinal_velocity_mps + velocity_margin_mps;
      continue;
    }
    const auto upper = std::upper_bound(
        processed->distance_m.begin(), processed->distance_m.end(), query);
    const std::size_t next = upper == processed->distance_m.begin() ? 1 :
        std::min<std::size_t>(
            std::distance(processed->distance_m.begin(), upper),
            processed->distance_m.size() - 1);
    const std::size_t previous = next - 1;
    const double length = processed->distance_m[next] -
                          processed->distance_m[previous];
    const double ratio = length < 1e-9 ? 0.0 :
        std::clamp((query - processed->distance_m[previous]) / length,
                   0.0, 1.0);
    output[i].longitudinal_velocity_mps = static_cast<float>(
        processed->velocity_mps[previous] + ratio *
        (processed->velocity_mps[next] -
         processed->velocity_mps[previous]) + velocity_margin_mps);
  }
  output.back().longitudinal_velocity_mps = 0.0F;
  if (!std::isfinite(stop_distance)) return output;
  for (std::size_t i = 0; i + 1 < output.size(); ++i) {
    if (arcs[i + 1] < stop_distance) continue;
    const double segment = arcs[i + 1] - arcs[i];
    const double ratio = segment < 1e-9 ? 0.0 :
        std::clamp((stop_distance - arcs[i]) / segment, 0.0, 1.0);
    std::size_t stop_index = i;
    if (ratio > 1e-3 && ratio < 1.0 - 1e-3) {
      auto point = output[i];
      point.x += ratio * (output[i + 1].x - output[i].x);
      point.y += ratio * (output[i + 1].y - output[i].y);
      point.z += ratio * (output[i + 1].z - output[i].z);
      output.insert(output.begin() + i + 1, point);
      stop_index = i + 1;
    } else if (ratio >= 1.0 - 1e-3) {
      stop_index = i + 1;
    }
    for (std::size_t j = stop_index; j < output.size(); ++j)
      output[j].longitudinal_velocity_mps = 0.0F;
    break;
  }
  return output;
}

JerkQpProblem buildCruiseOptimizationProblem(
    const CruiseOptimizationInput &input,
    const CruiseOptimizationWeights &weights) {
  const auto n = static_cast<Eigen::Index>(input.time_s.size());
  if (n < 2 || input.boundaries.size() != static_cast<std::size_t>(n))
    throw std::invalid_argument("순항 최적화 시각·경계 수가 유효하지 않습니다");
  const double values[] = {
      input.initial_distance_m, input.initial_velocity_mps,
      input.initial_acceleration_mps2, input.maximum_velocity_mps,
      input.maximum_acceleration_mps2, input.minimum_acceleration_mps2,
      input.hard_maximum_acceleration_mps2,
      input.hard_minimum_acceleration_mps2,
      input.maximum_jerk_mps3, input.minimum_jerk_mps3,
      input.hard_maximum_jerk_mps3, input.hard_minimum_jerk_mps3,
      input.dangerous_time_s, input.idling_time_s,
      weights.maximum_distance, weights.maximum_velocity,
      weights.safety_distance_slack, weights.ideal_distance_slack,
      weights.velocity_slack, weights.acceleration_slack,
      weights.jerk_slack};
  if (!std::all_of(std::begin(values), std::end(values),
                   [](double value) { return std::isfinite(value); }) ||
      input.maximum_velocity_mps < 0.0 ||
      input.maximum_acceleration_mps2 <= input.minimum_acceleration_mps2 ||
      input.hard_maximum_acceleration_mps2 <
          input.hard_minimum_acceleration_mps2 ||
      input.maximum_jerk_mps3 <= input.minimum_jerk_mps3 ||
      input.hard_maximum_jerk_mps3 < input.hard_minimum_jerk_mps3 ||
      input.minimum_acceleration_mps2 >= 0.0 ||
      input.dangerous_time_s < 0.0 || input.idling_time_s < 0.0 ||
      weights.maximum_distance < 0.0 || weights.maximum_velocity < 0.0 ||
      weights.safety_distance_slack < 0.0 ||
      weights.ideal_distance_slack < 0.0 ||
      weights.velocity_slack < 0.0 ||
      weights.acceleration_slack < 0.0 ||
      weights.jerk_slack < 0.0)
    throw std::invalid_argument("순항 최적화 매개변수가 유효하지 않습니다");
  for (Eigen::Index i = 0; i < n; ++i) {
    if (!std::isfinite(input.time_s[i]) ||
        !std::isfinite(input.boundaries[i].maximum_distance_m) ||
        (i > 0 && input.time_s[i] <= input.time_s[i - 1]))
      throw std::invalid_argument("순항 최적화 시각·거리 경계가 유효하지 않습니다");
  }
  constexpr Eigen::Index blocks = 9;
  const auto index = [n](Eigen::Index block, Eigen::Index i) {
    return block * n + i;
  };
  JerkQpProblem problem;
  problem.optimized_points = static_cast<std::size_t>(n);
  problem.hessian = Eigen::MatrixXd::Zero(blocks * n, blocks * n);
  problem.linear_cost = Eigen::VectorXd::Zero(blocks * n);
  problem.constraints = Eigen::MatrixXd::Zero(10 * n, blocks * n);
  problem.lower_bound = Eigen::VectorXd::Zero(10 * n);
  problem.upper_bound = Eigen::VectorXd::Zero(10 * n);
  const double vmax = std::max(input.maximum_velocity_mps, 0.1);
  const double accel_range = std::max(
      input.maximum_acceleration_mps2 - input.minimum_acceleration_mps2,
      0.1);
  const double jerk_range = std::max(
      input.maximum_jerk_mps3 - input.minimum_jerk_mps3, 0.1);
  const double ideal_coefficient = input.initial_velocity_mps /
      (2.0 * std::abs(input.minimum_acceleration_mps2)) +
      input.idling_time_s;
  const double safety_coefficient = input.initial_velocity_mps /
      (2.0 * std::abs(input.minimum_acceleration_mps2)) +
      input.dangerous_time_s;
  for (Eigen::Index i = 0; i < n; ++i) {
    const double dt = i + 1 < n ?
        input.time_s[i + 1] - input.time_s[i] :
        input.time_s[n - 1] - input.time_s[n - 2];
    const double max_distance = std::max(
        input.boundaries[i].maximum_distance_m, 0.01);
    const double distance_weight = weights.maximum_distance /
        (max_distance * max_distance) * dt;
    problem.linear_cost(index(0, i)) -=
        weights.maximum_distance / max_distance * dt;
    problem.linear_cost(index(1, i)) -=
        weights.maximum_velocity / vmax * dt;
    problem.hessian(index(0, i), index(0, i)) += distance_weight;
    problem.hessian(index(1, i), index(1, i)) +=
        weights.maximum_velocity / (vmax * vmax) * dt;
    if (input.boundaries[i].object_boundary) {
      problem.linear_cost(index(1, i)) -=
          weights.maximum_distance / max_distance * ideal_coefficient * dt;
      problem.hessian(index(1, i), index(1, i)) +=
          distance_weight * ideal_coefficient * ideal_coefficient;
      problem.hessian(index(0, i), index(1, i)) +=
          distance_weight * ideal_coefficient;
      problem.hessian(index(1, i), index(0, i)) +=
          distance_weight * ideal_coefficient;
    }
    problem.hessian(index(4, i), index(4, i)) +=
        weights.safety_distance_slack / (max_distance * max_distance) * dt;
    problem.hessian(index(5, i), index(5, i)) +=
        weights.ideal_distance_slack / (max_distance * max_distance) * dt;
    problem.hessian(index(6, i), index(6, i)) +=
        weights.velocity_slack / (vmax * vmax) * dt;
    problem.hessian(index(7, i), index(7, i)) +=
        weights.acceleration_slack / accel_range * dt;
    problem.hessian(index(8, i), index(8, i)) +=
        weights.jerk_slack / jerk_range * dt;
  }

  Eigen::Index row = 0;
  for (Eigen::Index i = 0; i < n; ++i, ++row) {
    problem.constraints(row, index(0, i)) = 1.0;
    if (input.boundaries[i].object_boundary)
      problem.constraints(row, index(1, i)) = safety_coefficient;
    problem.constraints(row, index(4, i)) = -1.0;
    problem.upper_bound(row) = input.boundaries[i].maximum_distance_m;
  }
  for (Eigen::Index i = 0; i < n; ++i, ++row) {
    problem.constraints(row, index(0, i)) = 1.0;
    if (input.boundaries[i].object_boundary)
      problem.constraints(row, index(1, i)) = ideal_coefficient;
    problem.constraints(row, index(5, i)) = -1.0;
    problem.upper_bound(row) = input.boundaries[i].maximum_distance_m;
  }
  for (Eigen::Index i = 0; i < n; ++i, ++row) {
    problem.constraints(row, index(1, i)) = 1.0;
    problem.constraints(row, index(6, i)) = -1.0;
    problem.upper_bound(row) = i + 1 == n ? 0.0 : vmax;
  }
  for (Eigen::Index i = 0; i < n; ++i, ++row) {
    problem.constraints(row, index(2, i)) = 1.0;
    problem.constraints(row, index(7, i)) = -1.0;
    problem.lower_bound(row) = i + 1 == n ? 0.0 :
        input.minimum_acceleration_mps2;
    problem.upper_bound(row) = i + 1 == n ? 0.0 :
        input.maximum_acceleration_mps2;
  }
  for (Eigen::Index i = 0; i < n; ++i, ++row) {
    problem.constraints(row, index(2, i)) = 1.0;
    problem.lower_bound(row) = input.hard_minimum_acceleration_mps2;
    problem.upper_bound(row) = input.hard_maximum_acceleration_mps2;
  }
  for (Eigen::Index i = 0; i < n; ++i, ++row) {
    problem.constraints(row, index(3, i)) = 1.0;
    problem.constraints(row, index(8, i)) = -1.0;
    problem.lower_bound(row) = i + 1 == n ? 0.0 :
        input.minimum_jerk_mps3;
    problem.upper_bound(row) = i + 1 == n ? 0.0 :
        input.maximum_jerk_mps3;
  }
  for (Eigen::Index i = 0; i < n; ++i, ++row) {
    problem.constraints(row, index(3, i)) = 1.0;
    problem.lower_bound(row) = input.hard_minimum_jerk_mps3;
    problem.upper_bound(row) = input.hard_maximum_jerk_mps3;
  }
  for (Eigen::Index i = 0; i + 1 < n; ++i, ++row) {
    const double dt = input.time_s[i + 1] - input.time_s[i];
    problem.constraints(row, index(0, i + 1)) = 1.0;
    problem.constraints(row, index(0, i)) = -1.0;
    problem.constraints(row, index(1, i)) = -dt;
    problem.constraints(row, index(2, i)) = -0.5 * dt * dt;
    problem.constraints(row, index(3, i)) = -dt * dt * dt / 6.0;
  }
  for (Eigen::Index i = 0; i + 1 < n; ++i, ++row) {
    const double dt = input.time_s[i + 1] - input.time_s[i];
    problem.constraints(row, index(1, i + 1)) = 1.0;
    problem.constraints(row, index(1, i)) = -1.0;
    problem.constraints(row, index(2, i)) = -dt;
    problem.constraints(row, index(3, i)) = -0.5 * dt * dt;
  }
  for (Eigen::Index i = 0; i + 1 < n; ++i, ++row) {
    const double dt = input.time_s[i + 1] - input.time_s[i];
    problem.constraints(row, index(2, i + 1)) = 1.0;
    problem.constraints(row, index(2, i)) = -1.0;
    problem.constraints(row, index(3, i)) = -dt;
  }
  const double initials[] = {input.initial_distance_m,
                             input.initial_velocity_mps,
                             input.initial_acceleration_mps2};
  for (Eigen::Index block = 0; block < 3; ++block, ++row) {
    problem.constraints(row, index(block, 0)) = 1.0;
    problem.lower_bound(row) = initials[block];
    problem.upper_bound(row) = initials[block];
  }
  if (row != problem.constraints.rows())
    throw std::logic_error("순항 최적화 제약 수가 일치하지 않습니다");
  return problem;
}

std::optional<CruiseOptimizationResult> optimizeCruiseVelocity(
    const CruiseOptimizationInput &input,
    const CruiseOptimizationWeights &weights,
    const JerkQpSolveOptions &options) {
  const auto problem = buildCruiseOptimizationProblem(input, weights);
  const auto solved = solveJerkQp(problem, options);
  if (!solved.solved) return std::nullopt;
  const auto n = input.time_s.size();
  CruiseOptimizationResult result;
  result.time_s = input.time_s;
  result.distance_m.resize(n);
  result.velocity_mps.resize(n);
  result.acceleration_mps2.resize(n);
  result.jerk_mps3.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    result.distance_m[i] = solved.solution(i);
    result.velocity_mps[i] = std::max(solved.solution(n + i), 0.0);
    result.acceleration_mps2[i] = solved.solution(2 * n + i);
    result.jerk_mps3[i] = solved.solution(3 * n + i);
  }
  result.velocity_mps.back() = 0.0;
  return result;
}

}  // planning 네임스페이스
