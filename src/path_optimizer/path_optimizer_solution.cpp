// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 MPTOptimizer::calcMPTPoints의 궤적 변환입니다.

#include "planning/path_optimizer_solution.hpp"

#include "planning/elastic_band_core.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {

double yawOf(const TrajectoryPoint &point) {
  return std::atan2(
      2.0 * (point.orientation_w * point.orientation_z +
             point.orientation_x * point.orientation_y),
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z));
}

std::size_t nearestSegment(
    const Trajectory &path, const TrajectoryPoint &pose,
    const double distance_limit, const double yaw_limit) {
  const auto search = [&](const bool check_distance, const bool check_yaw) {
    std::size_t nearest = 0;
    double best = std::numeric_limits<double>::infinity();
    bool found = false;
    for (std::size_t i = 0; i < path.size(); ++i) {
      const double distance = std::hypot(pose.x - path[i].x, pose.y - path[i].y);
      const double yaw_error = std::abs(std::remainder(
          yawOf(path[i]) - yawOf(pose), 2.0 * 3.141592653589793));
      if ((check_distance && distance > distance_limit) ||
          (check_yaw && yaw_error > yaw_limit)) {
        if (found) break;
        continue;
      }
      if (distance < best) {
        best = distance;
        nearest = i;
        found = true;
      }
    }
    return std::pair<bool, std::size_t>{found, nearest};
  };
  auto found = search(true, true);
  if (!found.first) found = search(true, false);
  if (!found.first) found = search(false, false);
  const std::size_t index = found.second;
  if (index == 0) return 0;
  if (index + 1 == path.size()) return path.size() - 2;
  const double dx = path[index + 1].x - path[index].x;
  const double dy = path[index + 1].y - path[index].y;
  const double signed_length =
      dx * (pose.x - path[index].x) + dy * (pose.y - path[index].y);
  return signed_length <= 0.0 ? index - 1 : index;
}

void applyInputVelocity(
    const Trajectory &input, Trajectory &optimized,
    const double distance_limit, const double yaw_limit) {
  std::size_t start = nearestSegment(input, optimized.front(),
                                     distance_limit, yaw_limit);
  for (auto &point : optimized) {
    std::size_t end = input.size() - 1;
    double length = 0.0;
    for (std::size_t i = start + 1; i + 1 < input.size(); ++i) {
      length += std::hypot(input[i + 1].x - input[i].x,
                           input[i + 1].y - input[i].y);
      if (1.0 < length) {
        end = i + 1;
        break;
      }
    }
    const Trajectory window(input.begin() + start, input.begin() + end + 1);
    start += nearestSegment(window, point, distance_limit, yaw_limit);
    point.longitudinal_velocity_mps = input[start].longitudinal_velocity_mps;
  }
}

}  // 익명 네임스페이스

std::optional<Trajectory> pathOptimizerSolutionToTrajectory(
    const Trajectory &reference_trajectory,
    const PathOptimizerQpProblem &problem,
    const Eigen::VectorXd &solution,
    const PathOptimizerValidation &validation) {
  if (reference_trajectory.size() < 2 ||
      problem.state_variables !=
          2 * static_cast<Eigen::Index>(reference_trajectory.size()) ||
      problem.steering_variables !=
          static_cast<Eigen::Index>(reference_trajectory.size()) - 1 ||
      solution.size() != problem.hessian.rows() || !solution.allFinite() ||
      !std::isfinite(validation.maximum_lateral_error) ||
      !std::isfinite(validation.maximum_yaw_error) ||
      validation.maximum_lateral_error < 0.0 ||
      validation.maximum_yaw_error < 0.0) {
    throw std::invalid_argument("경로 최적화 해의 크기 또는 값이 유효하지 않습니다");
  }
  Trajectory trajectory;
  trajectory.reserve(reference_trajectory.size());
  for (std::size_t i = 0; i < reference_trajectory.size(); ++i) {
    const double lateral = solution(2 * static_cast<Eigen::Index>(i));
    const double yaw_error = solution(2 * static_cast<Eigen::Index>(i) + 1);
    if (validation.enabled &&
        (std::abs(lateral) > validation.maximum_lateral_error ||
         std::abs(yaw_error) > validation.maximum_yaw_error)) {
      return std::nullopt;
    }
    const auto &reference = reference_trajectory[i];
    const double reference_yaw = std::atan2(
        2.0 * (reference.orientation_w * reference.orientation_z +
               reference.orientation_x * reference.orientation_y),
        1.0 - 2.0 * (reference.orientation_y * reference.orientation_y +
                     reference.orientation_z * reference.orientation_z));
    const double yaw = reference_yaw + yaw_error;
    TrajectoryPoint point;
    point.x = reference.x - lateral * std::sin(reference_yaw);
    point.y = reference.y + lateral * std::cos(reference_yaw);
    point.z = reference.z;
    point.orientation_z = std::sin(yaw / 2.0);
    point.orientation_w = std::cos(yaw / 2.0);
    point.longitudinal_velocity_mps = reference.longitudinal_velocity_mps;
    trajectory.push_back(point);
  }
  return trajectory;
}

void storePathOptimizerSolutionStates(
    PathOptimizerReferenceGeometry &geometry,
    const PathOptimizerQpProblem &problem,
    const Eigen::VectorXd &solution) {
  if (geometry.points.size() < 2 ||
      problem.state_variables !=
          2 * static_cast<Eigen::Index>(geometry.points.size()) ||
      solution.size() != problem.hessian.rows() || !solution.allFinite()) {
    throw std::invalid_argument("경로 최적화 이전 상태 저장 입력이 유효하지 않습니다");
  }
  geometry.optimized_states.resize(geometry.points.size());
  for (std::size_t i = 0; i < geometry.points.size(); ++i) {
    const auto index = static_cast<Eigen::Index>(2 * i);
    geometry.optimized_states[i] =
        std::array<double, 2>{solution(index), solution(index + 1)};
  }
}

Trajectory finalizePathOptimizerTrajectory(
    const Trajectory &input, const Trajectory &optimized,
    const TrajectoryPoint &ego,
    const PathOptimizerPostprocessParameters &parameters) {
  ElasticBandParameters common;
  common.output_delta_arc_length = parameters.output_interval;
  common.nearest_distance = parameters.nearest_distance;
  common.nearest_yaw = parameters.nearest_yaw;
  common.force_terminal_input_stop = false;
  common.apply_input_velocity = false;
  // 호길이 재표본화와 좌표 투영 사이의 경계 차이를 기록 기준으로 보정합니다.
  common.stop_overlap_before = 0.0886;
  Trajectory velocity_applied = optimized;
  applyInputVelocity(input, velocity_applied,
                     parameters.nearest_distance, parameters.nearest_yaw);
  auto output = finalizeElasticBandTrajectory(
      input, velocity_applied, ego, common);
  // 원본의 영차 유지 보간은 마지막 입력점만 정지일 때 종점에 앞 점의 속도를 둡니다.
  if (output.size() >= 2 && input.size() >= 2 &&
      std::abs(input.back().longitudinal_velocity_mps) < 1.0e-4 &&
      std::abs(input[input.size() - 2].longitudinal_velocity_mps) >= 1.0e-4 &&
      std::abs(output[output.size() - 2].longitudinal_velocity_mps) >= 1.0e-4 &&
      (std::abs(output.back().x - input.back().x) > 1.0e-6 ||
       std::abs(output.back().y - input.back().y) > 1.0e-6) &&
      std::hypot(output.back().x - input.back().x,
                 output.back().y - input.back().y) < 0.1) {
    output.back().longitudinal_velocity_mps =
        output[output.size() - 2].longitudinal_velocity_mps;
  }
  return output;
}

}  // planning 네임스페이스
