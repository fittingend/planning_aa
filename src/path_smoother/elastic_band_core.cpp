// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 원본 EBPathSmoother::updateConstraint와 결과 변환의 계산 부분입니다.

#include "planning/elastic_band_core.hpp"

#include "planning/jerk_qp_problem.hpp"
#include "planning/jerk_qp_solver.hpp"
#include "planning/jerk_resample.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace planning {
namespace {

double pointYaw(const TrajectoryPoint &point) {
  return std::atan2(
      2.0 * (point.orientation_w * point.orientation_z +
             point.orientation_x * point.orientation_y),
      1.0 - 2.0 * (point.orientation_y * point.orientation_y +
                   point.orientation_z * point.orientation_z));
}

void setYaw(TrajectoryPoint &point, double yaw) {
  point.orientation_x = 0.0;
  point.orientation_y = 0.0;
  point.orientation_z = std::sin(yaw / 2.0);
  point.orientation_w = std::cos(yaw / 2.0);
}

std::size_t nearestSegment(
    const Trajectory &path, const TrajectoryPoint &pose,
    double distance_limit, double yaw_limit) {
  const auto search = [&](bool check_distance, bool check_yaw) {
    std::size_t nearest = 0;
    double best = std::numeric_limits<double>::infinity();
    bool found = false;
    for (std::size_t i = 0; i < path.size(); ++i) {
      const double dx = pose.x - path[i].x;
      const double dy = pose.y - path[i].y;
      const double squared_distance = dx * dx + dy * dy;
      const double yaw_error = std::fabs(std::remainder(
          pointYaw(path[i]) - pointYaw(pose), 2.0 * 3.141592653589793));
      if ((check_distance &&
           squared_distance > distance_limit * distance_limit) ||
          (check_yaw && yaw_error > yaw_limit)) {
        if (found) break;
        continue;
      }
      if (squared_distance < best) {
        best = squared_distance;
        nearest = i;
        found = true;
      }
    }
    return std::pair<bool, std::size_t>{found, nearest};
  };
  auto result = search(true, true);
  if (!result.first) result = search(true, false);
  if (!result.first) result = search(false, false);
  const auto index = result.second;
  if (index == 0) return 0;
  if (index + 1 == path.size()) return path.size() - 2;
  const double dx = path[index + 1].x - path[index].x;
  const double dy = path[index + 1].y - path[index].y;
  const double signed_length =
      (dx * (pose.x - path[index].x) +
       dy * (pose.y - path[index].y));
  return signed_length <= 0.0 ? index - 1 : index;
}

double segmentOffset(
    const Trajectory &path, std::size_t segment,
    const TrajectoryPoint &pose) {
  const double dx = path[segment + 1].x - path[segment].x;
  const double dy = path[segment + 1].y - path[segment].y;
  const double length = std::hypot(dx, dy);
  if (length < 1.0e-9) return 0.0;
  return ((pose.x - path[segment].x) * dx +
          (pose.y - path[segment].y) * dy) / length;
}

double signedArcFromStart(const Trajectory &path,
                          const TrajectoryPoint &pose) {
  std::size_t nearest = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double distance = std::hypot(
        pose.x - path[i].x, pose.y - path[i].y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = i;
    }
  }
  const std::size_t segment = [&]() {
    if (nearest == 0) return std::size_t{0};
    if (nearest + 1 == path.size()) return path.size() - 2;
    return segmentOffset(path, nearest, pose) <= 0.0
               ? nearest - 1 : nearest;
  }();
  double arc = segmentOffset(path, segment, pose);
  for (std::size_t i = 0; i < segment; ++i)
    arc += std::hypot(path[i + 1].x - path[i].x,
                      path[i + 1].y - path[i].y);
  return arc;
}

double lateralOffset(
    const Trajectory &path, const TrajectoryPoint &pose,
    double distance_limit, double yaw_limit) {
  const auto segment = nearestSegment(path, pose, distance_limit, yaw_limit);
  const double dx = path[segment + 1].x - path[segment].x;
  const double dy = path[segment + 1].y - path[segment].y;
  const double length = std::hypot(dx, dy);
  if (length < 1.0e-9) return 0.0;
  return (dx * (pose.y - path[segment].y) -
          dy * (pose.x - path[segment].x)) / length;
}

std::optional<TrajectoryPoint> forwardPoint(
    const Trajectory &path, std::size_t segment, double offset) {
  for (std::size_t i = segment; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x - path[i].x;
    const double dy = path[i + 1].y - path[i].y;
    const double length = std::hypot(dx, dy);
    if (length < 1.0e-9) continue;
    if (offset <= length) {
      auto point = path[i];
      point.x += dx * offset / length;
      point.y += dy * offset / length;
      return point;
    }
    offset -= length;
  }
  return std::nullopt;
}

Trajectory resampleAtInterval(
    const Trajectory &path, double interval, bool include_stop_point) {
  Trajectory unique;
  unique.reserve(path.size());
  unique.push_back(path.front());
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (std::hypot(path[i].x - unique.back().x,
                   path[i].y - unique.back().y) >= 1.0e-9) {
      unique.push_back(path[i]);
    }
  }
  if (unique.size() < 2) return unique;
  std::vector<double> arcs{0.0};
  for (std::size_t i = 1; i < unique.size(); ++i) {
    arcs.push_back(arcs.back() + std::hypot(
        unique[i].x - unique[i - 1].x,
        unique[i].y - unique[i - 1].y));
  }
  std::vector<double> samples;
  for (double arc = 0.0; arc < arcs.back(); arc += interval) {
    samples.push_back(arc);
  }
  // 원본 motion_utils의 종점 중복 판정은 0.1m를 사용합니다.
  if (arcs.back() - samples.back() < 0.1) {
    samples.back() = arcs.back();
  } else {
    samples.push_back(arcs.back());
  }
  if (include_stop_point) {
    const auto stop = std::find_if(unique.begin(), unique.end(), [](const auto &point) {
      return std::fabs(point.longitudinal_velocity_mps) < 1.0e-4;
    });
    if (stop != unique.end()) {
      const double stop_arc = arcs[static_cast<std::size_t>(stop - unique.begin())];
      const auto next = std::upper_bound(samples.begin(), samples.end(), stop_arc);
      if (next != samples.begin() && next != samples.end()) {
        const auto previous = next - 1;
        if (stop_arc - *previous < 0.1) {
          *previous = stop_arc;
        } else if (*next - stop_arc < 0.1) {
          *next = stop_arc;
        } else {
          samples.insert(next, stop_arc);
        }
      }
    }
  }
  auto result = resampleJerkAtArcs(unique, samples);
  if (result.size() >= 2) {
    const double first_yaw = pointYaw(unique.front());
    const double first_dx = unique[1].x - unique[0].x;
    const double first_dy = unique[1].y - unique[0].y;
    const bool forward = std::cos(first_yaw) * first_dx +
                         std::sin(first_yaw) * first_dy > 0.0;
    if (forward) {
      for (std::size_t i = 0; i + 1 < result.size(); ++i)
        setYaw(result[i], std::atan2(result[i + 1].y - result[i].y,
                                     result[i + 1].x - result[i].x));
      result.back().orientation_x = result[result.size() - 2].orientation_x;
      result.back().orientation_y = result[result.size() - 2].orientation_y;
      result.back().orientation_z = result[result.size() - 2].orientation_z;
      result.back().orientation_w = result[result.size() - 2].orientation_w;
    } else {
      for (std::size_t i = result.size() - 1; i > 0; --i)
        setYaw(result[i], std::atan2(result[i - 1].y - result[i].y,
                                     result[i - 1].x - result[i].x));
      result.front().orientation_x = result[1].orientation_x;
      result.front().orientation_y = result[1].orientation_y;
      result.front().orientation_z = result[1].orientation_z;
      result.front().orientation_w = result[1].orientation_w;
    }
    if (!include_stop_point) {
      result.front().orientation_x = unique.front().orientation_x;
      result.front().orientation_y = unique.front().orientation_y;
      result.front().orientation_z = unique.front().orientation_z;
      result.front().orientation_w = unique.front().orientation_w;
    }
  }
  return result;
}

}  // 익명 네임스페이스

std::optional<ElasticBandResult> smoothElasticBandResampled(
    const Trajectory &input, bool goal_contained,
    const ElasticBandParameters &p) {
  if (input.empty() || p.num_points < 3 || p.num_points > 1000 ||
      p.num_joint_points >= p.num_points ||
      !std::isfinite(p.delta_arc_length) || p.delta_arc_length <= 0.0 ||
      !std::isfinite(p.backward_length) || p.backward_length < 0.0 ||
      !std::isfinite(p.nearest_distance) || p.nearest_distance < 0.0 ||
      !std::isfinite(p.nearest_yaw) || p.nearest_yaw < 0.0 ||
      !std::isfinite(p.clearance_for_fix) || p.clearance_for_fix < 0.0 ||
      !std::isfinite(p.clearance_for_joint) || p.clearance_for_joint < 0.0 ||
      !std::isfinite(p.clearance_for_smooth) || p.clearance_for_smooth < 0.0 ||
      !std::isfinite(p.smooth_weight) || p.smooth_weight < 0.0 ||
      !std::isfinite(p.lateral_error_weight) || p.lateral_error_weight <= 0.0 ||
      !std::isfinite(p.max_validation_error) || p.max_validation_error < 0.0) {
    throw std::invalid_argument("Elastic Band 입력 또는 설정이 잘못되었습니다");
  }
  for (const auto &point : input) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.orientation_x) ||
        !std::isfinite(point.orientation_y) ||
        !std::isfinite(point.orientation_z) ||
        !std::isfinite(point.orientation_w)) {
      throw std::invalid_argument("Elastic Band 경로 좌표가 유한하지 않습니다");
    }
  }
  if (input.size() == 1) return ElasticBandResult{input, 1, 0, 0.0};

  const auto n = static_cast<Eigen::Index>(p.num_points);
  const auto valid = std::min(input.size(), p.num_points);
  Eigen::MatrixXd difference = Eigen::MatrixXd::Zero(n - 2, n);
  for (Eigen::Index i = 0; i + 2 < n; ++i) {
    difference(i, i) = 1.0;
    difference(i, i + 1) = -2.0;
    difference(i, i + 2) = 1.0;
  }
  const Eigen::MatrixXd smooth = p.smooth_weight *
      difference.transpose() * difference;
  Eigen::MatrixXd theta = Eigen::MatrixXd::Zero(n, 2 * n);
  Eigen::VectorXd coordinates(2 * n);
  for (Eigen::Index i = 0; i < n; ++i) {
    const auto &point = input[std::min(static_cast<std::size_t>(i), valid - 1)];
    const double yaw = pointYaw(point);
    theta(i, i) = -std::sin(yaw);
    theta(i, i + n) = std::cos(yaw);
    coordinates(i) = point.x;
    coordinates(i + n) = point.y;
  }
  Eigen::MatrixXd block = Eigen::MatrixXd::Zero(2 * n, 2 * n);
  block.topLeftCorner(n, n) = smooth;
  block.bottomRightCorner(n, n) = smooth;
  const Eigen::MatrixXd theta_smooth = theta * block;

  JerkQpProblem problem;
  problem.hessian = theta_smooth * theta.transpose();
  problem.hessian.diagonal().array() += p.lateral_error_weight;
  problem.linear_cost = theta_smooth * coordinates;
  problem.constraints = Eigen::MatrixXd::Identity(n, n);
  problem.lower_bound.resize(n);
  problem.upper_bound.resize(n);
  for (Eigen::Index i = 0; i < n; ++i) {
    double clearance = p.clearance_for_smooth;
    if (i == 0 || (goal_contained &&
                   (i >= n - 2 || static_cast<std::size_t>(i) >= valid - 2))) {
      clearance = p.clearance_for_fix;
    } else if (static_cast<std::size_t>(i) <= p.num_joint_points) {
      clearance = p.clearance_for_joint;
    }
    problem.lower_bound(i) = -clearance;
    problem.upper_bound(i) = clearance;
  }
  JerkQpSolveOptions options;
  options.max_iterations = 10000;
  options.absolute_tolerance = 1.0e-7;
  options.relative_tolerance = 1.0e-7;
  const auto solution = solveJerkQp(problem, options);
  if (!solution.solved || solution.solution.size() != n) return std::nullopt;

  ElasticBandResult result;
  result.trajectory = input;
  result.trajectory.resize(valid);
  result.optimized_points = valid;
  result.qp_iterations = static_cast<std::size_t>(solution.iterations);
  result.qp_primal_residual = solution.primal_residual;
  for (std::size_t i = 0; i < valid; ++i) {
    const double offset = solution.solution(static_cast<Eigen::Index>(i));
    if (!std::isfinite(offset) ||
        (p.enable_optimization_validation &&
         std::fabs(offset) > p.max_validation_error)) return std::nullopt;
    const double yaw = pointYaw(input[i]);
    result.trajectory[i].x -= std::sin(yaw) * offset;
    result.trajectory[i].y += std::cos(yaw) * offset;
  }
  for (std::size_t i = 0; i < valid; ++i) {
    const auto &a = result.trajectory[i == valid - 1 ? i - 1 : i];
    const auto &b = result.trajectory[i == valid - 1 ? i : i + 1];
    setYaw(result.trajectory[i], std::atan2(b.y - a.y, b.x - a.x));
  }
  return result;
}

ElasticBandInput prepareElasticBandInput(
    const Trajectory &input, const TrajectoryPoint &ego,
    const Trajectory *previous, const ElasticBandParameters &p) {
  if (input.size() < 2 || p.num_points < 3 ||
      !std::isfinite(p.delta_arc_length) || p.delta_arc_length <= 0.0 ||
      !std::isfinite(p.backward_length) || p.backward_length < 0.0 ||
      !std::isfinite(p.nearest_distance) || p.nearest_distance < 0.0 ||
      !std::isfinite(p.nearest_yaw) || p.nearest_yaw < 0.0 ||
      !std::isfinite(ego.x) || !std::isfinite(ego.y) ||
      (previous && previous->empty())) {
    throw std::invalid_argument("Elastic Band 전처리 입력이 잘못되었습니다");
  }
  for (const auto &point : input) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw std::invalid_argument("Elastic Band 입력 경로 좌표가 유한하지 않습니다");
    }
  }
  const auto segment = nearestSegment(
      input, ego, p.nearest_distance, p.nearest_yaw);
  const double offset = segmentOffset(input, segment, ego);
  const double forward_length = p.num_points * p.delta_arc_length;
  double distance = -offset;
  std::size_t end = input.size();
  for (std::size_t i = segment + 1; i < input.size(); ++i) {
    distance += std::hypot(input[i].x - input[i - 1].x,
                           input[i].y - input[i - 1].y);
    if (distance > forward_length) {
      end = i;
      break;
    }
  }
  std::size_t begin = 0;
  distance = -offset;
  for (std::size_t i = segment; i > 0; --i) {
    distance -= std::hypot(input[i].x - input[i - 1].x,
                           input[i].y - input[i - 1].y);
    if (distance < -p.backward_length) {
      begin = i;
      break;
    }
  }
  if (end <= begin + 1) {
    begin = 0;
    end = input.size();
  }
  Trajectory cropped(input.begin() + begin, input.begin() + end);
  const bool goal_contained = end == input.size();
  if (previous && previous->size() >= 2) {
    const auto previous_segment = nearestSegment(
        *previous, cropped.front(), p.nearest_distance, p.nearest_yaw);
    const auto &fixed = (*previous)[previous_segment];
    if (signedArcFromStart(cropped, fixed) <= 0.0) {
      const double gap = std::hypot(
          fixed.x - cropped.front().x, fixed.y - cropped.front().y);
      if (gap < p.delta_arc_length) cropped.front() = fixed;
      else cropped.insert(cropped.begin(), fixed);
    }
  }
  return {resampleAtInterval(cropped, p.delta_arc_length, false), goal_contained};
}

Trajectory finalizeElasticBandTrajectory(
    const Trajectory &input, const Trajectory &optimized,
    const TrajectoryPoint &ego, const ElasticBandParameters &p) {
  if (input.size() < 2 || optimized.size() < 2 ||
      !std::isfinite(p.output_delta_arc_length) ||
      p.output_delta_arc_length <= 0.0 ||
      !std::isfinite(p.stop_overlap_before) ||
      p.stop_overlap_before < 0.0 ||
      !std::isfinite(ego.x) || !std::isfinite(ego.y)) {
    throw std::invalid_argument("Elastic Band 후처리 입력이 잘못되었습니다");
  }
  Trajectory motion = optimized;
  if (p.apply_input_velocity) {
    std::size_t input_segment = 0;
    for (auto &point : motion) {
      const Trajectory remaining(input.begin() + input_segment, input.end());
      input_segment += nearestSegment(
          remaining, point, p.nearest_distance, p.nearest_yaw);
      point.longitudinal_velocity_mps =
          input[input_segment].longitudinal_velocity_mps;
    }
  }

  // 원본 applyInputVelocity는 경로 확장 전에 최적화 구간 안의 첫
  // 입력 정지점을 삽입한다. 종점 바로 바깥이면 이번 주기에는 삽입하지 않는다.
  const auto stop = std::find_if(input.begin(), input.end(),
      [](const auto &point) {
        return std::fabs(point.longitudinal_velocity_mps) < 1.0e-4;
      });
  if (stop != input.end()) {
    const auto segment = nearestSegment(
        motion, *stop, p.nearest_distance, p.nearest_yaw);
    const double projection = segmentOffset(motion, segment, *stop);
    const double length = std::hypot(
        motion[segment + 1].x - motion[segment].x,
        motion[segment + 1].y - motion[segment].y);
    const double ratio = length > 1.0e-9 ? projection / length : 0.0;
    const double x = motion[segment].x +
        ratio * (motion[segment + 1].x - motion[segment].x);
    const double y = motion[segment].y +
        ratio * (motion[segment + 1].y - motion[segment].y);
    const double offset = std::hypot(stop->x - x, stop->y - y);
    if (length > 1.0e-9 && offset <= p.nearest_distance &&
        (segment + 2 < motion.size() || projection <= length)) {
      if (std::fabs(x - motion[segment].x) <= 1.0e-6 &&
          std::fabs(y - motion[segment].y) <= 1.0e-6) {
        motion[segment].longitudinal_velocity_mps = 0.0F;
      } else if (std::fabs(x - motion[segment + 1].x) <= 1.0e-6 &&
                 std::fabs(y - motion[segment + 1].y) <= 1.0e-6) {
        motion[segment + 1].longitudinal_velocity_mps = 0.0F;
      } else {
        auto point = motion[segment];
        point.x = x;
        point.y = y;
        point.longitudinal_velocity_mps = 0.0F;
        motion.insert(motion.begin() + segment + 1, point);
      }
    }
  }

  const auto joint_segment = nearestSegment(
      input, motion.back(), p.nearest_distance, p.nearest_yaw);
  double distance = -segmentOffset(input, joint_segment, motion.back());
  std::optional<std::size_t> joint_end;
  if (distance < 5.0) {
    for (std::size_t i = joint_segment + 1; i < input.size(); ++i) {
      distance += std::hypot(input[i].x - input[i - 1].x,
                             input[i].y - input[i - 1].y);
      if (distance > 5.0) joint_end = i - 1;
      if (distance > 15.0) break;
    }
  }
  if (joint_end) {
    Trajectory extension(input.begin() + *joint_end, input.end());
    if (!extension.empty()) {
      extension.front().longitudinal_velocity_mps =
          motion.back().longitudinal_velocity_mps;
      motion.insert(motion.end(), extension.begin(), extension.end());
    }
  }
  auto output = resampleAtInterval(motion, p.output_delta_arc_length, true);

  // 정지점이 출력 범위 안이면 원본 입력의 정지 위치를 별도 점으로 삽입합니다.
  if (stop != input.end() && joint_end &&
      static_cast<std::size_t>(stop - input.begin()) > joint_segment &&
      static_cast<std::size_t>(stop - input.begin()) <= *joint_end &&
      stop != input.begin() &&
      std::fabs((stop - 1)->longitudinal_velocity_mps) >= 1.0e-4) {
    const auto segment = nearestSegment(
        output, *stop, p.nearest_distance, p.nearest_yaw);
    const double projection = segmentOffset(output, segment, *stop);
    const double length = std::hypot(
        output[segment + 1].x - output[segment].x,
        output[segment + 1].y - output[segment].y);
    const double distance_to_path = std::hypot(
        stop->x - output[segment].x -
            (output[segment + 1].x - output[segment].x) * projection / length,
        stop->y - output[segment].y -
            (output[segment + 1].y - output[segment].y) * projection / length);
    if (projection >= 0.0 && projection <= length &&
        distance_to_path <= p.nearest_distance) {
      const double ratio = projection / length;
      if (projection < p.stop_overlap_before) {
        output[segment].x +=
            (output[segment + 1].x - output[segment].x) * ratio;
        output[segment].y +=
            (output[segment + 1].y - output[segment].y) * ratio;
        output[segment].longitudinal_velocity_mps = 0.0F;
      } else if (length - projection < 0.1) {
        output[segment + 1].x =
            output[segment].x +
            (output[segment + 1].x - output[segment].x) * ratio;
        output[segment + 1].y =
            output[segment].y +
            (output[segment + 1].y - output[segment].y) * ratio;
        output[segment + 1].longitudinal_velocity_mps = 0.0F;
      } else {
        auto point = output[segment];
        point.x += (output[segment + 1].x - point.x) * ratio;
        point.y += (output[segment + 1].y - point.y) * ratio;
        point.longitudinal_velocity_mps = 0.0F;
        output.insert(output.begin() + segment + 1, point);
      }
    }
  }
  if (p.force_terminal_input_stop &&
      std::fabs(input.back().longitudinal_velocity_mps) < 1.0e-4 &&
      std::hypot(output.back().x - input.back().x,
                 output.back().y - input.back().y) < 0.1) {
    output.back().longitudinal_velocity_mps = 0.0F;
  }
  bool stopped = false;
  for (auto &point : output) {
    if (std::fabs(point.longitudinal_velocity_mps) < 1.0e-4) stopped = true;
    if (stopped) point.longitudinal_velocity_mps = 0.0F;
  }
  return output;
}

ElasticBandPlanner::ElasticBandPlanner(
    ElasticBandParameters smoothing,
    ElasticBandReplanParameters replanning)
    : smoothing_(smoothing), replanning_(replanning) {
  const double values[] = {
      replanning_.around_ego_lateral_distance,
      replanning_.forward_lateral_distance,
      replanning_.forward_longitudinal_distance,
      replanning_.ego_moving_distance,
      replanning_.goal_moving_distance,
      replanning_.maximum_delta_time};
  for (double value : values) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("Elastic Band 재계획 설정이 잘못되었습니다");
    }
  }
}

void ElasticBandPlanner::reset() {
  previous_input_.clear();
  previous_optimized_.clear();
  initialized_ = false;
  previous_replan_time_ = 0.0;
  previous_time_ = 0.0;
}

void ElasticBandPlanner::seed(
    const Trajectory &previous_input,
    const Trajectory &previous_optimized,
    const TrajectoryPoint &previous_ego,
    double previous_timestamp_sec) {
  if (previous_input.size() < 2 || previous_optimized.size() < 2 ||
      !std::isfinite(previous_ego.x) ||
      !std::isfinite(previous_ego.y) ||
      !std::isfinite(previous_timestamp_sec)) {
    throw std::invalid_argument("Elastic Band 이전 상태가 잘못되었습니다");
  }
  previous_input_ = previous_input;
  previous_optimized_ = previous_optimized;
  previous_ego_ = previous_ego;
  previous_replan_time_ = previous_timestamp_sec;
  previous_time_ = previous_timestamp_sec;
  initialized_ = true;
}

ElasticBandPlan ElasticBandPlanner::run(
    const Trajectory &input, const TrajectoryPoint &ego,
    double ego_velocity, double timestamp_sec) {
  if (input.size() < 2 || !std::isfinite(ego_velocity) ||
      !std::isfinite(timestamp_sec) ||
      (initialized_ && timestamp_sec < previous_time_)) {
    throw std::invalid_argument("Elastic Band 재계획 입력이나 시각이 잘못되었습니다");
  }
  bool reset_required = !initialized_;
  if (initialized_) {
    const double previous_lateral = lateralOffset(
        previous_input_, ego, smoothing_.nearest_distance,
        smoothing_.nearest_yaw);
    const double current_lateral = lateralOffset(
        input, ego, smoothing_.nearest_distance, smoothing_.nearest_yaw);
    reset_required =
        std::fabs(previous_lateral - current_lateral) >=
            replanning_.around_ego_lateral_distance ||
        std::hypot(ego.x - previous_ego_.x,
                   ego.y - previous_ego_.y) >
            replanning_.ego_moving_distance ||
        (std::fabs(ego_velocity) <= 1.0e-3 &&
         std::hypot(input.back().x - previous_input_.back().x,
                    input.back().y - previous_input_.back().y) >=
             replanning_.goal_moving_distance);
  }
  if (reset_required) previous_optimized_.clear();
  bool replan_required = reset_required || previous_optimized_.empty() ||
      !replanning_.enabled ||
      timestamp_sec - previous_replan_time_ > replanning_.maximum_delta_time;
  if (!replan_required && initialized_) {
    const auto previous_segment = nearestSegment(
        previous_input_, ego, smoothing_.nearest_distance,
        smoothing_.nearest_yaw);
    for (double distance = 10.0;
         distance <= replanning_.forward_longitudinal_distance;
         distance += 10.0) {
      const auto forward = forwardPoint(
          previous_input_, previous_segment, distance);
      if (forward && std::fabs(lateralOffset(
                         input, *forward, smoothing_.nearest_distance,
                         smoothing_.nearest_yaw)) >
                         replanning_.forward_lateral_distance) {
        replan_required = true;
        break;
      }
    }
  }

  if (replan_required) {
    const auto prepared = prepareElasticBandInput(
        input, ego,
        previous_optimized_.empty() ? nullptr : &previous_optimized_,
        smoothing_);
    const auto result = smoothElasticBandResampled(
        prepared.trajectory, prepared.goal_contained, smoothing_);
    if (result) previous_optimized_ = result->trajectory;
    else if (previous_optimized_.empty()) previous_optimized_ = input;
    previous_replan_time_ = timestamp_sec;
  }
  ElasticBandPlan plan;
  plan.replanned = replan_required;
  plan.reset = reset_required;
  plan.optimized = previous_optimized_;
  plan.trajectory = finalizeElasticBandTrajectory(
      input, plan.optimized, ego, smoothing_);
  previous_input_ = input;
  previous_ego_ = ego;
  previous_time_ = timestamp_sec;
  initialized_ = true;
  return plan;
}

}  // planning 네임스페이스
