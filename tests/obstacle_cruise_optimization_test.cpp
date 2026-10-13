// SPDX-License-Identifier: Apache-2.0
// 순항 시간·거리 QP의 객체 경계와 운동 방정식 및 작은 풀이를 확인합니다.

#include "planning/obstacle_cruise_optimization.hpp"

#include <cmath>
#include <stdexcept>

int main() {
  planning::CruiseOptimizationInput input;
  input.time_s = {0.0, 1.0, 2.0};
  input.boundaries = {{100.0, false}, {8.0, true}, {8.0, true}};
  input.initial_velocity_mps = 2.0;
  input.maximum_velocity_mps = 4.0;
  input.maximum_jerk_mps3 = 2.0;
  input.minimum_jerk_mps3 = -2.0;
  input.hard_maximum_jerk_mps3 = 2.0;
  input.hard_minimum_jerk_mps3 = -2.0;
  const auto problem = planning::buildCruiseOptimizationProblem(input);
  const auto near = [](double a, double b) {
    return std::abs(a - b) < 1e-12;
  };
  if (problem.hessian.rows() != 27 || problem.constraints.rows() != 30 ||
      problem.constraints.cols() != 27 ||
      !near(problem.constraints(1, 1), 1.0) ||
      !near(problem.constraints(1, 4), 1.5) ||
      !near(problem.constraints(1, 13), -1.0) ||
      !near(problem.constraints(21, 1), 1.0) ||
      !near(problem.constraints(21, 0), -1.0) ||
      !near(problem.constraints(21, 3), -1.0) ||
      !near(problem.constraints(21, 6), -0.5) ||
      !near(problem.constraints(21, 9), -1.0 / 6.0) ||
      !near(problem.lower_bound(27), 0.0) ||
      !near(problem.lower_bound(28), 2.0))
    throw std::runtime_error("순항 최적화 목적 함수나 제약 행렬이 잘못되었습니다");
  const auto time = planning::cruiseOptimizationTimeVector();
  if (time.size() != 36 || !near(time[25], 5.0) ||
      !near(time.back(), 25.0))
    throw std::runtime_error("순항 최적화 시간 격자가 잘못되었습니다");
  const auto first_motion = planning::cruiseOptimizationInitialMotion(
      2.0, 4.0, std::nullopt, std::nullopt);
  const auto replan_motion = planning::cruiseOptimizationInitialMotion(
      8.0, 4.0,
      planning::CruiseOptimizationInitialMotion{2.0, -0.4},
      std::nullopt);
  const auto engage_motion = planning::cruiseOptimizationInitialMotion(
      0.0, 4.0,
      planning::CruiseOptimizationInitialMotion{0.0, 0.0},
      1.0);
  const auto held_motion = planning::cruiseOptimizationInitialMotion(
      0.0, 4.0,
      planning::CruiseOptimizationInitialMotion{0.0, -0.2},
      0.2);
  if (!near(first_motion.velocity_mps, 2.0) ||
      !near(replan_motion.acceleration_mps2, -0.4) ||
      !near(replan_motion.velocity_mps, 8.0) ||
      !near(engage_motion.velocity_mps, 0.25) ||
      !near(held_motion.velocity_mps, 0.0))
    throw std::runtime_error("순항 최적화 초기 운동 상태 선택이 잘못되었습니다");
  const planning::CruiseOptimizationObstacle on_object{
      2.0, {{20.0, 0.0, 0.0}}};
  const auto on_boundaries = planning::cruiseOptimizationBoundaries(
      input.time_s, 100.0, {on_object});
  if (!near(on_boundaries[0].maximum_distance_m, 16.0) ||
      !near(on_boundaries[1].maximum_distance_m, 18.0) ||
      !on_boundaries[2].object_boundary)
    throw std::runtime_error("경로 안 객체의 거리 경계가 잘못되었습니다");
  const planning::CruiseOptimizationObstacle off_object{
      0.0, {{8.0, 3.0, 1.5}}};
  const auto off_boundaries = planning::cruiseOptimizationBoundaries(
      input.time_s, 100.0, {off_object});
  if (!near(off_boundaries[0].maximum_distance_m, 2.0) ||
      !near(off_boundaries[1].maximum_distance_m, 2.0) ||
      !near(off_boundaries[2].maximum_distance_m, 100.0))
    throw std::runtime_error("경로 밖 객체의 시간 경계가 잘못되었습니다");
  planning::CruiseOptimizationResult raw;
  raw.time_s = {0.0, 1.0, 2.0};
  raw.distance_m = {0.0, 2.0, 3.0};
  raw.velocity_mps = {2.0, 1.0, 0.0};
  raw.acceleration_mps2 = {0.0, -1.0, 0.0};
  raw.jerk_mps3 = {-1.0, 1.0, 0.0};
  const auto processed = planning::processCruiseOptimizationResult(
      2.0, raw, 10.0);
  if (!processed || processed->distance_m.size() != 3 ||
      !near(processed->distance_m.back(), 13.0) ||
      processed->velocity_mps.back() != 0.0)
    throw std::runtime_error("순항 최적화 정차 후처리가 잘못되었습니다");
  planning::Trajectory path(6);
  for (std::size_t i = 0; i < path.size(); ++i) {
    path[i].x = static_cast<double>(i);
    path[i].longitudinal_velocity_mps = 4.0F;
  }
  raw.distance_m.back() = 2.5;
  const auto applied = planning::applyCruiseOptimizationResult(
      path, 0, 2.0, raw);
  if (!applied || applied->size() != path.size() + 1 ||
      !near((*applied)[3].x, 2.5) ||
      (*applied)[3].longitudinal_velocity_mps != 0.0F ||
      applied->back().longitudinal_velocity_mps != 0.0F)
    throw std::runtime_error("순항 최적화 속도·정지점의 경로 적용이 잘못되었습니다");
#ifdef PLANNING_HAS_PROXQP
  planning::CruiseOptimizationWeights weights;
  weights.safety_distance_slack = 100.0;
  weights.velocity_slack = 100.0;
  weights.acceleration_slack = 10.0;
  weights.jerk_slack = 10.0;
  const auto result = planning::optimizeCruiseVelocity(input, weights);
  if (!result || result->time_s.size() != 3 ||
      std::abs(result->velocity_mps.front() - 2.0) > 0.01 ||
      result->velocity_mps.back() != 0.0 ||
      result->distance_m[2] < result->distance_m[1])
    throw std::runtime_error("순항 최적화의 작은 문제 풀이가 잘못되었습니다");
#endif
}
