#pragma once

// 장애물 순항의 선택적 시간·거리·속도 최적화 문제를 ROS 없이 구성합니다.

#include "planning/jerk_qp_problem.hpp"
#include "planning/jerk_qp_solver.hpp"

#include <optional>
#include <vector>

namespace planning {

struct CruiseOptimizationBoundary {
  double maximum_distance_m = 0.0;
  bool object_boundary = false;
};

struct CruiseOptimizationWeights {
  double maximum_distance = 100.0;
  double maximum_velocity = 1.0;
  double safety_distance_slack = 1000000.0;
  double ideal_distance_slack = 50.0;
  double velocity_slack = 500000.0;
  double acceleration_slack = 5000.0;
  double jerk_slack = 10000.0;
};

struct CruiseOptimizationInput {
  std::vector<double> time_s;
  std::vector<CruiseOptimizationBoundary> boundaries;
  double initial_distance_m = 0.0;
  double initial_velocity_mps = 0.0;
  double initial_acceleration_mps2 = 0.0;
  double maximum_velocity_mps = 10.0;
  double maximum_acceleration_mps2 = 1.0;
  double minimum_acceleration_mps2 = -1.0;
  double hard_maximum_acceleration_mps2 = 1.0;
  double hard_minimum_acceleration_mps2 = -2.5;
  double maximum_jerk_mps3 = 1.0;
  double minimum_jerk_mps3 = -1.0;
  double hard_maximum_jerk_mps3 = 1.0;
  double hard_minimum_jerk_mps3 = -1.0;
  double dangerous_time_s = 0.5;
  double idling_time_s = 2.0;
};

struct CruiseOptimizationResult {
  std::vector<double> time_s;
  std::vector<double> distance_m;
  std::vector<double> velocity_mps;
  std::vector<double> acceleration_mps2;
  std::vector<double> jerk_mps3;
};

struct CruiseOptimizationCollision {
  double distance_from_ego_front_m = 0.0;
  double lateral_distance_m = 0.0;
  double time_from_now_s = 0.0;
};

struct CruiseOptimizationObstacle {
  double velocity_mps = 0.0;
  std::vector<CruiseOptimizationCollision> collision_points;
};

struct CruiseOptimizationBoundaryParameters {
  double safe_distance_margin_m = 6.0;
  double minimum_object_acceleration_mps2 = -1.0;
  double vehicle_half_width_m = 0.948;
};

struct CruiseOptimizationInitialMotion {
  double velocity_mps = 0.0;
  double acceleration_mps2 = 0.0;
};

struct CruiseOptimizationEngageParameters {
  double replan_velocity_deviation_mps = 5.0;
  double engage_velocity_mps = 0.25;
  double engage_acceleration_mps2 = 0.1;
  double engage_exit_ratio = 0.5;
  double stop_distance_to_prohibit_engage_m = 0.5;
};

CruiseOptimizationInitialMotion cruiseOptimizationInitialMotion(
    double ego_velocity_mps, double target_velocity_mps,
    std::optional<CruiseOptimizationInitialMotion> previous_motion,
    std::optional<double> distance_to_forward_stop_m,
    const CruiseOptimizationEngageParameters &parameters = {});

std::vector<double> cruiseOptimizationTimeVector(
    double dense_interval_s = 0.2, double sparse_interval_s = 2.0,
    double dense_horizon_s = 5.0, double maximum_horizon_s = 25.0);

std::vector<CruiseOptimizationBoundary> cruiseOptimizationBoundaries(
    const std::vector<double> &time_s, double trajectory_length_m,
    const std::vector<CruiseOptimizationObstacle> &obstacles,
    const CruiseOptimizationBoundaryParameters &parameters = {});

// QP의 정차 이후를 영속도로 채우고 비진행 점을 정리합니다.
std::optional<CruiseOptimizationResult> processCruiseOptimizationResult(
    double initial_velocity_mps, const CruiseOptimizationResult &result,
    double trajectory_front_offset_m);

// 최적화 속도를 입력 경로 호길이에 보간하고 가장 가까운 정지점을 삽입합니다.
std::optional<Trajectory> applyCruiseOptimizationResult(
    const Trajectory &trajectory, std::size_t ego_index,
    double initial_velocity_mps,
    const CruiseOptimizationResult &raw_result,
    double velocity_margin_mps = 0.2);

// 변수 블록은 거리·속도·가속도·저크와 다섯 개의 완화 변수 순서입니다.
JerkQpProblem buildCruiseOptimizationProblem(
    const CruiseOptimizationInput &input,
    const CruiseOptimizationWeights &weights = {});

std::optional<CruiseOptimizationResult> optimizeCruiseVelocity(
    const CruiseOptimizationInput &input,
    const CruiseOptimizationWeights &weights = {},
    const JerkQpSolveOptions &options = {});

}  // planning 네임스페이스
