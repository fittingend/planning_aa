// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 경로 평활화의 Elastic Band 최적화 계산입니다.

#pragma once

#include "planning/trajectory.hpp"

#include <cstddef>
#include <optional>

namespace planning {

struct ElasticBandParameters {
  double delta_arc_length = 1.0;
  double backward_length = 5.0;
  double nearest_distance = 3.0;
  double nearest_yaw = 1.046;
  double output_delta_arc_length = 0.5;
  std::size_t num_points = 100;
  std::size_t num_joint_points = 3;
  double clearance_for_fix = 0.0;
  double clearance_for_joint = 0.1;
  double clearance_for_smooth = 0.1;
  double smooth_weight = 1.0;
  double lateral_error_weight = 0.001;
  double max_validation_error = 3.0;
  bool enable_optimization_validation = false;
  // 최종 입력 정지점이 별도 재표본화에서 누락됐을 때 종점 속도를 보정합니다.
  bool force_terminal_input_stop = false;
  bool apply_input_velocity = true;
  // 입력 정지점을 기존 출력 점으로 합칠 때 앞 점과 비교할 거리입니다.
  double stop_overlap_before = 0.1;
};

struct ElasticBandReplanParameters {
  bool enabled = true;
  double around_ego_lateral_distance = 2.0;
  double forward_lateral_distance = 0.2;
  double forward_longitudinal_distance = 100.0;
  double ego_moving_distance = 5.0;
  double goal_moving_distance = 15.0;
  double maximum_delta_time = 0.0;
};

struct ElasticBandInput {
  Trajectory trajectory;
  bool goal_contained = false;
};

struct ElasticBandResult {
  Trajectory trajectory;
  std::size_t optimized_points = 0;
  std::size_t qp_iterations = 0;
  double qp_primal_residual = 0.0;
};

// 차량 앞뒤 자르기와 1m 간격 재표본화를 마친 경로를 입력받습니다.
// 첫 점을 고정하고 목표점이 포함되면 끝의 두 점도 고정합니다.
// QP가 풀리지 않거나 결과가 유효하지 않으면 결과를 반환하지 않습니다.
std::optional<ElasticBandResult> smoothElasticBandResampled(
    const Trajectory &input, bool goal_contained,
    const ElasticBandParameters &parameters = {});

// 원본 노드의 최적화 앞단처럼 차량 주변을 자르고 일정 거리로 재표본화합니다.
// previous에는 같은 모듈이 직전에 최적화한 경로를 넣을 수 있습니다.
ElasticBandInput prepareElasticBandInput(
    const Trajectory &input, const TrajectoryPoint &ego,
    const Trajectory *previous = nullptr,
    const ElasticBandParameters &parameters = {});

// 최적화 경로에 입력 속도를 다시 적용하고 나머지 경로를 이어 붙입니다.
// 원본 노드의 후처리 단계이며 지도 경계와 ROS 발행은 포함하지 않습니다.
Trajectory finalizeElasticBandTrajectory(
    const Trajectory &input, const Trajectory &optimized,
    const TrajectoryPoint &ego,
    const ElasticBandParameters &parameters = {});

struct ElasticBandPlan {
  Trajectory trajectory;
  Trajectory optimized;
  bool replanned = false;
  bool reset = false;
};

// 호출자가 제공하는 단조 증가 시각으로 재계획과 이전 경로 고정을 관리합니다.
class ElasticBandPlanner {
 public:
  explicit ElasticBandPlanner(
      ElasticBandParameters smoothing = {},
      ElasticBandReplanParameters replanning = {});
  ElasticBandPlan run(
      const Trajectory &input, const TrajectoryPoint &ego,
      double ego_velocity, double timestamp_sec);
  // 기록 중간부터 재생할 때만 직전 원본 내부 상태를 초기값으로 넣습니다.
  void seed(
      const Trajectory &previous_input,
      const Trajectory &previous_optimized,
      const TrajectoryPoint &previous_ego,
      double previous_timestamp_sec);
  void reset();

 private:
  ElasticBandParameters smoothing_;
  ElasticBandReplanParameters replanning_;
  Trajectory previous_input_;
  Trajectory previous_optimized_;
  TrajectoryPoint previous_ego_;
  double previous_replan_time_ = 0.0;
  double previous_time_ = 0.0;
  bool initialized_ = false;
};

}  // planning 네임스페이스
