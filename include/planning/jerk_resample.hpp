// Copyright 2021 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 기본 JerkFiltered 스무더의 최적화용 재샘플링입니다.

#pragma once

#include "planning/trajectory.hpp"


namespace planning {

struct JerkResampleParameters {
  double max_trajectory_length = 200.0;
  double min_trajectory_length = 180.0;
  double resample_time = 2.0;
  double dense_resample_dt = 0.2;
  double dense_min_interval_distance = 0.1;
  double sparse_resample_dt = 0.5;
  double sparse_min_interval_distance = 4.0;
};

// JerkFilteredSmoother::apply가 사용하는 재샘플링 경로를 따릅니다. 현재 위치는
// 첫 입력점이며, 속도와 가속도에는 영차 유지를 적용하고 정지점과 종점을 보존합니다.
Trajectory resampleJerkTrajectory(
    const Trajectory &input, double initial_velocity,
    const JerkResampleParameters &parameters = {});

// 원본 노드가 스무더 호출 전에 사용하는 위치 기준 재샘플링입니다.
Trajectory resampleJerkNodeInput(
    const Trajectory &input, const TrajectoryPoint &current_pose,
    double nearest_distance, double nearest_yaw,
    const JerkResampleParameters &parameters, double nominal_ds);

// 원본 노드가 스무더 호출 후에 사용하는 시간 기준 재샘플링입니다.
Trajectory resampleJerkNodeOutput(
    const Trajectory &input, double current_velocity,
    const TrajectoryPoint &current_pose, double nearest_distance,
    double nearest_yaw, const JerkResampleParameters &parameters,
    bool preserve_velocity_restarts = false);

// 횡가속도 필터가 사용하는 고정 거리 간격의 재표본화입니다.
Trajectory resampleJerkUniform(const Trajectory &input, double interval);

// 차로 ID 경계점처럼 간격 격자에서 옮긴 호길이에 경로를 재표본화합니다.
Trajectory resampleJerkAtArcs(
    const Trajectory &input, const std::vector<double> &output_arcs);


}  // planning 네임스페이스
