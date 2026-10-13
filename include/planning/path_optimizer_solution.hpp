// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// 경로 최적화 QP 해를 궤적 좌표로 되돌립니다.

#pragma once

#include "planning/path_optimizer_qp.hpp"
#include "planning/path_optimizer_reference_geometry.hpp"
#include "planning/trajectory.hpp"

#include <Eigen/Core>

#include <optional>

namespace planning {

struct PathOptimizerValidation {
  bool enabled = false;
  double maximum_lateral_error = 5.0;
  double maximum_yaw_error = 1.046;
};

// 원본 calcMPTPoints처럼 각 기준점의 횡방향·방향 오차를 적용합니다.
// 원본 출력이 보존하는 종방향 속도와 높이만 복사합니다.
std::optional<Trajectory> pathOptimizerSolutionToTrajectory(
    const Trajectory &reference_trajectory,
    const PathOptimizerQpProblem &problem,
    const Eigen::VectorXd &solution,
    const PathOptimizerValidation &validation = {});

// 풀이 결과의 횡방향·방향 오차를 다음 주기의 고정 상태로 저장합니다.
void storePathOptimizerSolutionStates(
    PathOptimizerReferenceGeometry &geometry,
    const PathOptimizerQpProblem &problem,
    const Eigen::VectorXd &solution);

struct PathOptimizerPostprocessParameters {
  double output_interval = 0.5;
  double nearest_distance = 3.0;
  double nearest_yaw = 1.046;
};

// 입력 속도와 정지점을 적용하고 최적화 뒤의 원래 경로를 이어 붙입니다.
Trajectory finalizePathOptimizerTrajectory(
    const Trajectory &input, const Trajectory &optimized,
    const TrajectoryPoint &ego,
    const PathOptimizerPostprocessParameters &parameters = {});

}  // planning 네임스페이스
