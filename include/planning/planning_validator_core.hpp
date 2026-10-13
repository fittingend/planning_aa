// Copyright 2021, 2022 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// autoware_planning_validator의 계산과 발행 상태를 이관했습니다.

#pragma once

#include "planning/trajectory_gate.hpp"
#include "planning/trajectory_precheck.hpp"
#include "planning/planning_validator_diagnostics.hpp"

#include <cstddef>
#include <optional>

namespace planning {

struct ValidationOutcome {
  bool data_ready = false;
  TrajectoryValidationResult checks;
  std::size_t consecutive_invalid_count = 0;
  std::vector<ValidationDiagnostic> diagnostics;
  std::optional<Trajectory> selected_trajectory;
};

class PlanningValidatorCore {
public:
  PlanningValidatorCore(
      PrecheckThresholds thresholds, InvalidTrajectoryPolicy policy,
      std::size_t diagnostic_error_count_threshold = 0);

  ValidationOutcome onTrajectory(const Trajectory &trajectory, const std::optional<EgoState> &ego);

  // 기록이 노드 실행 중간부터 시작할 때 이전 진단 수치를 복원합니다.
  void seedObservedValues(
      double longitudinal_distance_deviation,
      double forward_length_required_m,
      double forward_length_measured_m);

private:
  PrecheckThresholds thresholds_;
  TrajectoryGate<Trajectory> gate_;
  std::size_t consecutive_invalid_count_ = 0;
  std::size_t diagnostic_error_count_threshold_ = 0;
  TrajectoryValidationResult last_checks_;
};

}  // planning 네임스페이스
