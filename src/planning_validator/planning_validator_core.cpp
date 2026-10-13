// Copyright 2021, 2022 Tier IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "planning/planning_validator_core.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace planning {

PlanningValidatorCore::PlanningValidatorCore(
    PrecheckThresholds thresholds, InvalidTrajectoryPolicy policy,
    std::size_t diagnostic_error_count_threshold)
    : thresholds_(std::move(thresholds)), gate_(policy),
      diagnostic_error_count_threshold_(diagnostic_error_count_threshold) {}

void PlanningValidatorCore::seedObservedValues(
    double longitudinal_distance_deviation,
    double forward_length_required_m,
    double forward_length_measured_m) {
  if (!std::isfinite(longitudinal_distance_deviation) ||
      !std::isfinite(forward_length_required_m) ||
      !std::isfinite(forward_length_measured_m)) {
    throw std::invalid_argument("복원할 validator 진단 수치가 잘못되었습니다");
  }
  last_checks_.ego.longitudinal_distance_deviation.observed =
      longitudinal_distance_deviation;
  last_checks_.ego.forward_length_required_m = forward_length_required_m;
  last_checks_.ego.forward_length.observed = forward_length_measured_m;
}

ValidationOutcome PlanningValidatorCore::onTrajectory(
    const Trajectory &trajectory, const std::optional<EgoState> &ego) {
  ValidationOutcome outcome;
  outcome.data_ready = ego.has_value();
  if (!outcome.data_ready) {
    outcome.consecutive_invalid_count = consecutive_invalid_count_;
    return outcome;
  }

  auto evaluated = validateTrajectory(trajectory, *ego, thresholds_);
  if (!evaluated.local.size.passed) {
    // 원본 validator는 즉시 반환하며 다른 상태 필드는 이전에 발행한 값을 유지합니다.
    last_checks_.local.size = evaluated.local.size;
  } else if (!evaluated.local.finite.passed) {
    last_checks_.local.size = evaluated.local.size;
    last_checks_.local.finite = evaluated.local.finite;
  } else {
    const auto nearest = evaluated.ego.velocity_deviation.index;
    if (nearest > 0 && nearest + 1 < trajectory.size()) {
      // 원본은 내부 최근접점에서 편차 수치를 다시 쓰지 않습니다.
      evaluated.ego.longitudinal_distance_deviation.observed =
          last_checks_.ego.longitudinal_distance_deviation.observed;
    }
    if (std::abs(ego->longitudinal_velocity_mps) < 1.0 / 3.6) {
      // 정차에 가까우면 전방 길이 검사 상태만 갱신하고 이전 수치를 유지합니다.
      evaluated.ego.forward_length.observed = last_checks_.ego.forward_length.observed;
      evaluated.ego.forward_length_required_m =
          last_checks_.ego.forward_length_required_m;
    }
    last_checks_ = evaluated;
  }
  outcome.checks = last_checks_;
  const bool valid = outcome.checks.allValid();
  consecutive_invalid_count_ = valid ? 0 : consecutive_invalid_count_ + 1;
  outcome.consecutive_invalid_count = consecutive_invalid_count_;
  outcome.diagnostics = makeValidationDiagnostics(
      outcome.checks, consecutive_invalid_count_,
      diagnostic_error_count_threshold_);
  outcome.selected_trajectory = gate_.select(trajectory, true, valid);
  return outcome;
}

}  // planning 네임스페이스
