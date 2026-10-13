#include "planning/planning_validator_core.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>

namespace {
void check(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}
}  // 익명 네임스페이스

int main() {
  const planning::Trajectory valid{{0.0, 0.0, 10.0F, 0.0F},
                                   {10.0, 0.0, 10.0F, 0.0F},
                                   {20.0, 0.0, 10.0F, 0.0F}};
  const planning::Trajectory invalid{{0.0, 0.0, 10.0F, 0.0F}};
  const planning::Trajectory corner{{0.0, 0.0, 2.0F, 0.0F},
                                    {2.0, 0.0, 2.0F, 0.0F},
                                    {2.0, 2.0, 2.0F, 0.0F}};
  const planning::EgoState ego{0.0, 0.0, 10.0};

  planning::PlanningValidatorCore as_is(
      {}, planning::InvalidTrajectoryPolicy::PublishAsIs);
  const auto waiting = as_is.onTrajectory(valid, std::nullopt);
  check(!waiting.data_ready && !waiting.selected_trajectory,
        "validator waits for ego state before evaluating or publishing");

  auto outcome = as_is.onTrajectory(valid, ego);
  check(outcome.checks.allValid() && outcome.selected_trajectory &&
            outcome.consecutive_invalid_count == 0,
        "valid trajectory is selected with zero invalid count");
  outcome = as_is.onTrajectory(invalid, ego);
  check(!outcome.checks.allValid() && outcome.selected_trajectory &&
            outcome.selected_trajectory->size() == 1 &&
            outcome.consecutive_invalid_count == 1,
        "sample decision policy publishes invalid trajectory and counts it");
  check(outcome.checks.local.finite.passed &&
            outcome.checks.local.interval.observed == 10.0 &&
            outcome.checks.ego.forward_length.observed == 20.0,
        "early size failure retains previously published status metrics");
  outcome = as_is.onTrajectory(corner, ego);
  check(outcome.consecutive_invalid_count == 2 &&
            !outcome.checks.ego.forward_length.passed,
        "reference corner increments invalid count due to short forward length");
  check(std::abs(outcome.checks.local.relative_angle.observed - 1.5707963267948966) < 1e-10 &&
            std::abs(outcome.checks.local.curvature.observed - 0.7071067811865475) < 1e-10 &&
            std::abs(outcome.checks.local.lateral_acceleration.observed - 2.82842712474619) <
                1e-10 &&
            outcome.checks.ego.forward_length.observed == 4.0,
        "corner metrics match the reference-image status fixture");
  outcome = as_is.onTrajectory(valid, ego);
  check(outcome.consecutive_invalid_count == 0, "valid result resets invalid count");
  outcome = as_is.onTrajectory(valid, planning::EgoState{-0.5, 0.0, 10.0});
  check(std::abs(outcome.checks.ego.longitudinal_distance_deviation.observed + 0.5) < 1e-9,
        "시작점 앞 차량의 종방향 편차를 기록합니다");
  outcome = as_is.onTrajectory(valid, planning::EgoState{10.0, 0.0, 10.0});
  check(std::abs(outcome.checks.ego.longitudinal_distance_deviation.observed + 0.5) < 1e-9,
        "내부 최근접점에서는 이전 종방향 편차 수치를 유지합니다");
  const double measured_before_stop = outcome.checks.ego.forward_length.observed;
  const double required_before_stop = outcome.checks.ego.forward_length_required_m;
  outcome = as_is.onTrajectory(valid, planning::EgoState{10.0, 0.0, 0.0});
  check(outcome.checks.ego.forward_length.passed &&
            outcome.checks.ego.forward_length.observed == measured_before_stop &&
            outcome.checks.ego.forward_length_required_m == required_before_stop,
        "정차 상태에서는 이전 전방 길이 수치를 유지합니다");

  planning::PlanningValidatorCore seeded(
      {}, planning::InvalidTrajectoryPolicy::PublishAsIs);
  seeded.seedObservedValues(-62.0, 7.0, 40.0);
  outcome = seeded.onTrajectory(valid, planning::EgoState{10.0, 0.0, 0.0});
  check(outcome.checks.ego.longitudinal_distance_deviation.observed == -62.0 &&
            outcome.checks.ego.forward_length_required_m == 7.0 &&
            outcome.checks.ego.forward_length.observed == 40.0,
        "기록 시작 전 유지된 진단 수치를 복원합니다");

  planning::PlanningValidatorCore previous(
      {}, planning::InvalidTrajectoryPolicy::UsePreviousResult);
  check(!previous.onTrajectory(invalid, ego).selected_trajectory,
        "no previous valid trajectory means no output");
  check(previous.onTrajectory(valid, ego).selected_trajectory->size() == 3,
        "valid trajectory becomes fallback");
  outcome = previous.onTrajectory(invalid, ego);
  check(outcome.selected_trajectory && outcome.selected_trajectory->size() == 3 &&
            outcome.consecutive_invalid_count == 1,
        "invalid trajectory republishes previous valid result");

  planning::PlanningValidatorCore diagnostics(
      {}, planning::InvalidTrajectoryPolicy::PublishAsIs, 2);
  const auto first_invalid = diagnostics.onTrajectory(invalid, ego);
  const auto second_invalid = diagnostics.onTrajectory(invalid, ego);
  check(first_invalid.diagnostics.size() == 14 &&
            first_invalid.diagnostics[0].level ==
                planning::ValidationDiagnosticLevel::Warn &&
            second_invalid.diagnostics[0].level ==
                planning::ValidationDiagnosticLevel::Error,
        "연속 무효 횟수에 따라 진단 경고가 오류로 전환됩니다");

  std::cout << "planning validator core behavior passed\n";
}
