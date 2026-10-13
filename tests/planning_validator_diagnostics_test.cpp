#include "planning/planning_validator_diagnostics.hpp"

#include <cstdlib>
#include <iostream>

namespace {

void check(bool valid, const char *message) {
  if (!valid) {
    std::cerr << "검증 실패: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

}  // 익명 네임스페이스

int main() {
  planning::TrajectoryValidationResult checks;
  checks.local.size.passed = true;
  checks.local.interval.passed = true;
  checks.local.relative_angle.passed = true;
  checks.local.curvature.passed = true;
  checks.local.lateral_acceleration.passed = true;
  checks.local.longitudinal_max_acceleration.passed = true;
  checks.local.longitudinal_min_acceleration.passed = true;
  checks.local.steering_angle.passed = true;
  checks.local.steering_rate.passed = true;
  checks.ego.velocity_deviation.passed = true;
  checks.ego.distance_deviation.passed = true;
  checks.ego.longitudinal_distance_deviation.passed = true;
  checks.ego.forward_length.passed = true;

  const auto warning = planning::makeValidationDiagnostics(checks, 1, 2);
  check(warning.size() == 14, "원본의 진단 항목 14개를 만듭니다");
  check(warning[1].name == "trajectory_validation_finite" &&
            warning[1].level == planning::ValidationDiagnosticLevel::Warn &&
            warning[1].message ==
                "infinite value is found (invalid count is less than error threshold: 1 < 2)",
        "오류 횟수가 임계값보다 작으면 원본 경고 문구를 사용합니다");
  check(warning[0].level == planning::ValidationDiagnosticLevel::Ok &&
            warning[0].message == "validated.",
        "통과한 검사는 정상 진단을 만듭니다");
  const auto error = planning::makeValidationDiagnostics(checks, 2, 2);
  check(error[1].level == planning::ValidationDiagnosticLevel::Error &&
            error[1].message == "infinite value is found",
        "오류 횟수가 임계값에 도달하면 오류로 전환합니다");
  const auto immediate = planning::makeValidationDiagnostics(checks, 0, 0);
  check(immediate[1].level == planning::ValidationDiagnosticLevel::Error,
        "기본 임계값 0에서는 즉시 오류를 만듭니다");
}
