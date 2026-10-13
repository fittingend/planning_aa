#include "planning/trajectory_precheck.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace {
void check(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

bool near(double actual, double expected) {
  return std::abs(actual - expected) < 1.0e-5;
}
}  // 익명 네임스페이스

int main() {
  const planning::PrecheckThresholds defaults;

  const auto too_short = planning::checkTrajectoryLocal({}, defaults);
  check(!too_short.size.passed && !too_short.finite.evaluated &&
            !too_short.localChecksPass(),
        "invalid size stops later checks");

  planning::Trajectory straight;
  for (int i = 0; i < 5; ++i) {
    straight.push_back({static_cast<double>(2 * i), 0.0, 2.0F, 0.0F});
  }
  auto result = planning::checkTrajectoryLocal(straight, defaults);
  check(result.localChecksPass(), "ordinary straight trajectory passes local checks");
  check(near(result.interval.observed, 2.0), "maximum interval is measured");

  auto nonfinite = straight;
  nonfinite[2].orientation_w = std::numeric_limits<double>::quiet_NaN();
  result = planning::checkTrajectoryLocal(nonfinite, defaults);
  check(!result.finite.passed && result.finite.index == 2 &&
            !result.interval.evaluated,
        "nonfinite orientation stops geometric checks");

  auto thresholds = defaults;
  thresholds.max_interval_m = 2.0;
  check(planning::checkTrajectoryLocal(straight, thresholds).interval.passed,
        "interval at threshold passes");
  thresholds.max_interval_m = 1.9;
  check(!planning::checkTrajectoryLocal(straight, thresholds).interval.passed,
        "interval above threshold fails");

  auto acceleration = straight;
  acceleration[1].acceleration_mps2 = 10.0F;
  acceleration[2].acceleration_mps2 = -10.0F;
  result = planning::checkTrajectoryLocal(acceleration, defaults);
  check(!result.longitudinal_max_acceleration.passed &&
            !result.longitudinal_min_acceleration.passed,
        "signed longitudinal acceleration thresholds are checked independently");

  const planning::Trajectory corner{{0.0, 0.0, 2.0F, 0.0F},
                                    {2.0, 0.0, 2.0F, 0.0F},
                                    {2.0, 2.0, 2.0F, 0.0F}};
  thresholds = defaults;
  thresholds.max_relative_angle_rad = 1.0;
  thresholds.max_curvature_per_m = 0.5;
  thresholds.max_lateral_acceleration_mps2 = 2.0;
  thresholds.max_steering_angle_rad = 1.0;
  thresholds.max_steering_rate_radps = 1.0;
  result = planning::checkTrajectoryLocal(corner, thresholds);
  check(near(result.relative_angle.observed, 1.57079632679) &&
            !result.relative_angle.passed,
        "ninety-degree turn exceeds relative-angle limit");
  check(near(result.curvature.observed, std::sqrt(0.5)) &&
            !result.curvature.passed,
        "turn curvature uses source Menger calculation");
  check(near(result.lateral_acceleration.observed, 4.0 * std::sqrt(0.5)) &&
            !result.lateral_acceleration.passed,
        "lateral acceleration is speed squared times curvature");
  check(!result.steering_angle.passed && !result.steering_rate.passed,
        "steering angle and rate detect the corner");

  const planning::Trajectory clockwise{{0.0, 0.0, 2.0F, 0.0F},
                                       {2.0, 0.0, 2.0F, 0.0F},
                                       {4.0, -1.0, 2.0F, 0.0F},
                                       {6.0, -3.0, 2.0F, 0.0F},
                                       {8.0, -6.0, 2.0F, 0.0F}};
  result = planning::checkTrajectoryLocal(clockwise, defaults);
  check(result.curvature.observed < 0.0,
        "전 구간의 곡률이 음수이면 최대값도 음수로 유지합니다");

  std::cout << "trajectory local checks passed\n";
}
