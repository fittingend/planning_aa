#include "planning/trajectory_precheck.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
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
  planning::Trajectory trajectory{{0.0, 0.0, 10.0F, 0.0F},
                                  {10.0, 0.0, 10.0F, 0.0F},
                                  {20.0, 0.0, 10.0F, 0.0F}};
  planning::PrecheckThresholds thresholds;
  thresholds.max_velocity_deviation_mps = 2.0;
  thresholds.max_distance_deviation_m = 1.0;
  thresholds.max_longitudinal_distance_deviation_m = 1.0;

  auto result = planning::validateTrajectory(trajectory, {0.0, 0.0, 10.0}, thresholds);
  check(result.allValid(), "valid ego state and adequate forward path pass all checks");
  check(near(result.ego.forward_length.observed, 20.0),
        "forward arc length is measured from ego projection");
  check(near(result.ego.forward_length_required_m, 100.0 / 6.0 - 2.0),
        "forward requirement uses stopping distance and margin");

  result = planning::validateTrajectory(trajectory, {10.0, 0.0, 10.0}, thresholds);
  check(result.ego.longitudinal_distance_deviation.passed,
        "interior nearest point has no endpoint deviation");
  check(!result.ego.forward_length.passed,
        "short remaining path fails at driving speed");

  result = planning::validateTrajectory(trajectory, {-3.0, 0.0, 10.0}, thresholds);
  check(!result.ego.longitudinal_distance_deviation.passed &&
            near(result.ego.longitudinal_distance_deviation.observed, -3.0),
        "ego behind first point exceeds longitudinal deviation");
  check(!result.ego.distance_deviation.passed,
        "ego offset from nearest point exceeds distance threshold");

  result = planning::validateTrajectory(trajectory, {22.0, 0.0, 10.0}, thresholds);
  check(!result.ego.longitudinal_distance_deviation.passed &&
            near(result.ego.longitudinal_distance_deviation.observed, 2.0),
        "ego beyond last point exceeds longitudinal deviation");

  result = planning::validateTrajectory(trajectory, {0.0, 0.0, 0.0}, thresholds);
  check(!result.ego.velocity_deviation.passed && result.ego.forward_length.passed,
        "near-stopped ego skips forward-length failure but checks speed deviation");

  std::cout << "trajectory ego checks passed\n";
}
