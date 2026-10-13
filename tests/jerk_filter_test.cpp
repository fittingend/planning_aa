#include "planning/jerk_filter.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void check(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

bool near(double actual, double expected, double tolerance = 1.0e-5) {
  return std::fabs(actual - expected) < tolerance;
}
}  // 익명 네임스페이스

int main() {
  const planning::JerkFilterLimits limits{1.0, -1.0, 0.0, 1.0, -1.0};

  const planning::FilterTrajectory acceleration_case{{0.0, 0.0, 10.0F, 0.0F},
                                                     {1.0, 0.0, 10.0F, 0.0F}};
  const auto accelerated = planning::runJerkFilterStage(acceleration_case, 0.0, 0.0, limits);
  check(accelerated.forward.size() == 2, "forward stage retains input points");
  check(near(accelerated.forward[0].longitudinal_velocity_mps, 0.0),
        "initial velocity is applied");
  check(near(accelerated.forward[1].longitudinal_velocity_mps,
             0.5 * std::pow(6.0, 1.0 / 3.0)),
        "forward acceleration matches source jerk-limited equation");
  check(near(accelerated.forward[1].acceleration_mps2, 1.0),
        "forward acceleration respects maximum");

  const planning::FilterTrajectory stop_case{{0.0, 0.0, 10.0F, 0.0F},
                                             {1.0, 0.0, 10.0F, 0.0F},
                                             {2.0, 0.0, 0.0F, 0.0F}};
  const auto stopped = planning::runJerkFilterStage(stop_case, 2.0, 0.0, limits);
  check(stopped.backward.size() == stop_case.size() &&
            stopped.merged.size() == stop_case.size(),
        "backward and merged stages retain point count");
  check(stopped.merged.front().longitudinal_velocity_mps == 2.0F,
        "merge retains the source algorithm's initial speed behavior");
  check(stopped.merged.back().longitudinal_velocity_mps == 0.0F,
        "terminal stop remains zero");
  for (const auto &point : stopped.merged) {
    check(std::isfinite(point.longitudinal_velocity_mps) &&
              std::isfinite(point.acceleration_mps2),
          "valid positive-spacing input produces finite stage values");
  }

  bool rejected = false;
  try {
    planning::runJerkFilterStage({}, 0.0, 0.0, limits);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, "empty input is rejected before stage functions access front");

  rejected = false;
  try {
    planning::runJerkFilterStage({stop_case[0], stop_case[0]}, 0.0, 0.0, limits);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, "zero spacing is rejected before a zero-duration division");

  std::cout << "jerk filter stage behavior passed\n";
}
