// SPDX-License-Identifier: Apache-2.0

#include "planning/remaining_distance_time_core.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}
}  // namespace

int main() {
  using namespace planning;
  try {
    bool fail_route = false;
    RemainingDistanceTimeCore calculator(
        [&](RouteCheckpoint ego,
            RouteCheckpoint goal) -> std::optional<double> {
          if (fail_route) return std::nullopt;
          return goal.x - ego.x;
        });
    check(!calculator.tick(), "scenario is required before publishing");
    calculator.onScenario(RemainingDistanceScenario::LaneDriving);
    check(!calculator.tick(), "route and ego are required");
    calculator.onEgo({2.0, 0.0, 0.0, 0.0});
    calculator.onGoal({102.0, 0.0, 0.0, 0.0});
    const auto initial = calculator.tick();
    check(initial && initial->distance_m == 100.0 &&
              initial->time_s > 1.0 && initial->time_s < 1.01,
          "default limit must be 99.99 m/s");
    calculator.onVelocityLimit(10.0);
    check(calculator.tick()->time_s == 10.0,
          "velocity limit should update time");
    calculator.onVelocityLimit(0.0);
    check(calculator.velocityLimit() == 10.0,
          "zero limit should be ignored like source callback");
    fail_route = true;
    calculator.onEgo({5.0, 0.0, 0.0, 0.0});
    check(calculator.tick()->distance_m == 100.0,
          "routing failure must retain previous distance");
    calculator.onScenario(RemainingDistanceScenario::Parking);
    check(calculator.tick()->distance_m == 0.0 &&
              calculator.tick()->time_s == 0.0,
          "parking should publish zeros");
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
