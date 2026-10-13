#include "planning/trajectory_gate.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {
struct Trajectory {
  int identifier;
  int timestamp;
};

void check(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}
}  // 익명 네임스페이스

int main() {
  using planning::InvalidTrajectoryPolicy;
  using planning::TrajectoryGate;

  const Trajectory valid{1, 10};
  const Trajectory invalid{2, 20};
  TrajectoryGate<Trajectory> previous(InvalidTrajectoryPolicy::UsePreviousResult);
  check(!previous.select(valid, false, true), "no output before required data arrives");
  check(!previous.select(invalid, true, false), "no fallback exists before a valid result");
  check(previous.select(valid, true, true)->identifier == 1,
        "valid trajectory is selected and cached");
  const auto fallback = previous.select(invalid, true, false);
  check(fallback && fallback->identifier == 1 && fallback->timestamp == 10,
        "fallback retains the prior trajectory and original timestamp");
  check(previous.previousValid()->identifier == 1,
        "invalid input never replaces the last valid trajectory");

  TrajectoryGate<Trajectory> stop(InvalidTrajectoryPolicy::StopPublishing);
  check(stop.select(valid, true, true).has_value(), "valid input publishes under stop policy");
  check(!stop.select(invalid, true, false), "invalid input stops publication");

  TrajectoryGate<Trajectory> as_is(InvalidTrajectoryPolicy::PublishAsIs);
  check(as_is.select(valid, true, true).has_value(), "valid input publishes under as-is policy");
  check(as_is.select(invalid, true, false)->identifier == 2,
        "as-is policy publishes invalid input, matching the configured source behavior");
  check(as_is.previousValid()->identifier == 1,
        "published invalid input does not become a valid fallback");

  std::cout << "trajectory gate behavior passed\n";
}
