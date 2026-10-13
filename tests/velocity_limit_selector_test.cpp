#include "planning/velocity_limit_selector.hpp"

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

planning::VelocityLimit makeLimit(float speed, const std::string &sender = "") {
  planning::VelocityLimit result;
  result.max_velocity = speed;
  result.sender = sender;
  return result;
}
}  // 익명 네임스페이스

int main() {
  planning::VelocityLimitSelectorConfig config;
  config.max_velocity = 15.0;
  planning::VelocityLimitSelector selector(config);

  check(selector.current().max_velocity == 0.0F, "initial value matches source node");
  check(!selector.onClear("", false, {1}), "false clear command produces no output");
  check(selector.onClear("", true, {2})->max_velocity == 15.0F,
        "empty table emits configured maximum after event");
  check(selector.current().stamp.nanoseconds == 2, "default carries event time");

  check(selector.onApiLimit(makeLimit(20.0F), {3}).max_velocity == 15.0F,
        "API limit above configured maximum is capped");
  auto api = makeLimit(10.0F);
  api.stamp.nanoseconds = 44;
  auto selected = selector.onApiLimit(api, {4});
  check(selected.max_velocity == 10.0F && selected.sender == "api",
        "API limit replaces previous API entry");
  check(selected.stamp.nanoseconds == 44, "winning limit keeps its source stamp");

  check(selector.onInternalLimit(makeLimit(5.0F, "obstacle"), {5}).max_velocity == 5.0F,
        "internal limit can be more restrictive");
  check(selector.onClear("obstacle", true, {6})->max_velocity == 10.0F,
        "clearing internal limit restores API limit");
  config.max_velocity = 2.5;
  selector.setConfig(config);
  check(selector.current().max_velocity == 10.0F,
        "parameter change takes effect on next event");
  check(selector.onClear("obstacle", true, {7})->max_velocity == 2.5F,
        "next event uses updated configured maximum");

  auto invalid = makeLimit(std::numeric_limits<float>::quiet_NaN(), "invalid");
  check(selector.onInternalLimit(invalid, {8}).max_velocity == 2.5F,
        "nonfinite limit falls back to configured maximum");

  auto constrained = makeLimit(2.0F, "constrained");
  constrained.use_constraints = true;
  constrained.constraints = {1.5F, -2.0F, 1.5F, -1.5F};
  selected = selector.onInternalLimit(constrained, {9});
  check(selected.max_velocity == 2.0F && selected.use_constraints,
        "constrained limit is selected");
  check(selected.constraints.max_acceleration == 1.5F &&
            selected.constraints.min_acceleration == -2.0F,
        "source acceleration constraint selection is retained");

  std::cout << "velocity limit selector behavior passed\n";
}
