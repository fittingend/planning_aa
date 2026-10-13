#include "planning/planning_validator_core.hpp"

#include <iostream>
#include <utility>

int main() {
  planning::PlanningValidatorCore core(
      {}, planning::InvalidTrajectoryPolicy::PublishAsIs);
  planning::Trajectory valid(3);
  for (std::size_t index = 0; index < valid.size(); ++index) {
    valid[index].x = static_cast<double>(index) * 10.0;
    valid[index].longitudinal_velocity_mps = 10.0F;
  }
  const planning::Trajectory invalid{valid.front()};
  const planning::EgoState ego{0.0, 0.0, 10.0};
  for (const auto &sample : {
           std::pair<const char *, const planning::Trajectory &>{"valid_before", valid},
           {"invalid_size", invalid},
           {"valid_after", valid}}) {
    const auto result = core.onTrajectory(sample.second, ego);
    for (const auto &diagnostic : result.diagnostics) {
      std::cout << sample.first << '\t' << diagnostic.name << '\t'
                << static_cast<int>(diagnostic.level) << '\t'
                << diagnostic.message << '\n';
    }
  }
}
