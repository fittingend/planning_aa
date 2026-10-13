#include "planning/planning_validator_core.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 2 && argc != 3 && argc != 6) return 2;
  std::ifstream input(argv[1]);
  if (!input) return 2;
  planning::PrecheckThresholds thresholds;
  thresholds.wheelbase_m = argc >= 3 ? std::stod(argv[2]) : 4.0;
  planning::PlanningValidatorCore core(
      thresholds, planning::InvalidTrajectoryPolicy::PublishAsIs);
  if (argc == 6) {
    core.seedObservedValues(
        std::stod(argv[3]), std::stod(argv[4]), std::stod(argv[5]));
  }
  std::cout << std::setprecision(17);
  std::string line;
  int active_case = -1;
  planning::Trajectory trajectory;
  planning::EgoState ego;
  auto emit = [&]() {
    if (active_case < 0) return;
    const auto result = core.onTrajectory(trajectory, ego);
    const auto &local = result.checks.local;
    const auto &state = result.checks.ego;
    const auto flag = [&](const char *name, const planning::CheckResult &check) {
      std::cout << active_case << ',' << name << ',' << check.passed << '\n';
    };
    const auto value = [&](const char *name, double observed) {
      std::cout << active_case << ',' << name << ',' << observed << '\n';
    };
    flag("is_valid_size", local.size);
    flag("is_valid_finite_value", local.finite);
    flag("is_valid_interval", local.interval);
    flag("is_valid_relative_angle", local.relative_angle);
    flag("is_valid_curvature", local.curvature);
    flag("is_valid_lateral_acc", local.lateral_acceleration);
    flag("is_valid_longitudinal_max_acc", local.longitudinal_max_acceleration);
    flag("is_valid_longitudinal_min_acc", local.longitudinal_min_acceleration);
    flag("is_valid_steering", local.steering_angle);
    flag("is_valid_steering_rate", local.steering_rate);
    flag("is_valid_velocity_deviation", state.velocity_deviation);
    flag("is_valid_distance_deviation", state.distance_deviation);
    flag("is_valid_longitudinal_distance_deviation", state.longitudinal_distance_deviation);
    flag("is_valid_forward_trajectory_length", state.forward_length);
    value("trajectory_size", local.size.observed);
    value("max_interval_distance", local.interval.observed);
    value("max_relative_angle", local.relative_angle.observed);
    value("max_curvature", local.curvature.observed);
    value("max_lateral_acc", local.lateral_acceleration.observed);
    value("max_longitudinal_acc", local.longitudinal_max_acceleration.observed);
    value("min_longitudinal_acc", local.longitudinal_min_acceleration.observed);
    value("max_steering", local.steering_angle.observed);
    value("max_steering_rate", local.steering_rate.observed);
    value("velocity_deviation", state.velocity_deviation.observed);
    value("distance_deviation", state.distance_deviation.observed);
    value("longitudinal_distance_deviation", state.longitudinal_distance_deviation.observed);
    value("forward_trajectory_length_required", state.forward_length_required_m);
    value("forward_trajectory_length_measured", state.forward_length.observed);
    value("invalid_count", result.consecutive_invalid_count);
  };
  while (std::getline(input, line)) {
    std::istringstream stream(line);
    std::vector<std::string> fields;
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(field);
    if (fields.size() != 14) return 2;
    const int case_index = std::stoi(fields[0]);
    if (case_index != active_case) {
      emit();
      active_case = case_index;
      trajectory.clear();
      ego = {std::stod(fields[11]), std::stod(fields[12]), std::stod(fields[13])};
    }
    planning::TrajectoryPoint point;
    point.x = std::stod(fields[1]);
    point.y = std::stod(fields[2]);
    point.z = std::stod(fields[3]);
    point.orientation_x = std::stod(fields[4]);
    point.orientation_y = std::stod(fields[5]);
    point.orientation_z = std::stod(fields[6]);
    point.orientation_w = std::stod(fields[7]);
    point.longitudinal_velocity_mps = std::stof(fields[8]);
    point.heading_rate_rps = std::stof(fields[9]);
    point.acceleration_mps2 = std::stof(fields[10]);
    trajectory.push_back(point);
  }
  emit();
}
