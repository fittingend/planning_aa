#include "planning/jerk_path_limits.hpp"
#include "planning/jerk_node_policy.hpp"
#include "planning/jerk_external_limit.hpp"
#include "planning/jerk_resample.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <sstream>
#include <string>

int main(int argc, char **argv) {
  const bool external_state = argc == 16 &&
      std::string(argv[12]) == "--external-state";
  const bool extended = (argc == 13 || argc == 14) &&
      std::string(argv[12]) == "--extended";
  if (argc != 12 && argc != 15 && !extended && !external_state) return 2;
  std::ifstream input_file(argv[1]);
  std::ofstream output_file(argv[2]);
  if (!input_file || !output_file) return 2;
  planning::Trajectory input;
  std::string line;
  while (std::getline(input_file, line)) {
    std::istringstream values(line);
    double x, y, z, qx, qy, qz, qw;
    float velocity, acceleration;
    char comma;
    if (!(values >> x >> comma >> y >> comma >> velocity >> comma >> acceleration >>
          comma >> z >> comma >> qx >> comma >> qy >> comma >> qz >> comma >> qw)) return 2;
    planning::TrajectoryPoint point{x, y, velocity, acceleration};
    point.z = z;
    point.orientation_x = qx;
    point.orientation_y = qy;
    point.orientation_z = qz;
    point.orientation_w = qw;
    input.push_back(point);
  }
  const bool reverse = planning::isReverseJerkTrajectory(input);
  if (reverse) planning::flipJerkTrajectoryVelocity(input);
  const planning::JerkPathLimitParameters parameters;
  planning::TrajectoryPoint ego;
  ego.x = std::stod(argv[5]);
  ego.y = std::stod(argv[6]);
  ego.z = std::stod(argv[7]);
  ego.orientation_x = std::stod(argv[8]);
  ego.orientation_y = std::stod(argv[9]);
  ego.orientation_z = std::stod(argv[10]);
  ego.orientation_w = std::stod(argv[11]);
  planning::Trajectory limited;
  if (external_state) {
    planning::JerkExternalLimitPlan plan;
    plan.velocity_before_limit = std::stod(argv[13]);
    plan.target_velocity = std::stod(argv[14]);
    plan.insertion_distance = std::stod(argv[15]);
    limited = planning::applyJerkExternalLimit(input, ego, plan);
  } else if (argc == 15) {
    const planning::JerkMotion previous{std::stod(argv[12]), std::stod(argv[13])};
    auto plan = planning::planJerkExternalLimit(
        previous, std::stod(argv[14]), planning::JerkExternalLimitParameters{});
    planning::advanceJerkExternalLimit(plan, ego.x - input.front().x);
    std::cerr << "외부 제한: 이전 속도=" << previous.velocity
              << ", 이전 가속도=" << previous.acceleration
              << ", 제한 전 속도=" << plan.velocity_before_limit
              << ", 삽입 거리=" << plan.insertion_distance
              << ", 거리 유효=" << plan.distance_valid << '\n';
    limited = planning::applyJerkExternalLimit(input, ego, plan);
  } else {
    const double maximum_velocity = argc == 14 ? std::stod(argv[13]) : 4.17;
    limited = planning::limitJerkNodeInputVelocity(
        input, maximum_velocity, 2.778, 0.0);
  }
  const auto lateral = planning::applyJerkLateralAccelerationLimit(
      limited, std::fabs(std::stod(argv[3])), std::stod(argv[4]), parameters);
  const auto steering = planning::applyJerkSteeringRateLimit(lateral, parameters);
  auto resampled = planning::resampleJerkNodeInput(
      steering, ego, 3.0, 1.046, planning::JerkResampleParameters{}, 0.1);
  resampled.back().longitudinal_velocity_mps = 0.0F;
  output_file << std::setprecision(17);
  for (const auto &point : limited) {
    output_file << "limited," << point.x << ',' << point.y << ','
                << point.longitudinal_velocity_mps << ',' << point.acceleration_mps2 << '\n';
  }
  for (const auto &point : lateral) {
    output_file << "lateral," << point.x << ',' << point.y << ','
                << (reverse ? -point.longitudinal_velocity_mps
                            : point.longitudinal_velocity_mps)
                << ',' << point.acceleration_mps2 << '\n';
  }
  for (const auto &point : steering) {
    output_file << "steering," << point.x << ',' << point.y << ','
                << (reverse ? -point.longitudinal_velocity_mps
                            : point.longitudinal_velocity_mps)
                << ',' << point.acceleration_mps2 << '\n';
  }
  for (const auto &point : resampled) {
    output_file << "resampled," << point.x << ',' << point.y << ','
                << (reverse ? -point.longitudinal_velocity_mps
                            : point.longitudinal_velocity_mps)
                << ',' << point.acceleration_mps2 << '\n';
    if (extended || external_state) {
      output_file << "resampled_full," << point.x << ',' << point.y << ','
                  << (reverse ? -point.longitudinal_velocity_mps
                              : point.longitudinal_velocity_mps)
                  << ',' << point.acceleration_mps2 << ',' << point.z << ','
                  << point.orientation_x << ',' << point.orientation_y << ','
                  << point.orientation_z << ',' << point.orientation_w << '\n';
    }
  }
}
