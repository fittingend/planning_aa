#include "planning/jerk_smoother.hpp"
#include "planning/jerk_node_policy.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

int main(int argc, char **argv) {
  if (argc != 16 && argc != 17 && argc != 18 && argc != 19) return 2;
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

  const double ego_velocity = std::stod(argv[3]);
  const double initial_velocity = std::stod(argv[13]);
  const double initial_acceleration = std::stod(argv[14]);
  const bool reverse = planning::isReverseJerkTrajectory(input);
  if (reverse) planning::flipJerkTrajectoryVelocity(input);
  const auto closest = static_cast<std::size_t>(std::stoul(argv[5]));
  if (closest >= input.size()) return 2;
  const planning::Trajectory clipped(input.begin() + closest, input.end());
  planning::TrajectoryPoint ego;
  ego.x = std::stod(argv[6]);
  ego.y = std::stod(argv[7]);
  ego.z = std::stod(argv[8]);
  ego.orientation_x = std::stod(argv[9]);
  ego.orientation_y = std::stod(argv[10]);
  ego.orientation_z = std::stod(argv[11]);
  ego.orientation_w = std::stod(argv[12]);
  const auto result = planning::runJerkSmoother(
      clipped, initial_velocity, initial_acceleration);
  if (!result.output || (clipped.size() > 1 && !result.qp_result.solved)) {
    std::cerr << "스무더 풀이 실패: 입력 " << clipped.size()
              << "점, 반복 " << result.qp_result.iterations
              << "회, 원시 잔차 " << result.qp_result.primal_residual
              << ", 쌍대 잔차 " << result.qp_result.dual_residual << '\n';
    return 1;
  }

  output_file << std::setprecision(17);
  for (const auto &point : result.optimization_input) {
    output_file << "merged," << point.x << ',' << point.y << ','
                << (reverse ? -point.longitudinal_velocity_mps
                            : point.longitudinal_velocity_mps)
                << ',' << point.acceleration_mps2 << '\n';
  }
  for (const auto &point : *result.output) {
    output_file << "optimized," << point.x << ',' << point.y << ','
                << (reverse ? -point.longitudinal_velocity_mps
                            : point.longitudinal_velocity_mps)
                << ',' << point.acceleration_mps2 << '\n';
  }

  planning::JerkNodeOutputParameters node_parameters;
  node_parameters.keep_closest_motion_behind =
      argc >= 17 && std::string(argv[16]) == "1";
  if (argc >= 18) node_parameters.max_velocity = std::stod(argv[17]);
  if (argc == 19) {
    node_parameters.preserve_velocity_restarts = std::string(argv[18]) == "1";
  }
  node_parameters.post_resampling.max_trajectory_length = 300.0;
  node_parameters.post_resampling.min_trajectory_length = 30.0;
  node_parameters.post_resampling.resample_time = 10.0;
  node_parameters.post_resampling.dense_resample_dt = 0.1;
  node_parameters.post_resampling.dense_min_interval_distance = 0.1;
  node_parameters.post_resampling.sparse_resample_dt = 0.1;
  node_parameters.post_resampling.sparse_min_interval_distance = 1.0;
  planning::Trajectory previous;
  if (argv[15][0] != '\0') {
    std::ifstream previous_file(argv[15]);
    if (!previous_file) return 2;
    while (std::getline(previous_file, line)) {
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
      previous.push_back(point);
    }
  }
  planning::Trajectory internal_output;
  auto final_output = previous.empty()
      ? planning::finalizeFirstJerkNodeTrajectory(
            input, closest, *result.output, ego, ego_velocity, node_parameters,
            &internal_output)
      : planning::finalizeNextJerkNodeTrajectory(
            input, closest, *result.output, previous, ego, ego_velocity,
            node_parameters, &internal_output);
  if (reverse) planning::flipJerkTrajectoryVelocity(final_output);
  if (reverse) planning::flipJerkTrajectoryVelocity(internal_output);
  for (const auto &point : internal_output) {
    output_file << "internal," << point.x << ',' << point.y << ','
                << point.longitudinal_velocity_mps << ',' << point.acceleration_mps2 << '\n';
  }
  for (const auto &point : final_output) {
    output_file << "output," << point.x << ',' << point.y << ','
                << point.longitudinal_velocity_mps << ',' << point.acceleration_mps2 << '\n';
  }
}
