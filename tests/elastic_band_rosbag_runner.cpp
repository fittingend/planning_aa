#include "planning/elastic_band_core.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

planning::Trajectory readTrajectory(std::istream &stream) {
  planning::Trajectory result;
  std::string line;
  while (std::getline(stream, line)) {
    std::istringstream values(line);
    planning::TrajectoryPoint point;
    char comma;
    if (!(values >> point.x >> comma >> point.y >> comma >> point.z >> comma >>
          point.orientation_x >> comma >> point.orientation_y >> comma >>
          point.orientation_z >> comma >> point.orientation_w >> comma >>
          point.longitudinal_velocity_mps)) return {};
    result.push_back(point);
  }
  return result;
}

int main(int argc, char **argv) {
  if (argc != 5 && argc != 6) return 2;
  std::ifstream input_file(argv[1]);
  std::ofstream output_file(argv[2]);
  if (!input_file || !output_file) return 2;
  auto input = readTrajectory(input_file);
  if (input.empty()) return 2;
  planning::Trajectory previous;
  if (argc == 6) {
    std::ifstream previous_file(argv[5]);
    if (!previous_file) return 2;
    previous = readTrajectory(previous_file);
    if (previous.empty()) return 2;
  }
  planning::TrajectoryPoint ego;
  ego.x = std::stod(argv[3]);
  ego.y = std::stod(argv[4]);
  const auto prepared = planning::prepareElasticBandInput(
      input, ego, previous.empty() ? nullptr : &previous);
  const auto result = planning::smoothElasticBandResampled(
      prepared.trajectory, prepared.goal_contained);
  if (!result) return 1;
  output_file << std::setprecision(17);
  for (const auto &point : prepared.trajectory) {
    output_file << "prepared," << point.x << ',' << point.y << '\n';
  }
  for (const auto &point : result->trajectory) {
    output_file << "optimized," << point.x << ',' << point.y << '\n';
  }
  const auto finalized = planning::finalizeElasticBandTrajectory(
      input, result->trajectory, ego);
  for (const auto &point : finalized) {
    output_file << "finalized," << point.x << ',' << point.y << ','
                << point.longitudinal_velocity_mps << '\n';
  }
}
