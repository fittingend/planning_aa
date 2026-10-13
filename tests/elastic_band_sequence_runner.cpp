#include "planning/elastic_band_core.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {
planning::Trajectory readPath(const std::string &filename) {
  std::ifstream stream(filename);
  if (!stream) return {};
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
}  // 익명 네임스페이스

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  std::ifstream manifest(argv[1]);
  std::ofstream output(argv[2]);
  if (!manifest || !output) return 2;
  output << std::setprecision(17);
  planning::ElasticBandPlanner planner;
  std::string line;
  int frame = 0;
  while (std::getline(manifest, line)) {
    std::istringstream values(line);
    std::string kind, timestamp, x, y, velocity, path_file, optimized_file;
    if (!std::getline(values, kind, ',') ||
        !std::getline(values, timestamp, ',') ||
        !std::getline(values, x, ',') ||
        !std::getline(values, y, ',') ||
        !std::getline(values, velocity, ',') ||
        !std::getline(values, path_file, ',')) return 2;
    planning::TrajectoryPoint ego;
    ego.x = std::stod(x);
    ego.y = std::stod(y);
    const auto path = readPath(path_file);
    if (path.empty()) return 2;
    if (kind == "seed") {
      if (!std::getline(values, optimized_file, ',')) return 2;
      const auto optimized = readPath(optimized_file);
      if (optimized.empty()) return 2;
      planner.seed(path, optimized, ego, std::stod(timestamp));
    } else if (kind == "frame") {
      const auto plan = planner.run(
          path, ego, std::stod(velocity), std::stod(timestamp));
      output << frame << ",state," << plan.reset << ',' << plan.replanned
             << ",0\n";
      for (const auto &point : plan.optimized) {
        output << frame << ",optimized," << point.x << ',' << point.y
               << ',' << point.longitudinal_velocity_mps << '\n';
      }
      for (const auto &point : plan.trajectory) {
        output << frame << ",output," << point.x << ',' << point.y
               << ',' << point.longitudinal_velocity_mps << '\n';
      }
      ++frame;
    } else {
      return 2;
    }
  }
}
