// SPDX-License-Identifier: Apache-2.0
// 저장된 궤적을 독립 evaluator 코어에 연속 입력합니다.

#include "planning/planning_evaluator_core.hpp"

#include <fstream>
#include <iomanip>
#include <string>

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  std::ifstream input(argv[1]);
  std::ofstream output(argv[2]);
  if (!input || !output) return 2;
  output << std::setprecision(17);
  planning::PlanningEvaluatorCore evaluator;
  std::size_t frames = 0;
  input >> frames;
  for (std::size_t frame = 0; frame < frames; ++frame) {
    planning::PlanningEvaluatorInput request;
    planning::TrajectoryPoint ego;
    std::size_t count = 0;
    input >> ego.x >> ego.y >> ego.z >> ego.orientation_x >>
        ego.orientation_y >> ego.orientation_z >> ego.orientation_w >>
        request.ego_speed_mps >> count;
    if (!input || count < 2) return 3;
    request.ego = ego;
    planning::Trajectory trajectory;
    trajectory.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      planning::TrajectoryPoint point;
      input >> point.x >> point.y >> point.z >>
          point.orientation_x >> point.orientation_y >>
          point.orientation_z >> point.orientation_w >>
          point.longitudinal_velocity_mps >> point.acceleration_mps2;
      if (!input) return 3;
      trajectory.push_back(point);
    }
    request.trajectory = std::move(trajectory);
    std::size_t reference_count = 0;
    input >> reference_count;
    if (!input) return 3;
    if (reference_count > 0) {
      planning::Trajectory reference;
      reference.reserve(reference_count);
      for (std::size_t i = 0; i < reference_count; ++i) {
        planning::TrajectoryPoint point;
        input >> point.x >> point.y >> point.z >>
            point.orientation_x >> point.orientation_y >>
            point.orientation_z >> point.orientation_w >>
            point.longitudinal_velocity_mps >> point.acceleration_mps2;
        if (!input) return 3;
        reference.push_back(point);
      }
      request.reference_trajectory = std::move(reference);
    }
    std::size_t object_count = 0;
    input >> object_count;
    if (!input) return 3;
    if (object_count > 0) {
      std::vector<std::pair<double, double>> objects;
      objects.reserve(object_count);
      for (std::size_t i = 0; i < object_count; ++i) {
        double x = 0.0, y = 0.0;
        input >> x >> y;
        if (!input) return 3;
        objects.emplace_back(x, y);
      }
      request.object_centers = std::move(objects);
    }
    const auto result = evaluator.tick(request);
    for (const std::string name : {"length", "point_interval", "velocity",
                                   "acceleration", "jerk", "duration",
                                   "curvature", "relative_angle",
                                   "lateral_deviation", "yaw_deviation",
                                   "velocity_deviation", "obstacle_distance"}) {
      const auto iterator = result.metrics.find(name);
      if (iterator == result.metrics.end()) {
        output << "nan nan nan ";
        continue;
      }
      const auto &stat = iterator->second;
      output << stat.minimum << ' ' << stat.maximum << ' ' << stat.mean << ' ';
    }
    output << '\n';
  }
}
