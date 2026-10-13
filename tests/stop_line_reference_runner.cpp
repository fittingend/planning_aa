#include "planning/behavior_path_postprocess.hpp"
#include "planning/stop_line_core.hpp"

#include <iomanip>
#include <iostream>

int main() {
  planning::StopLineSegment line;
  planning::StopLineParameters parameters;
  double ego_x = 0.0;
  double ego_y = 0.0;
  std::size_t count = 0;
  if (!(std::cin >> line.x1 >> line.y1 >> line.x2 >> line.y2 >>
        parameters.front_offset_m >> ego_x >> ego_y >> count)) return 1;
  planning::Trajectory path(count);
  for (auto &point : path) {
    if (!(std::cin >> point.x >> point.y >> point.z >>
          point.orientation_x >> point.orientation_y >> point.orientation_z >>
          point.orientation_w >> point.longitudinal_velocity_mps >>
          point.lateral_velocity_mps >> point.heading_rate_rps)) return 1;
  }
  planning::StopLineCore core(line, parameters);
  const auto result = core.apply(path, ego_x, ego_y, false, 0.0);
  if (!result.stop_arc_m) return 2;
  const auto output = planning::postprocessBehaviorPath(result.trajectory);
  std::cout << output.size() << ' ' << std::setprecision(17)
            << *result.stop_arc_m << '\n';
  for (const auto &point : output) {
    std::cout << point.x << ' ' << point.y << ' ' << point.z << ' '
              << point.orientation_x << ' ' << point.orientation_y << ' '
              << point.orientation_z << ' ' << point.orientation_w << ' '
              << point.longitudinal_velocity_mps << ' '
              << point.lateral_velocity_mps << ' '
              << point.heading_rate_rps << '\n';
  }
  return 0;
}
