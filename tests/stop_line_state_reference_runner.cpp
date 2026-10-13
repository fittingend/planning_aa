#include "planning/behavior_path_postprocess.hpp"
#include "planning/stop_line_core.hpp"

#include <iomanip>
#include <iostream>

int main() {
  planning::StopLineSegment line;
  planning::StopLineParameters parameters;
  std::size_t path_size = 0;
  std::size_t frame_count = 0;
  if (!(std::cin >> line.x1 >> line.y1 >> line.x2 >> line.y2 >>
        parameters.front_offset_m >> path_size >> frame_count)) return 1;
  planning::Trajectory path(path_size);
  for (auto &point : path) {
    if (!(std::cin >> point.x >> point.y >> point.z >>
          point.orientation_x >> point.orientation_y >> point.orientation_z >>
          point.orientation_w >> point.longitudinal_velocity_mps >>
          point.lateral_velocity_mps >> point.heading_rate_rps)) return 1;
  }
  planning::StopLineCore core(line, parameters);
  std::cout << std::setprecision(17);
  for (std::size_t i = 0; i < frame_count; ++i) {
    double now_s = 0.0;
    double ego_x = 0.0;
    double ego_y = 0.0;
    double ego_velocity_mps = 0.0;
    if (!(std::cin >> now_s >> ego_x >> ego_y >> ego_velocity_mps)) return 1;
    const auto result = core.apply(
        path, ego_x, ego_y, ego_velocity_mps < 1.0e-3, now_s);
    const auto output = planning::postprocessBehaviorPath(result.trajectory);
    std::cout << static_cast<int>(result.state) << ' ' << output.size() << ' '
              << (result.stop_arc_m ? *result.stop_arc_m : -1.0) << '\n';
    for (const auto &point : output) {
      std::cout << point.x << ' ' << point.y << ' ' << point.z << ' '
                << point.orientation_x << ' ' << point.orientation_y << ' '
                << point.orientation_z << ' ' << point.orientation_w << ' '
                << point.longitudinal_velocity_mps << ' '
                << point.lateral_velocity_mps << ' '
                << point.heading_rate_rps << '\n';
    }
  }
  return 0;
}
