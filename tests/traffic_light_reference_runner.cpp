#include "planning/traffic_light_core.hpp"
#include "planning/behavior_path_postprocess.hpp"

#include <iomanip>
#include <iostream>

int main() {
  planning::StopLineSegment line;
  double front_offset = 0.0;
  std::size_t count = 0;
  planning::TrafficLightInput input;
  if (!(std::cin >> line.x1 >> line.y1 >> line.x2 >> line.y2 >>
        front_offset >> input.ego_x >> input.ego_y >>
        input.ego_velocity_mps >> input.ego_acceleration_mps2 >> count)) return 1;
  planning::Trajectory path(count);
  for (auto &point : path) {
    if (!(std::cin >> point.x >> point.y >> point.z >>
          point.orientation_x >> point.orientation_y >> point.orientation_z >>
          point.orientation_w >> point.longitudinal_velocity_mps >>
          point.lateral_velocity_mps >> point.heading_rate_rps)) return 1;
  }
  planning::TrafficLightParameters parameters;
  parameters.front_offset_m = front_offset;
  planning::TrafficLightCore core(line, parameters);
  core.apply(path, input);
  input.now_s = 0.2;
  const auto result = core.apply(path, input);
  if (!result.applied_stop) return 2;
  const auto output = planning::postprocessBehaviorPath(result.trajectory);
  std::cout << output.size() << '\n' << std::setprecision(17);
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
