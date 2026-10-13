#include "planning/behavior_path_postprocess.hpp"
#include "planning/crosswalk_core.hpp"
#include "planning/lanelet_osm_adapter.hpp"

#include <iomanip>
#include <iostream>

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  const auto polygon = planning::loadCrosswalkPolygon(argv[1], std::stoll(argv[2]));
  std::size_t count = 0;
  std::cin >> count;
  planning::Trajectory path(count);
  for (auto &point : path) {
    std::cin >> point.x >> point.y >> point.z >> point.longitudinal_velocity_mps;
  }
  double ego_x, ego_y, ego_speed, ego_acceleration;
  double object_x, object_y, object_length, object_speed;
  int enable_in_intersection;
  std::cin >> ego_x >> ego_y >> ego_speed >> ego_acceleration;
  std::cin >> object_x >> object_y >> object_length >> object_speed;
  std::cin >> enable_in_intersection;
  if (!std::cin) return 3;
  planning::CrosswalkStuckParameters parameters;
  parameters.enable_in_intersection = enable_in_intersection != 0;
  const auto result = planning::planCrosswalkStuckVehicleStop(
      path, polygon, {ego_x, ego_y}, ego_speed, ego_acceleration,
      {{{object_x, object_y}, object_length, object_speed, true}},
      true, true, {}, parameters);
  const auto output = planning::postprocessBehaviorPath(result.trajectory);
  std::cout << std::setprecision(17) << output.size() << '\n';
  for (const auto &point : output) {
    std::cout << point.x << ' ' << point.y << ' ' << point.z << ' '
              << point.longitudinal_velocity_mps << ' '
              << point.orientation_x << ' ' << point.orientation_y << ' '
              << point.orientation_z << ' ' << point.orientation_w << '\n';
  }
}
