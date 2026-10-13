#include "planning/behavior_path_postprocess.hpp"
#include "planning/intersection_stuck_core.hpp"
#include "planning/lanelet_bin_adapter.hpp"

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 2) return 1;
  std::ifstream stream(argv[1], std::ios::binary);
  if (!stream) return 2;
  const std::vector<std::uint8_t> map_data{
      std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  planning::LaneletBinAdapter map(map_data);
  double ego_x = 0.0, ego_y = 0.0;
  double ego_velocity = 0.0, ego_acceleration = 0.0;
  std::size_t path_size = 0, object_size = 0;
  if (!(std::cin >> ego_x >> ego_y >> ego_velocity >>
        ego_acceleration >> path_size)) return 3;
  planning::Trajectory path(path_size);
  for (auto &point : path) {
    if (!(std::cin >> point.x >> point.y >> point.z >>
          point.orientation_x >> point.orientation_y >>
          point.orientation_z >> point.orientation_w >>
          point.longitudinal_velocity_mps >>
          point.lateral_velocity_mps >> point.heading_rate_rps)) return 4;
  }
  if (!(std::cin >> object_size)) return 5;
  std::vector<planning::IntersectionStuckObject> objects(object_size);
  for (auto &object : objects) {
    int classification = 0;
    if (!(std::cin >> object.position.x >> object.position.y >>
          object.speed_mps >> classification)) return 6;
    object.classification = static_cast<std::uint8_t>(classification);
  }
  std::vector<planning::CrosswalkBounds> conflicts;
  for (const auto id : map.intersectionConflictingLaneletIds(25001)) {
    conflicts.push_back(map.laneBounds(id));
  }
  planning::IntersectionStuckParameters settings;
  settings.detection_lanes.push_back(map.laneBounds(25001));
  settings.insert_control_points = true;
  settings.ego_velocity_mps = ego_velocity;
  settings.ego_acceleration_mps2 = ego_acceleration;
  for (const auto id : map.intersectionConflictingLaneletIds(25001)) {
    settings.conflict_centerlines.push_back(map.laneCenterline(id));
  }
  const auto output = planning::planIntersectionStuckStop(
      path, conflicts, objects, ego_x, ego_y, settings);
  const auto trajectory = planning::postprocessBehaviorPath(output.trajectory);
  std::cout << std::setprecision(17) << output.stuck_detected << ' '
            << (output.first_conflict_arc_m ? *output.first_conflict_arc_m : -1.0)
            << ' ' << (output.stop_arc_m ? *output.stop_arc_m : -1.0)
            << ' ' << trajectory.size() << '\n';
  for (const auto &point : trajectory) {
    std::cout << point.x << ' ' << point.y << ' ' << point.z << ' '
              << point.orientation_x << ' ' << point.orientation_y << ' '
              << point.orientation_z << ' ' << point.orientation_w << ' '
              << point.longitudinal_velocity_mps << ' '
              << point.lateral_velocity_mps << ' '
              << point.heading_rate_rps << '\n';
  }
}
