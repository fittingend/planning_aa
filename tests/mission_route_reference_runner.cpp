#include "planning/lanelet_bin_adapter.hpp"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 2) return 1;
  std::ifstream map_file(argv[1], std::ios::binary);
  if (!map_file) return 2;
  const std::vector<std::uint8_t> data{
      std::istreambuf_iterator<char>(map_file), std::istreambuf_iterator<char>()};
  planning::LaneletBinAdapter adapter(data);
  planning::RouteCheckpoint start, goal;
  if (!(std::cin >> start.x >> start.y >> start.yaw_rad >>
        goal.x >> goal.y >> goal.yaw_rad)) return 3;
  std::vector<planning::RouteCheckpoint> checkpoints{start};
  std::size_t waypoint_count = 0;
  if (std::cin >> waypoint_count) {
    for (std::size_t i = 0; i < waypoint_count; ++i) {
      planning::RouteCheckpoint waypoint;
      if (!(std::cin >> waypoint.x >> waypoint.y >> waypoint.yaw_rad)) return 4;
      checkpoints.push_back(waypoint);
    }
  }
  checkpoints.push_back(goal);
  int consider_no_drivable = 0;
  if (!(std::cin >> consider_no_drivable)) consider_no_drivable = 0;
  if (!(std::cin >> checkpoints.back().z)) checkpoints.back().z = 0.0;
  std::string failure_reason;
  const auto route = adapter.routeThroughCheckpoints(
      checkpoints, consider_no_drivable != 0, &failure_reason);
  if (!route) {
    if (!failure_reason.empty()) std::cerr << failure_reason << '\n';
    std::cout << 0 << '\n';
    return 0;
  }
  std::cout << route->sections.size() << '\n';
  for (const auto &section : route->sections) {
    std::cout << section.preferred_lanelet_id << ' '
              << section.primitive_lanelet_ids.size();
    for (const auto id : section.primitive_lanelet_ids) std::cout << ' ' << id;
    std::cout << '\n';
  }
  std::cout.precision(17);
  std::cout << route->goal_height_m << '\n';
}
