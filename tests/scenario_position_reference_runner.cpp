#include "planning/lanelet_bin_adapter.hpp"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  std::ifstream map_file(argv[1], std::ios::binary);
  if (!map_file) return 3;
  const std::vector<std::uint8_t> map_data{
      std::istreambuf_iterator<char>(map_file), std::istreambuf_iterator<char>()};
  planning::LaneletBinAdapter adapter(map_data);
  double ego_x, ego_y, ego_yaw, goal_x, goal_y;
  std::size_t count = 0;
  if (!(std::cin >> ego_x >> ego_y >> ego_yaw >> goal_x >> goal_y >> count)) return 4;
  std::vector<std::int64_t> ids(count);
  for (auto &id : ids) {
    if (!(std::cin >> id)) return 5;
  }
  const auto result = adapter.scenarioPosition(
      ego_x, ego_y, ego_yaw, goal_x, goal_y, ids);
  std::cout << result.in_lane << ' ' << result.goal_in_lane << ' '
            << result.in_parking_lot << ' ' << result.along_lane << '\n';
}
