// SPDX-License-Identifier: Apache-2.0
// Export route primitive centerlines and graph successors from Rosless-Lanelet2.
// The offline comparator applies the source RouteHandler's selection policy.

#include "planning/lanelet_bin_adapter.hpp"

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <string_view>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 2 && !(argc == 3 && std::string_view(argv[2]) == "--with-z"))
    return 2;
  const bool with_z = argc == 3;
  std::ifstream input(argv[1], std::ios::binary);
  if (!input) return 3;
  const std::vector<std::uint8_t> map_data(
      std::istreambuf_iterator<char>{input},
      std::istreambuf_iterator<char>{});
  const planning::LaneletBinAdapter map(map_data);
  std::size_t section_count = 0;
  if (!(std::cin >> section_count) || section_count == 0) return 4;
  std::vector<std::vector<std::int64_t>> sections(section_count);
  for (auto &section : sections) {
    std::size_t count = 0;
    if (!(std::cin >> count) || count == 0) return 5;
    section.resize(count);
    for (auto &id : section) if (!(std::cin >> id)) return 6;
  }
  std::cout << std::setprecision(17);
  std::cout << section_count << '\n';
  for (const auto &section : sections) {
    std::cout << section.size() << '\n';
    for (const auto id : section) {
      if (with_z) {
        const auto centerline = map.laneCenterline3d(id);
        std::cout << id << ' ' << centerline.size() << ' '
                  << map.laneSpeedLimit(id) << '\n';
        for (const auto &point : centerline)
          std::cout << point.x << ' ' << point.y << ' ' << point.z << '\n';
      } else {
        const auto centerline = map.laneCenterline(id);
        std::cout << id << ' ' << centerline.size() << ' '
                  << map.laneSpeedLimit(id) << '\n';
        for (const auto &point : centerline)
          std::cout << point.x << ' ' << point.y << '\n';
      }
    }
  }
  for (std::size_t section_index = 0; section_index + 1 < sections.size();
       ++section_index) {
    for (const auto id : sections[section_index]) {
      const auto next = map.followingRoadLaneletId(
          id, sections[section_index + 1]);
      std::cout << id << ' ' << (next ? *next : 0) << '\n';
    }
  }
  return 0;
}
