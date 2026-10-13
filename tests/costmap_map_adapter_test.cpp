// SPDX-License-Identifier: Apache-2.0

#include "planning/costmap_map_adapter.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>

namespace {
void check(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}
}  // namespace

int main(int argc, char **argv) {
  if (argc != 2) return EXIT_FAILURE;
  try {
    std::ifstream file(argv[1], std::ios::binary);
    check(file.good(), "synthetic map unavailable");
    const std::vector<std::uint8_t> bytes{
        std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
    planning::LaneletBinAdapter map(bytes);
    const auto areas = map.costmapAreaPolygons();
    check(!areas.road.empty() && !areas.parking.empty(),
          "road and parking polygons must both be extracted");
    planning::CostmapMapAdapter generator(bytes);
    planning::CostmapGeneratorInput frame;
    frame.ego_x = 2585.016995171108;
    frame.ego_y = 23942.35769895078;
    frame.ego_z = 0.0;
    const auto baseline = generator.tick(frame);
    check(baseline && baseline->occupancy.width > 200 &&
              baseline->occupancy.height > 200,
          "parking lot must activate sample decision costmap");
    frame.points_map_frame.push_back(
        {frame.ego_x, frame.ego_y, 1.0});
    const auto obstacle = generator.tick(frame);
    const auto &grid = obstacle->occupancy;
    const auto x = grid.width - 1 - static_cast<std::size_t>(std::ceil(
        (grid.origin_x + grid.width * grid.resolution_m - frame.ego_x) /
        grid.resolution_m));
    const auto y = grid.height - 1 - static_cast<std::size_t>(std::ceil(
        (grid.origin_y + grid.height * grid.resolution_m - frame.ego_y) /
        grid.resolution_m));
    const auto index = y * grid.width + x;
    check(obstacle->points[index] == 1.0f &&
              obstacle->combined[index] >= baseline->combined[index],
          "map-frame point must appear in parking occupancy");
    frame.ego_x = 100000.0;
    frame.ego_y = 100000.0;
    check(!generator.tick(frame),
          "costmap must stop publishing outside parking lot");
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
