// SPDX-License-Identifier: Apache-2.0
// 기록된 행동 경로 후보의 지도 차로 이탈을 독립 코어에서 확인합니다.

#include "planning/behavior_path_lane_safety.hpp"
#include "planning/lanelet_bin_adapter.hpp"

#include <cstdint>
#include <fstream>
#include <iterator>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 4) return 2;
  std::ifstream map_stream(argv[1], std::ios::binary);
  std::ifstream input(argv[2]);
  std::ofstream output(argv[3]);
  if (!map_stream || !input || !output) return 2;
  const std::vector<std::uint8_t> map_data(
      std::istreambuf_iterator<char>{map_stream},
      std::istreambuf_iterator<char>{});
  planning::LaneletBinAdapter map(map_data);
  std::size_t frame_count = 0;
  input >> frame_count;
  for (std::size_t frame = 0; frame < frame_count; ++frame) {
    std::size_t point_count = 0;
    input >> point_count;
    planning::Trajectory path(point_count);
    for (auto &point : path) {
      input >> point.x >> point.y >> point.orientation_x >>
          point.orientation_y >> point.orientation_z >> point.orientation_w;
    }
    if (!input) return 2;
    const auto polygons = map.lanePolygonsNearPath(path);
    const auto departure = planning::firstBehaviorPathLaneDeparture(
        path, polygons);
    output << polygons.size() << ' ';
    if (departure) {
      output << departure->segment_index << ' '
             << departure->segment_fraction << '\n';
    } else {
      output << "-1 0\n";
    }
  }
  return 0;
}
